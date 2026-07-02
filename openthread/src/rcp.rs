//! RCP-host support: run the OpenThread stack on this MCU while the 802.15.4
//! radio lives on a *separate* chip (an OpenThread **RCP** — Radio Co-Processor)
//! reached over a UART/SPI link using the **spinel** protocol.
//!
//! # Status
//!
//! **Design only — not yet implemented.** This module documents the intended
//! API and architecture for the `rcp` feature; the actual `RadioSpinel` driving
//! is a `TODO` (see [`OpenThread::run_rcp`]). Enabling the `rcp` Cargo feature
//! today links the required OpenThread libraries (`openthread-radio-spinel`,
//! `openthread-spinel-rcp`, `openthread-hdlc`) but the host transport bridge
//! below is stubbed.
//!
//! # SoC vs RCP-host vs NCP
//!
//! OpenThread has three deployment roles. This crate supports the first two:
//!
//! - **SoC / local radio** (default, `rcp` off): the 802.15.4 radio is on *this*
//!   MCU. OpenThread's `otPlatRadio*` platform callbacks drive it directly, and
//!   the user supplies a [`crate::Radio`] implementation to
//!   [`OpenThread::run`](crate::OpenThread::run).
//!
//! - **RCP-host** (`rcp` on, *this module*): the radio is a separate chip. This
//!   MCU runs the **full OpenThread stack** and keeps the full OpenThread API
//!   (`otUdp*`, `otIp6*`, `otSrp*`, …); it just reaches the radio remotely. The
//!   `otPlatRadio*` callbacks are serviced by OpenThread's `RadioSpinel` client,
//!   which talks to the RCP over a **transport** the user supplies (see
//!   [`SpinelTransport`]) — *not* a [`crate::Radio`]. This is the
//!   "stack on chip A, radio on chip B over UART" deployment (e.g. an
//!   application MCU + a companion 802.15.4 radio running OpenThread's `ot-rcp`).
//!
//! - **NCP** (*not supported*): the radio chip runs the *full* stack and exposes
//!   it over spinel to a host that does **not** run OpenThread — the host uses OS
//!   IPv6 sockets against a `wpan0`/tun interface plus a daemon (`wpantund` /
//!   `ot-ctl`). That host-side API is not the OpenThread API this crate
//!   provides, so the `openthread-ncp-*` libraries are intentionally not built.
//!
//! # Architecture of the RCP-host path (to implement)
//!
//! ```text
//!   ┌──────────────── this MCU (rcp feature) ─────────────────┐    UART/SPI      ┌─ RCP chip ─┐
//!   │  app ─ otUdp*/otIp6*/otSrp* ─ OpenThread core (mtd/ftd)  │   (spinel)       │  ot-rcp    │
//!   │                       │ otPlatRadio*                     │ ◄──────────────► │  (radio)   │
//!   │                 ot::Spinel::RadioSpinel  ── SpinelInterface ──┐             └────────────┘
//!   │                                                              ▼              │
//!   │                                              user `impl SpinelTransport`    │
//!   └──────────────────────────────────────────────────────────────────────────┘
//! ```
//!
//! Implementing this requires:
//!
//! 1. A C/C++ shim that implements OpenThread's `otPlatRadio*` callbacks by
//!    delegating to a `ot::Spinel::RadioSpinel` instance (cf. OpenThread's
//!    `src/posix/platform/radio.cpp`, which does exactly this over a UART file
//!    descriptor). On `no_std` we provide our own minimal equivalent instead of
//!    the POSIX one.
//! 2. A `SpinelInterface` subclass (C++ virtual class with `SendFrame`,
//!    `WaitForFrame`, `Process`, …) that forwards bytes to/from the Rust
//!    [`SpinelTransport`] the user provides. (Likely a C shim exposing the
//!    transport as function pointers, to avoid subclassing C++ from Rust.)
//! 3. An async pump ([`OpenThread::run_rcp`]) that, in place of the local-radio
//!    [`run_radio`](crate::OpenThread) loop, services `RadioSpinel::Process` and
//!    the transport's RX/TX, alongside the existing alarm + tasklet loops.

use core::future::Future;

use core::cell::UnsafeCell;

use embassy_futures::block_on;
use embassy_futures::select::{select3, Either3};

use crate::ot;

/// Maximum number of TX bytes (HDLC-encoded spinel frames awaiting transmission
/// to the RCP) buffered between the C shim and the async pump.
const RCP_TX_BUF_SIZE: usize = crate::sys::OT_RADIO_FRAME_MAX_SIZE as usize * 4;

/// Size of the buffer used to read a chunk of bytes from the transport.
const RCP_RX_CHUNK: usize = 256;

/// How long each steady-state RX poll waits for transport bytes before the pump
/// loops back to flush TX / advance the state machine / yield. Short, so the
/// pump stays responsive without busy-spinning.
const RCP_RX_POLL_TIMEOUT_US: u64 = 2_000;

/// How long the pump yields between iterations, letting the alarm/tasklet loops
/// (and any other tasks) run.
const RCP_PUMP_YIELD_US: u64 = 500;

/// The byte-stream transport to the remote RCP radio (typically a UART, or SPI).
///
/// In RCP-host mode the user supplies this **instead of** a [`crate::Radio`]:
/// OpenThread's spinel client frames/deframes radio commands over it. The
/// transport is a simple full-duplex byte pipe; spinel + HDLC framing is handled
/// inside OpenThread (`openthread-hdlc` / `openthread-radio-spinel`).
///
/// The [`read`](SpinelTransport::read) / [`write`](SpinelTransport::write)
/// methods deliberately mirror [`embedded_io_async::Read`] /
/// [`embedded_io_async::Write`] (both return the number of bytes transferred),
/// so a `SpinelTransport` can be implemented as a thin layer over any
/// `embedded-io-async` byte stream — e.g. an embassy UART.
///
/// # Note
///
/// This is a **design sketch**; the exact method set may change once the
/// `RadioSpinel`/`SpinelInterface` bridge is implemented (e.g. it may need a
/// "wait for readable with timeout" to map onto `SpinelInterface::WaitForFrame`,
/// and a bus-speed hint for `GetBusSpeed`).
pub trait SpinelTransport {
    /// The transport error type.
    type Error: core::fmt::Debug;

    /// Write some bytes of a spinel/HDLC frame to the RCP, returning the number
    /// of bytes written (`>= 1`). Like [`embedded_io_async::Write::write`], a
    /// short write is allowed; the caller loops to send the rest.
    fn write(&mut self, bytes: &[u8]) -> impl Future<Output = Result<usize, Self::Error>>;

    /// Read bytes from the RCP into `buf`, returning the number read (`>= 1`).
    /// Resolves when at least one byte is available. Mirrors
    /// [`embedded_io_async::Read::read`].
    fn read(&mut self, buf: &mut [u8]) -> impl Future<Output = Result<usize, Self::Error>>;

    /// Nominal bus speed in bits/sec (used by OpenThread to size spinel
    /// timeouts). E.g. the UART baud rate. A reasonable default may be provided
    /// once wired up.
    fn bus_speed(&self) -> u32;
}

impl<T> SpinelTransport for &mut T
where
    T: SpinelTransport + ?Sized,
{
    type Error = T::Error;

    fn write(&mut self, bytes: &[u8]) -> impl Future<Output = Result<usize, Self::Error>> {
        T::write(self, bytes)
    }

    fn read(&mut self, buf: &mut [u8]) -> impl Future<Output = Result<usize, Self::Error>> {
        T::read(self, buf)
    }

    fn bus_speed(&self) -> u32 {
        T::bus_speed(self)
    }
}

// ---------------------------------------------------------------------------
// C shim bridge (see `openthread-sys/gen/support/src/rcp_shim.cpp`).
// ---------------------------------------------------------------------------

extern "C" {
    /// Construct + initialize the spinel stack (interface -> driver -> radio).
    /// Must be called before `otInstanceInitSingle()`. Drives the reset
    /// handshake, which calls back into `otRcpHostPumpRx`.
    fn otRcpInit(
        bus_speed: u32,
        reset_radio: bool,
        skip_compatibility_check: bool,
    ) -> crate::sys::otError;

    /// Feed bytes received from the transport into the spinel stack (HDLC decode).
    fn otRcpReceive(buf: *const u8, len: u16);

    /// Run one non-blocking iteration of the spinel processing.
    fn otRcpProcess();

    /// Tear the spinel stack down.
    fn otRcpDeinit();
}

/// The transport-I/O operations the bridge needs, type-erased so the global
/// bridge can hold them without knowing the concrete `SpinelTransport`.
///
/// Both methods are **synchronous** because they are (also) invoked from the C
/// shim inside `SpinelInterface::WaitForFrame`, which runs on a synchronous C
/// call stack (`RadioSpinel::WaitResponse`) and cannot `.await`. The single
/// implementor (installed by `run_rcp`) services them by `block_on`-ing the
/// async transport — the async-runtime analog of the blocking `select()` that
/// OpenThread's POSIX `HdlcInterface` uses. This is sound only because `run_rcp`
/// is the sole driver of the transport on a single-threaded executor.
trait RcpIo {
    /// Read a chunk from the transport, waiting up to `timeout_us`, and feed any
    /// received bytes into the spinel stack (`otRcpReceive`). Returns `true` if
    /// at least one byte was received.
    fn pump_rx(&mut self, timeout_us: u64) -> bool;

    /// Write all currently-queued outgoing bytes (see [`RcpBridge::tx`]) to the
    /// transport.
    fn flush_tx(&mut self);
}

/// State shared between the C shim (calling `otRcpHostEnqueueTx` /
/// `otRcpHostPumpRx`) and the async pump in [`crate::OpenThread::run_rcp`].
///
/// A single `OpenThread` instance may exist at a time (guarded by `OT_REFCNT`),
/// so a single global instance of this bridge is sufficient. The `io` hook is
/// installed for the entire lifetime of `run_rcp` (both bring-up and steady
/// state), because `WaitForFrame` — hence RX pumping — is needed for every
/// synchronous spinel request/response, not only during setup.
struct RcpBridge {
    /// Outgoing (HDLC-encoded) bytes queued by the shim's `SendFrame`, drained
    /// by [`RcpIo::flush_tx`] and written to the transport.
    tx: heapless::Vec<u8, RCP_TX_BUF_SIZE>,
    /// The type-erased transport-I/O hook installed by `run_rcp`. `None` before
    /// `run_rcp` installs it. It is the *sole* owner of the transport, so all
    /// transport reads/writes (from both the shim's `WaitForFrame` and the async
    /// pump) funnel through it — there is never a second borrow of the transport.
    io: Option<*mut dyn RcpIo>,
}

impl RcpBridge {
    const fn new() -> Self {
        Self {
            tx: heapless::Vec::new(),
            io: None,
        }
    }
}

// SAFETY: single-threaded, bare-metal target; access is serialized by the fact
// that OpenThread's C code and the pump run on the same executor/thread. Mirrors
// `OT_ACTIVE_STATE` in `platform.rs`.
struct SyncBridge(UnsafeCell<RcpBridge>);
unsafe impl Sync for SyncBridge {}

static RCP_BRIDGE: SyncBridge = SyncBridge(UnsafeCell::new(RcpBridge::new()));

#[allow(clippy::mut_from_ref)]
fn rcp_bridge() -> &'static mut RcpBridge {
    // SAFETY: see `SyncBridge` — single-threaded access.
    unsafe { &mut *RCP_BRIDGE.0.get() }
}

/// Called by the C shim to enqueue an outgoing (HDLC-encoded) spinel frame.
#[no_mangle]
extern "C" fn otRcpHostEnqueueTx(buf: *const u8, len: u16) {
    if buf.is_null() || len == 0 {
        return;
    }

    // SAFETY: the shim passes a valid `len`-byte buffer for the duration of the call.
    let bytes = unsafe { core::slice::from_raw_parts(buf, len as usize) };

    let bridge = rcp_bridge();
    if bridge.tx.extend_from_slice(bytes).is_err() {
        warn!("RCP TX buffer overflow; dropping {} bytes", len);
    }
}

/// Called by the C shim (from `SpinelInterface::WaitForFrame`) to synchronously
/// pump the transport for up to `timeout_us`. Delegates to the transport-I/O
/// hook installed by `run_rcp` (which `block_on`s the async read).
#[no_mangle]
extern "C" fn otRcpHostPumpRx(timeout_us: u64) -> bool {
    match rcp_bridge().io {
        // SAFETY: the pointer targets the `RcpIo` value on `run_rcp`'s stack. It
        // is installed before use and cleared before that value is dropped, and
        // is only dereferenced synchronously (from the C shim, or from the pump)
        // while `run_rcp` is live. There is never a concurrent second borrow: the
        // shim only calls this from within `otRcpProcess`/instance init, which the
        // pump invokes between its own (non-overlapping) uses of the hook.
        Some(io) => unsafe { (*io).pump_rx(timeout_us) },
        None => false,
    }
}

impl crate::OpenThread<'_> {
    /// Run the OpenThread stack in **RCP-host** mode, driving a remote radio over
    /// `transport` (spinel), instead of a local [`crate::Radio`].
    ///
    /// Mirrors [`OpenThread::run`](crate::OpenThread::run), but replaces the
    /// local-radio loop with the spinel ↔ `RadioSpinel` pump. The alarm and
    /// tasklet loops are shared with the SoC path.
    ///
    /// In `rcp` builds the OpenThread instance is *not* initialized by
    /// [`OpenThread::new`](crate::OpenThread::new) — `otInstanceInitSingle` drives
    /// the `otPlatRadio*` callbacks which perform synchronous spinel exchanges
    /// with the remote radio, so it can only run once the transport is live. This
    /// method therefore initializes the spinel stack and then the OpenThread
    /// instance before entering the pump loop; the instance becomes operational
    /// only after `run_rcp` is first polled.
    ///
    /// This future never returns.
    pub async fn run_rcp<T>(&self, transport: T) -> !
    where
        T: SpinelTransport,
    {
        let bus_speed = transport.bus_speed();

        // The single owner of the transport. All transport I/O — from both the C
        // shim's synchronous `WaitForFrame` (via `otRcpHostPumpRx`) and the async
        // pump below — funnels through this one object, so the transport is never
        // borrowed twice. It is installed into the global bridge for the entire
        // lifetime of `run_rcp`.
        let mut io = TransportIo { transport };

        // SAFETY: `io` lives for the rest of this `!`-returning function, and the
        // erased pointer is only dereferenced synchronously (from the shim or the
        // pump, never concurrently — see `otRcpHostPumpRx`). Since `run_rcp` never
        // returns, the pointer never dangles.
        {
            let hook: *mut dyn RcpIo = &mut io;
            let hook: *mut (dyn RcpIo + 'static) = unsafe { core::mem::transmute(hook) };
            rcp_bridge().io = Some(hook);
        }

        // Bring up the spinel stack (interface -> driver -> radio, incl. the RCP
        // reset handshake) *before* initializing the OpenThread instance. Both
        // steps drive `WaitForFrame` -> `otRcpHostPumpRx` -> `io.pump_rx`, which is
        // why the hook is installed first.
        let init = ot!(unsafe { otRcpInit(bus_speed, true, false) })
            // The instance init drives `otPlatRadio*` -> `RadioSpinel`, so it must
            // run only now that the radio platform is live. Deferred from `new`
            // under the `rcp` feature.
            .and_then(|()| self.init());

        if let Err(e) = init {
            panic!("RCP-host init failed: {:?}", e);
        }

        // Steady state: the spinel transport pump plus the shared alarm and
        // tasklet loops. `io` is a local that lives to the end of this
        // never-returning function, so the hook pointer installed above stays
        // valid for the whole run.
        let mut spinel = core::pin::pin!(self.run_spinel());
        let mut alarm = core::pin::pin!(self.run_alarm());
        let mut tasklets = core::pin::pin!(self.run_tasklets());

        match select3(&mut spinel, &mut alarm, &mut tasklets).await {
            Either3::First(r) | Either3::Second(r) | Either3::Third(r) => r,
        }
    }

    /// The steady-state spinel pump.
    ///
    /// It does not touch the transport directly — that is owned by the installed
    /// [`RcpIo`] hook in the global bridge. Each iteration flushes any queued TX,
    /// pumps a chunk of RX, advances the spinel + OpenThread state machines
    /// (`otRcpProcess` + tasklets), then yields briefly so the alarm and tasklet
    /// loops (and any other tasks) can run. RX is *also* pumped on demand by the
    /// shim's `WaitForFrame` during synchronous requests; this loop keeps things
    /// moving when nothing is being awaited (e.g. delivering unsolicited RX
    /// frames and transmit-done notifications).
    async fn run_spinel(&self) -> ! {
        loop {
            // SAFETY: single-threaded; the hook is installed for the whole
            // `run_rcp` lifetime before this loop starts, and these calls do not
            // overlap the shim's (also-synchronous) use of the same hook.
            if let Some(hook) = rcp_bridge().io {
                unsafe {
                    (*hook).flush_tx();
                    (*hook).pump_rx(RCP_RX_POLL_TIMEOUT_US);
                    otRcpProcess();
                }
            }

            self.activate().process_tasklets();

            // Yield so the alarm/tasklet loops (and any other tasks) can run.
            embassy_time::Timer::after(embassy_time::Duration::from_micros(RCP_PUMP_YIELD_US))
                .await;
        }
    }
}

/// The single owner of the [`SpinelTransport`], implementing [`RcpIo`] by
/// `block_on`-ing the async transport (see [`RcpIo`] for why this must be
/// synchronous).
struct TransportIo<T> {
    transport: T,
}

impl<T: SpinelTransport> RcpIo for TransportIo<T> {
    fn pump_rx(&mut self, timeout_us: u64) -> bool {
        let mut buf = [0u8; RCP_RX_CHUNK];

        let read = block_on(async {
            let timeout =
                embassy_time::Timer::after(embassy_time::Duration::from_micros(timeout_us));

            match embassy_futures::select::select(self.transport.read(&mut buf), timeout).await {
                embassy_futures::select::Either::First(res) => res.ok(),
                embassy_futures::select::Either::Second(()) => None,
            }
        });

        match read {
            Some(n) if n > 0 => {
                // SAFETY: `otRcpReceive` copies out of `buf` synchronously.
                unsafe { otRcpReceive(buf.as_ptr(), n as u16) };
                true
            }
            _ => false,
        }
    }

    fn flush_tx(&mut self) {
        loop {
            // Take the queued bytes out of the bridge so we don't hold its borrow
            // across the (block_on'd) write.
            let mut chunk: heapless::Vec<u8, RCP_TX_BUF_SIZE> = heapless::Vec::new();
            {
                let bridge = rcp_bridge();
                if bridge.tx.is_empty() {
                    break;
                }
                let _ = chunk.extend_from_slice(&bridge.tx);
                bridge.tx.clear();
            }

            block_on(async {
                let mut off = 0;
                while off < chunk.len() {
                    match self.transport.write(&chunk[off..]).await {
                        Ok(n) if n > 0 => off += n,
                        _ => break,
                    }
                }
            });
        }
    }
}
