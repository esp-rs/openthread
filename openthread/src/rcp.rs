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

impl crate::OpenThread<'_> {
    /// Run the OpenThread stack in **RCP-host** mode, driving a remote radio over
    /// `transport` (spinel), instead of a local [`crate::Radio`].
    ///
    /// Mirrors [`OpenThread::run`](crate::OpenThread::run), but replaces the
    /// local-radio loop with the spinel ↔ `RadioSpinel` pump. The alarm and
    /// tasklet loops are shared with the SoC path.
    ///
    /// # Status
    ///
    /// **Unimplemented.** See the module docs for the implementation plan.
    #[cfg(feature = "rcp")]
    pub async fn run_rcp<T>(&self, _transport: T) -> !
    where
        T: SpinelTransport,
    {
        // TODO:
        //  1. Construct a `ot::Spinel::RadioSpinel` via a C shim, wiring its
        //     `SpinelInterface` to `_transport` (SendFrame/WaitForFrame/Process).
        //  2. Call its `Init(...)` (skipping the local-radio platform init that
        //     the SoC path performs).
        //  3. Replace `run_radio` with a loop servicing `RadioSpinel::Process`
        //     and `_transport` RX/TX; keep `run_alarm` + `run_tasklets`.
        unimplemented!(
            "RCP-host mode (`run_rcp`) is not yet implemented; see the `rcp` module docs"
        )
    }
}
