//! RCP-host support: run the OpenThread stack on this MCU while the 802.15.4
//! radio lives on a *separate* chip (an OpenThread **RCP** — Radio Co-Processor)
//! reached over a UART/SPI link using the **spinel** protocol.
//!
//! # ⚠️ Not yet hardware-tested
//!
//! **This entire module is unverified against real hardware.** [`SpinelRadio`]
//! and both transports ([`UartSpinelTransport`] and [`SpiSpinelTransport`]) have
//! so far only been *compile*-checked — no exchange with an actual `ot-rcp` has
//! been run. The spinel/HDLC/SPI framing is implemented to the OpenThread
//! reference, but timing, flow-control edge cases, and the startup handshake may
//! not behave correctly on a physical link. Treat this as experimental until
//! validated end-to-end with an RCP dongle.
//!
//! # Design
//!
//! Unlike OpenThread's POSIX host — which drives the radio with the C++
//! `RadioSpinel` client (a synchronous, mainloop-blocking `WaitForFrame` model)
//! — this crate exposes the remote radio as an ordinary [`crate::Radio`]
//! implementation: [`SpinelRadio`]. The user hands it to the *same*
//! [`OpenThread::run`](crate::OpenThread::run) as a local (SoC) radio; the
//! generic async radio loop (`run_radio` + `MacRadio`) drives it. There is no
//! separate run loop, no blocking, and the `otPlatRadio*` platform layer is the
//! standard SoC one — a `SpinelRadio` is just another `Radio` driver, exactly
//! like [`crate::ProxyRadio`], only the "other side" is a chip on a wire.
//!
//! This is possible because the [`crate::Radio`] trait already operates at the
//! raw-PHY level: MAC-layer security is performed by OpenThread's core before a
//! frame reaches the radio (see the [`crate::Radio`] docs), so `SpinelRadio`
//! transmits already-secured PSDUs and never needs the RCP's MAC keys.
//!
//! # Wire protocol
//!
//! `SpinelRadio` speaks the spinel wire protocol directly, so it works against a
//! **stock, unmodified `ot-rcp` firmware**. A spinel frame is a header byte, a
//! variable-length command, a variable-length property key, and a payload; the
//! config setters map to `PROP_VALUE_SET`s that are flushed just before a
//! transmit / receive, and TX/RX map to the `STREAM_RAW` property. The only piece
//! reused from OpenThread's C is the variable-length "packed-uint" codec
//! (`spinel_packed_uint_*`, bound directly from `spinel.h` under the `rcp`
//! feature); everything else (little-endian scalars, the length-prefixed data
//! blob) is done here.
//!
//! `SpinelRadio` builds and parses only *raw* spinel frames; putting the frame on
//! the wire — HDLC byte-stuffing for a UART, or the 5-byte SPI header protocol
//! for SPI — is the job of the [`SpinelTransport`]. Two transports are provided:
//! [`UartSpinelTransport`] and [`SpiSpinelTransport`].

use core::future::Future;

use embassy_time::{Duration, Timer};

use crate::radio::{
    Capabilities, Config, MacCapabilities, PsduMeta, Radio, RadioCaps, RadioErrorKind,
};
use crate::sys::OT_RADIO_FRAME_MAX_SIZE;

// ---------------------------------------------------------------------------
// SpinelTransport: the user-provided *frame* pipe to the RCP.
// ---------------------------------------------------------------------------

/// A framed transport to the remote RCP radio: it sends and receives **one
/// complete raw spinel frame** at a time (header byte + packed command + packed
/// property + payload — see the module docs).
///
/// # Why frame-oriented (not a byte stream)
///
/// Spinel needs *some* way to delimit frames on the wire, but the mechanism is
/// transport-specific, so it belongs *inside* the transport rather than in
/// [`SpinelRadio`]:
///
/// - Over a **UART** (a raw byte stream with no framing) the frames are HDLC
///   byte-stuffed (RFC 1662). That is what [`UartSpinelTransport`] does.
/// - Over **SPI** the frames are *not* HDLC-framed: each SPI transaction carries
///   a 5-byte header whose length field delimits the payload, so the spinel
///   frame is carried raw. That is what [`SpiSpinelTransport`] does — and it also
///   handles SPI's lack of a peripheral-initiated channel via an interrupt line.
///
/// So `SpinelRadio` builds and parses only *raw* spinel frames and is agnostic to
/// framing; each transport frames however its wire requires.
pub trait SpinelTransport {
    /// The transport error type.
    type Error: core::fmt::Debug;

    /// Send one complete raw spinel `frame` to the RCP. Resolves once the frame
    /// has been handed to the wire.
    fn send(&mut self, frame: &[u8]) -> impl Future<Output = Result<(), Self::Error>>;

    /// Receive one complete raw spinel frame from the RCP into `buf`, returning
    /// its length. Resolves when a whole frame is available; the caller races it
    /// against its own timeout.
    fn recv(&mut self, buf: &mut [u8]) -> impl Future<Output = Result<usize, Self::Error>>;
}

impl<T> SpinelTransport for &mut T
where
    T: SpinelTransport + ?Sized,
{
    type Error = T::Error;

    fn send(&mut self, frame: &[u8]) -> impl Future<Output = Result<(), Self::Error>> {
        T::send(self, frame)
    }

    fn recv(&mut self, buf: &mut [u8]) -> impl Future<Output = Result<usize, Self::Error>> {
        T::recv(self, buf)
    }
}

// ---------------------------------------------------------------------------
// Spinel packed-uint (variable-length int) codec.
//
// The command id and property key are spinel "packed unsigned ints". Everything
// else in a spinel frame is a plain little-endian scalar or a length-prefixed
// blob, built/parsed directly below — but the packed-uint encoding is non-trivial
// (7 bits/byte, MSB continuation), so we reuse OpenThread's own C codec
// (`spinel_packed_uint_*` in `spinel.c`, part of `libopenthread-spinel-rcp.a`).
// These bindings are generated by bindgen from `lib/spinel/spinel.h`, gated on
// the `rcp` feature (see `openthread-sys/gen/{include/include_rcp.h,builder.rs}`).
// ---------------------------------------------------------------------------

use crate::sys::{spinel_packed_uint_decode, spinel_packed_uint_encode};

/// Encode a spinel packed-uint into `buf`; returns bytes written (or `None`).
fn spinel_uint_encode(buf: &mut [u8], value: u32) -> Option<usize> {
    // SAFETY: `buf`/`buf.len()` describe a valid writable region.
    let n = unsafe { spinel_packed_uint_encode(buf.as_mut_ptr(), buf.len() as _, value as _) };
    (n > 0).then_some(n as usize)
}

/// Decode a spinel packed-uint from `buf`; returns `(value, bytes_consumed)`.
fn spinel_uint_decode(buf: &[u8]) -> Option<(u32, usize)> {
    let mut value = 0;
    // SAFETY: `buf`/`buf.len()` describe a valid readable region; `value` is valid.
    let n = unsafe { spinel_packed_uint_decode(buf.as_ptr(), buf.len() as _, &mut value) };
    (n > 0).then_some((value as u32, n as usize))
}

// ---------------------------------------------------------------------------
// Spinel property ids (from the spinel spec).
//
// The structural command/status/header constants are bound directly from
// `spinel.h` (see the `crate::sys::SPINEL_*` re-typed consts below). The property
// ids here are stable spec values, mirrored so the driver reads in one place.
// ---------------------------------------------------------------------------

const PROP_LAST_STATUS: u32 = 0;
const PROP_PROTOCOL_VERSION: u32 = 1;
const PROP_CAPS: u32 = 5;
const PROP_HWADDR: u32 = 8;
const PROP_PHY_ENABLED: u32 = 0x20;
/// `SPINEL_PROP_RADIO_CAPS` — the RCP's `otRadioCaps` bitmask (packed-uint).
const PROP_RADIO_CAPS: u32 = 0x1207;
const PROP_PHY_CHAN: u32 = 0x21;
const PROP_PHY_TX_POWER: u32 = 0x25;
const PROP_MAC_15_4_LADDR: u32 = 0x34;
const PROP_MAC_15_4_SADDR: u32 = 0x35;
const PROP_MAC_15_4_PANID: u32 = 0x36;
const PROP_MAC_RAW_STREAM_ENABLED: u32 = 0x37;
const PROP_MAC_PROMISCUOUS_MODE: u32 = 0x38;
const PROP_STREAM_RAW: u32 = 0x71;

/// The RCP capability ids we require (a real RCP in raw-MAC mode).
const CAP_CONFIG_RADIO: u32 = 34;
const CAP_MAC_RAW: u32 = 513;

/// The single interface id we use (non-multipan host).
const SPINEL_IID: u8 = 0;

// Structural spinel constants, from the (bindgen-generated) `crate::sys`
// bindings of `spinel.h`. Re-typed to the `u32`/`u8` this driver uses (bindgen
// gives them the C enum/macro repr).
const HEADER_FLAG: u8 = crate::sys::SPINEL_HEADER_FLAG as u8;
const CMD_RESET: u32 = crate::sys::SPINEL_CMD_RESET as u32;
const CMD_PROP_VALUE_GET: u32 = crate::sys::SPINEL_CMD_PROP_VALUE_GET as u32;
const CMD_PROP_VALUE_SET: u32 = crate::sys::SPINEL_CMD_PROP_VALUE_SET as u32;
const CMD_PROP_VALUE_IS: u32 = crate::sys::SPINEL_CMD_PROP_VALUE_IS as u32;
const RESET_STACK: u32 = crate::sys::SPINEL_RESET_STACK as u32;
const STATUS_RESET_BEGIN: u32 = crate::sys::SPINEL_STATUS_RESET__BEGIN as u32;
const STATUS_RESET_END: u32 = crate::sys::SPINEL_STATUS_RESET__END as u32;

/// Response timeout for a spinel request.
const RESPONSE_TIMEOUT: Duration = Duration::from_millis(1000);

/// Max on-the-wire spinel frame (pre-HDLC) we build/parse.
const MAX_SPINEL_FRAME: usize = OT_RADIO_FRAME_MAX_SIZE as usize + 128;

// ---------------------------------------------------------------------------
// Transports: putting a raw spinel frame on a concrete wire.
//
// `SpinelRadio` is transport-agnostic; each of these implements
// `SpinelTransport` for a specific bus and owns that bus's framing (HDLC for a
// UART, the 5-byte header protocol for SPI).
// ---------------------------------------------------------------------------

pub mod spi;
pub mod uart;

pub use spi::{IntPolarity, SpiSpinelTransport, SpiTransportError};
pub use uart::{UartSpinelTransport, UartTransportError};

/// Host serial device (`std` feature): an async serial byte stream over a
/// `/dev/tty*` device, ready to wrap in a [`UartSpinelTransport`] to drive an
/// `ot-rcp` from a Linux/macOS host over USB. See [`serial::SerialPort`].
#[cfg(feature = "std")]
pub mod serial;
#[cfg(feature = "std")]
pub use serial::SerialPort;

// ---------------------------------------------------------------------------
// Spinel frame build/parse.
// ---------------------------------------------------------------------------

/// Build a spinel frame header + command + property into `out`, returning the
/// number of bytes written (the caller then appends the payload).
fn spinel_frame_prefix(out: &mut [u8], tid: u8, cmd: u32, prop: u32) -> Option<usize> {
    let mut n = 0;
    if out.is_empty() {
        return None;
    }
    // Header: FLAG | (iid << 4) | tid.
    // SAFETY: FFI call, no pointers.
    out[0] = HEADER_FLAG | (SPINEL_IID << 4) | (tid & 0x0f);
    n += 1;

    n += spinel_uint_encode(&mut out[n..], cmd)?;
    n += spinel_uint_encode(&mut out[n..], prop)?;
    Some(n)
}

/// Parse the header of an incoming spinel frame: returns `(tid, cmd, prop,
/// payload_offset)`.
fn spinel_parse_header(frame: &[u8]) -> Option<(u8, u32, u32, usize)> {
    if frame.is_empty() {
        return None;
    }
    let tid = frame[0] & 0x0f;
    let mut off = 1;

    let (cmd, n) = spinel_uint_decode(&frame[off..])?;
    off += n;
    let (prop, n) = spinel_uint_decode(&frame[off..])?;
    off += n;

    Some((tid, cmd, prop, off))
}

/// Parse a spinel radio frame body (the payload of a `STREAM_RAW`
/// `PROP_VALUE_IS`, used both for a received frame and for the ACK reported in a
/// transmit-done status). Returns the PSDU slice and its RSSI.
///
/// Layout (from OpenThread's `RadioSpinel::ParseRadioFrame`):
/// `DATA_WLEN(psdu) + i8 rssi + i8 noise + u16 flags + ...`; we take the PSDU and
/// the RSSI (the fields after are LQI/timestamp/rx-error, unused here).
fn parse_radio_frame(body: &[u8]) -> Option<(&[u8], Option<i8>)> {
    if body.len() < 2 {
        return None;
    }
    let plen = u16::from_le_bytes([body[0], body[1]]) as usize;
    if body.len() < 2 + plen {
        return None;
    }
    let psdu = &body[2..2 + plen];
    let rssi = body.get(2 + plen).map(|&b| b as i8);
    Some((psdu, rssi))
}

/// A tiny set of outstanding spinel transaction ids (1..=15), stored as a
/// bitmask. Used to drain the acknowledgements of a pipelined burst of
/// `PROP_VALUE_SET`s, matching each ack to its request by TID regardless of
/// arrival order.
#[derive(Clone, Copy, Default)]
struct TidSet(u16);

impl TidSet {
    const fn new() -> Self {
        Self(0)
    }

    fn insert(&mut self, tid: u8) {
        self.0 |= 1 << (tid & 0x0f);
    }

    fn remove(&mut self, tid: u8) {
        self.0 &= !(1 << (tid & 0x0f));
    }

    fn contains(&self, tid: u8) -> bool {
        self.0 & (1 << (tid & 0x0f)) != 0
    }

    fn is_empty(&self) -> bool {
        self.0 == 0
    }
}

// ---------------------------------------------------------------------------
// SpinelRadio: a `Radio` over a spinel transport.
// ---------------------------------------------------------------------------

/// The PHY capabilities we advertise to OpenThread.
///
/// These are the capabilities a raw-MAC `ot-rcp` provides *as part of its
/// `STREAM_RAW` transmit contract*: it performs CSMA/CA backoff, per-frame
/// automatic retransmission, and the ACK-timeout wait for us (we drive them via
/// the `csmaCaEnabled` / `maxCsmaBackoffs` / `maxFrameRetries` fields of the
/// transmit payload). We advertise only this fixed, guaranteed subset — the
/// *variable* PHY caps a specific RCP may additionally have (e.g.
/// `TRANSMIT_SEC`, precise TX/RX timing) are reported by the RCP at runtime via
/// `PROP_RADIO_CAPS`, which the `Radio` trait's compile-time `const CAPS` cannot
/// yet carry (see the `CAPS` impl for the follow-up). Under-claiming here is
/// safe: OpenThread performs any unclaimed capability in software.
const SPINEL_RADIO_CAPS: Capabilities = Capabilities::ACK_TIMEOUT
    .union(Capabilities::CSMA_BACKOFF)
    .union(Capabilities::TRANSMIT_RETRIES);

/// The MAC-offload capabilities we advertise to OpenThread.
///
/// Unlike the PHY caps, these are **not** guessed: they are guaranteed by the
/// `CAP_MAC_RAW` capability that [`SpinelRadio::ensure_init`] already requires
/// from the RCP. A raw-MAC RCP always hardware-filters received frames by PAN
/// ID / short / extended address (we push those filters via the `MAC_15_4_*ADDR`
/// properties) and handles 802.15.4 acknowledgements autonomously — sending the
/// ACK for a received frame (`RX_ACK`) and reporting the received ACK of a
/// transmitted frame (`TX_ACK`, surfaced by [`SpinelRadio::transmit`]). So
/// OpenThread's `MacRadio` software fallback is not needed for any of these.
const SPINEL_RADIO_MAC_CAPS: MacCapabilities = MacCapabilities::FILTER_PAN_ID
    .union(MacCapabilities::FILTER_SHORT_ADDR)
    .union(MacCapabilities::FILTER_EXT_ADDR)
    .union(MacCapabilities::TX_ACK)
    .union(MacCapabilities::RX_ACK);

/// A [`crate::Radio`] implementation that drives a remote 802.15.4 radio (an
/// OpenThread RCP) over a [`SpinelTransport`] using the spinel protocol.
///
/// Hand it to [`OpenThread::run`](crate::OpenThread::run) exactly like a local
/// radio, wrapping the wire in the matching [`SpinelTransport`]:
///
/// ```ignore
/// let transport = UartSpinelTransport::new(uart);   // or SpiSpinelTransport::new(spi, int)
/// let radio = SpinelRadio::new(transport);
/// ot.run(radio).await
/// ```
pub struct SpinelRadio<T> {
    transport: T,
    /// The RCP is brought up on the first radio operation (or eagerly via
    /// [`Radio::init`]). `None` until the startup handshake runs.
    eui64: Option<[u8; 8]>,
    /// The PHY capabilities read from the RCP's `PROP_RADIO_CAPS` during the
    /// handshake. Until the handshake runs it holds the fixed baseline
    /// ([`SPINEL_RADIO_CAPS`]); afterwards it is the RCP's reported set.
    caps: Capabilities,
    /// Last-applied config; used to only re-send changed properties.
    config: Option<Config>,
    /// Whether raw-stream (RX) is currently enabled on the RCP.
    rx_enabled: bool,
    /// Next transaction id (1..=15, 0 is reserved for unsolicited notifications).
    next_tid: u8,
    /// Scratch buffer for the raw spinel frame being built for transmission.
    tx_frame: [u8; MAX_SPINEL_FRAME],
    /// The most recently received raw spinel frame, and its length. Response
    /// parsers borrow `rx_frame[..rx_len]` after a `recv_frame`.
    rx_frame: [u8; MAX_SPINEL_FRAME],
    rx_len: usize,
}

impl<T> SpinelRadio<T>
where
    T: SpinelTransport,
{
    /// Create a new `SpinelRadio` over `transport`. The RCP is initialized on the
    /// first radio operation.
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            eui64: None,
            caps: SPINEL_RADIO_CAPS,
            config: None,
            rx_enabled: false,
            next_tid: 1,
            tx_frame: [0; MAX_SPINEL_FRAME],
            rx_frame: [0; MAX_SPINEL_FRAME],
            rx_len: 0,
        }
    }

    fn alloc_tid(&mut self) -> u8 {
        let tid = self.next_tid;
        self.next_tid = if self.next_tid >= 15 {
            1
        } else {
            self.next_tid + 1
        };
        tid
    }

    /// Send an already-built raw spinel `frame` to the transport (which frames it
    /// for its wire — HDLC for UART, the SPI header protocol for SPI).
    async fn send_frame(&mut self, frame: &[u8]) -> Result<(), RadioErrorKind> {
        self.transport
            .send(frame)
            .await
            .map_err(|_| RadioErrorKind::TxFailed)
    }

    /// Receive one complete raw spinel frame into `self.rx_frame`, or fail on
    /// `timeout`. Returns the frame length (also stashed in `self.rx_len`);
    /// callers parse `self.rx_frame[..len]`.
    async fn recv_frame(&mut self, timeout: Duration) -> Result<usize, RadioErrorKind> {
        let recv_fut = self.transport.recv(&mut self.rx_frame);
        let mut recv_fut = core::pin::pin!(recv_fut);
        let mut timeout_fut = core::pin::pin!(Timer::after(timeout));

        match embassy_futures::select::select(&mut recv_fut, &mut timeout_fut).await {
            embassy_futures::select::Either::First(r) => {
                let len = r.map_err(|_| RadioErrorKind::RxFailed)?;
                self.rx_len = len;
                Ok(len)
            }
            embassy_futures::select::Either::Second(()) => Err(RadioErrorKind::RxFailed),
        }
    }

    /// Send a `PROP_VALUE_SET` with a raw payload and await its echoed
    /// `PROP_VALUE_IS` acknowledgement (matched by TID).
    async fn set_prop(&mut self, prop: u32, payload: &[u8]) -> Result<(), RadioErrorKind> {
        self.send_prop_await(prop, payload).await.map(|_| ())
    }

    /// Send a `PROP_VALUE_SET` with a raw payload and await its matched
    /// `PROP_VALUE_IS` response, returning `(prop, payload_offset)` into
    /// `self.rx_frame` (length `self.rx_len`). Most callers only need the ack and
    /// use [`Self::set_prop`]; `transmit` uses this to read the transmit-done
    /// body (status + ACK frame).
    async fn send_prop_await(
        &mut self,
        prop: u32,
        payload: &[u8],
    ) -> Result<(u32, usize), RadioErrorKind> {
        let tid = self.alloc_tid();
        let cmd = CMD_PROP_VALUE_SET;

        let frame_len = {
            let mut n = spinel_frame_prefix(&mut self.tx_frame, tid, cmd, prop)
                .ok_or(RadioErrorKind::TxFailed)?;
            if n + payload.len() > self.tx_frame.len() {
                return Err(RadioErrorKind::TxFailed);
            }
            self.tx_frame[n..n + payload.len()].copy_from_slice(payload);
            n += payload.len();
            n
        };

        // Copy the frame out of `self.tx_frame` so we don't hold the borrow.
        let mut frame = [0u8; MAX_SPINEL_FRAME];
        frame[..frame_len].copy_from_slice(&self.tx_frame[..frame_len]);
        self.send_frame(&frame[..frame_len]).await?;

        self.await_response(tid).await
    }

    /// Pipeline a batch of `PROP_VALUE_SET`s: send every frame back-to-back
    /// *without* awaiting acks between them, then drain all their
    /// acknowledgements (matched by TID, in any order). This collapses N property
    /// writes from N serialized round-trips into a single send-burst + a single
    /// ack-drain — one round-trip of latency instead of N.
    ///
    /// Each spinel frame still carries exactly one property (the wire protocol has
    /// no multi-set command), and each SET is still individually acknowledged
    /// (the per-op ack is intrinsic to the lossy, independently-resettable RCP
    /// link — it is the *serialization* between them, not the acks, that this
    /// removes). Sending the frames back-to-back also lets a framed transport
    /// (e.g. the SPI header protocol) coalesce them into as few bus transfers as
    /// possible.
    ///
    /// `props` is an iterator of `(prop_key, payload)`; an empty batch is a no-op.
    async fn set_props<'p>(
        &mut self,
        props: impl IntoIterator<Item = (u32, &'p [u8])>,
    ) -> Result<(), RadioErrorKind> {
        let cmd = CMD_PROP_VALUE_SET;

        let mut pending = TidSet::new();

        for (prop, payload) in props {
            let tid = self.alloc_tid();

            // Build the raw spinel frame (header | cmd | prop | payload) into a
            // small local scratch — these config SET frames are tiny.
            let mut raw = [0u8; 32];
            let mut n =
                spinel_frame_prefix(&mut raw, tid, cmd, prop).ok_or(RadioErrorKind::TxFailed)?;
            if n + payload.len() > raw.len() {
                return Err(RadioErrorKind::TxFailed);
            }
            raw[n..n + payload.len()].copy_from_slice(payload);
            n += payload.len();

            // Send it now, but do NOT await its ack — keep the pipeline full.
            self.send_frame(&raw[..n]).await?;
            pending.insert(tid);
        }

        if pending.is_empty() {
            return Ok(());
        }

        self.drain_acks(pending).await
    }

    /// Send a `PROP_VALUE_GET` and await the response frame (matched by TID),
    /// invoking `f` with the response's property payload.
    async fn get_prop<R>(
        &mut self,
        prop: u32,
        f: impl FnOnce(&[u8]) -> R,
    ) -> Result<R, RadioErrorKind> {
        let tid = self.alloc_tid();
        let cmd = CMD_PROP_VALUE_GET;

        let frame_len = spinel_frame_prefix(&mut self.tx_frame, tid, cmd, prop)
            .ok_or(RadioErrorKind::TxFailed)?;
        let mut frame = [0u8; MAX_SPINEL_FRAME];
        frame[..frame_len].copy_from_slice(&self.tx_frame[..frame_len]);
        self.send_frame(&frame[..frame_len]).await?;

        let (_prop, off) = self.await_response(tid).await?;
        let len = self.rx_len;
        Ok(f(&self.rx_frame[off..len]))
    }

    /// Await a single response frame with a matching `tid`, dispatching any
    /// unsolicited (`tid == 0`) frames received meanwhile. Returns `(prop,
    /// payload_offset)` into `self.rx_frame` (whose length is `self.rx_len`).
    async fn await_response(&mut self, tid: u8) -> Result<(u32, usize), RadioErrorKind> {
        let cmd_is = CMD_PROP_VALUE_IS;

        loop {
            let frame_len = self.recv_frame(RESPONSE_TIMEOUT).await?;

            let frame = &self.rx_frame[..frame_len];
            let Some((rtid, rcmd, rprop, off)) = spinel_parse_header(frame) else {
                continue;
            };

            if rtid == tid && rcmd == cmd_is {
                return Ok((rprop, off));
            }
            // Unsolicited (tid 0) or mismatched — ignore here; the RX path polls
            // for STREAM_RAW notifications separately.
        }
    }

    /// Drain the acknowledgements for a *set* of outstanding transaction ids,
    /// one `PROP_VALUE_IS` per TID (arriving in any order, possibly coalesced in
    /// a single transport read — important for SPI). Returns once every TID in
    /// `pending` has been acknowledged, or errors on the response timeout.
    ///
    /// The value payloads of the acks are ignored (these are the echoed SET
    /// confirmations); only their arrival is required. Unsolicited (`tid == 0`)
    /// and unrelated frames seen meanwhile are dropped, matching
    /// [`Self::await_response`].
    async fn drain_acks(&mut self, mut pending: TidSet) -> Result<(), RadioErrorKind> {
        let cmd_is = CMD_PROP_VALUE_IS;

        while !pending.is_empty() {
            let frame_len = self.recv_frame(RESPONSE_TIMEOUT).await?;

            let frame = &self.rx_frame[..frame_len];
            let Some((rtid, rcmd, _rprop, _off)) = spinel_parse_header(frame) else {
                continue;
            };

            if rcmd == cmd_is && pending.contains(rtid) {
                pending.remove(rtid);
            }
        }

        Ok(())
    }

    /// Run the RCP startup handshake once: reset, verify it is a raw-MAC RCP,
    /// read the EUI-64, enable the PHY.
    async fn ensure_init(&mut self) -> Result<(), RadioErrorKind> {
        if self.eui64.is_some() {
            return Ok(());
        }

        // Software reset → wait for the RCP's reset status notification.
        {
            let tid = 0; // reset uses tid 0 in OT; the reply is an unsolicited status
            let cmd = CMD_RESET;
            let reset_arg = RESET_STACK;

            let mut n = spinel_frame_prefix(&mut self.tx_frame, tid, cmd, reset_arg)
                .ok_or(RadioErrorKind::Other)?;
            // RESET is a bare command; the "prop" slot above already carried the
            // reset kind as a packed-uint argument. Truncate any stray bytes.
            let frame_len = n.min(self.tx_frame.len());
            n = frame_len;
            let mut frame = [0u8; MAX_SPINEL_FRAME];
            frame[..n].copy_from_slice(&self.tx_frame[..n]);
            self.send_frame(&frame[..n]).await?;

            // Wait for a LAST_STATUS in the reset range (best-effort).
            let begin = STATUS_RESET_BEGIN;
            let end = STATUS_RESET_END;
            let _ = self.wait_reset_status(begin, end).await;
        }

        // Read the spinel protocol version (major.minor packed-uints). We only
        // require that the RCP answers; a mismatch would surface later as a
        // capability/prop error. This also confirms the post-reset link is live.
        let major = self
            .get_prop(PROP_PROTOCOL_VERSION, |payload| {
                spinel_uint_decode(payload).map(|(v, _)| v).unwrap_or(0)
            })
            .await?;
        if major == 0 {
            return Err(RadioErrorKind::Other);
        }

        // Verify capabilities: must be a radio-config RCP with raw MAC.
        let (has_config_radio, has_mac_raw) = self
            .get_prop(PROP_CAPS, |payload| {
                let mut off = 0;
                let mut cfg = false;
                let mut raw = false;
                while off < payload.len() {
                    if let Some((cap, n)) = spinel_uint_decode(&payload[off..]) {
                        if cap == CAP_CONFIG_RADIO {
                            cfg = true;
                        }
                        if cap == CAP_MAC_RAW {
                            raw = true;
                        }
                        off += n;
                    } else {
                        break;
                    }
                }
                (cfg, raw)
            })
            .await?;

        if !has_config_radio || !has_mac_raw {
            return Err(RadioErrorKind::Other);
        }

        // Read the RCP's EUI-64.
        let eui64 = self
            .get_prop(PROP_HWADDR, |payload| {
                let mut e = [0u8; 8];
                if payload.len() >= 8 {
                    e.copy_from_slice(&payload[..8]);
                }
                e
            })
            .await?;
        self.eui64 = Some(eui64);

        // Read the RCP's PHY capabilities (`otRadioCaps` bitmask). This is the
        // authoritative, per-device PHY cap set — reported at runtime, which is
        // why the `Radio` trait's compile-time `const CAPS` cannot carry it and
        // [`Radio::init`] returns it instead. We keep any bits our fixed baseline
        // guarantees even if a minimal RCP under-reports.
        let caps_bits = self
            .get_prop(PROP_RADIO_CAPS, |payload| {
                spinel_uint_decode(payload).map(|(v, _)| v).unwrap_or(0)
            })
            .await?;
        self.caps = Capabilities::from_bits_truncate(caps_bits as u16) | SPINEL_RADIO_CAPS;

        // Enable the PHY.
        self.set_prop(PROP_PHY_ENABLED, &[1]).await?;

        Ok(())
    }

    /// Wait for an unsolicited `LAST_STATUS` in `[begin, end)` (the RCP reset
    /// acknowledgement). Best-effort with a timeout.
    async fn wait_reset_status(&mut self, begin: u32, end: u32) -> Result<(), RadioErrorKind> {
        let cmd_is = CMD_PROP_VALUE_IS;
        loop {
            let frame_len = self.recv_frame(RESPONSE_TIMEOUT).await?;
            let frame = &self.rx_frame[..frame_len];
            let Some((_tid, rcmd, rprop, off)) = spinel_parse_header(frame) else {
                continue;
            };
            if rcmd == cmd_is && rprop == PROP_LAST_STATUS {
                if let Some((status, _)) = spinel_uint_decode(&frame[off..]) {
                    if (begin..end).contains(&status) {
                        return Ok(());
                    }
                }
            }
        }
    }

    /// Flush the config to the RCP: send only the properties that changed since
    /// the last flush, pipelined as a single burst (one round-trip regardless of
    /// how many properties changed — see [`Self::set_props`]).
    async fn flush_config(&mut self, config: &Config) -> Result<(), RadioErrorKind> {
        let prev = self.config.clone();
        let changed = |get: fn(&Config) -> u64| prev.as_ref().map(get) != Some(get(config));

        // Materialize each changed property's little-endian payload into a local
        // so its slice stays valid for the whole burst, then stage the
        // `(prop, payload)` pairs. At most six properties, so a fixed array +
        // length avoids any allocation.
        let chan = [config.channel];
        let power = [config.power as u8];
        let promisc = [config.promiscuous as u8];
        let pan_id = config.pan_id.unwrap_or(0xffff).to_le_bytes();
        let short_addr = config.short_addr.unwrap_or(0xffff).to_le_bytes();
        let ext_addr = config.ext_addr.unwrap_or(0).to_le_bytes();

        let mut batch: [(u32, &[u8]); 6] = [(0, &[]); 6];
        let mut count = 0;

        if changed(|c| c.channel as u64) {
            batch[count] = (PROP_PHY_CHAN, &chan);
            count += 1;
        }
        if changed(|c| c.power as u8 as u64) {
            batch[count] = (PROP_PHY_TX_POWER, &power);
            count += 1;
        }
        if changed(|c| c.promiscuous as u64) {
            batch[count] = (PROP_MAC_PROMISCUOUS_MODE, &promisc);
            count += 1;
        }
        if changed(|c| c.pan_id.unwrap_or(0xffff) as u64) {
            batch[count] = (PROP_MAC_15_4_PANID, &pan_id);
            count += 1;
        }
        if changed(|c| c.short_addr.unwrap_or(0xffff) as u64) {
            batch[count] = (PROP_MAC_15_4_SADDR, &short_addr);
            count += 1;
        }
        if changed(|c| c.ext_addr.unwrap_or(0)) {
            batch[count] = (PROP_MAC_15_4_LADDR, &ext_addr);
            count += 1;
        }

        self.set_props(batch[..count].iter().copied()).await?;

        self.config = Some(config.clone());
        Ok(())
    }

    /// Ensure raw-stream RX is enabled (so the RCP forwards received frames).
    async fn ensure_rx_enabled(&mut self, enabled: bool) -> Result<(), RadioErrorKind> {
        if self.rx_enabled != enabled {
            self.set_prop(PROP_MAC_RAW_STREAM_ENABLED, &[enabled as u8])
                .await?;
            self.rx_enabled = enabled;
        }
        Ok(())
    }
}

impl<T> Radio for SpinelRadio<T>
where
    T: SpinelTransport,
{
    type Error = RadioErrorKind;

    async fn init(&mut self) -> Result<RadioCaps, Self::Error> {
        // Run the startup handshake (idempotent) and report the RCP's discovered
        // capabilities: the PHY set from the RCP's `PROP_RADIO_CAPS`, and the MAC
        // offload set guaranteed by the raw-MAC contract (`ensure_init` requires
        // `CAP_MAC_RAW`). The hot paths still call `ensure_init` defensively, so a
        // radio used without an eager `init` (or one whose eager init failed)
        // still recovers.
        //
        // A future RCP that reports *additional* MAC offload at runtime (e.g.
        // hardware crypto — a `TRANSMIT_SEC`-style capability) would union it in
        // here from the relevant spinel property; the const is only the baseline.
        self.ensure_init().await?;
        Ok(RadioCaps {
            phy: self.caps,
            mac: SPINEL_RADIO_MAC_CAPS,
        })
    }

    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error> {
        self.ensure_init().await?;
        self.flush_config(config).await
    }

    async fn transmit(
        &mut self,
        psdu: &[u8],
        cca: bool,
        ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Self::Error> {
        self.ensure_init().await?;

        let channel = self.config.as_ref().map(|c| c.channel).unwrap_or(11);
        let tx_power = self.config.as_ref().map(|c| c.power).unwrap_or(0);

        // Build the STREAM_RAW transmit payload:
        //   data-with-len(psdu) + channel + maxCsmaBackoffs + maxFrameRetries
        //   + csmaCaEnabled + isHeaderUpdated + isARetx + isSecurityProcessed
        //   + txDelay(u32) + txDelayBaseTime(u32) + rxChannelAfterTxDone + txPower(i8)
        let mut payload = [0u8; MAX_SPINEL_FRAME];
        let mut n = 0;

        // DATA_WLEN: uint16-LE length prefix + bytes.
        let plen = psdu.len() as u16;
        payload[n..n + 2].copy_from_slice(&plen.to_le_bytes());
        n += 2;
        payload[n..n + psdu.len()].copy_from_slice(psdu);
        n += psdu.len();

        payload[n] = channel;
        n += 1;
        // Let the RCP do CSMA/CA backoff and frame retries — we advertise
        // `CSMA_BACKOFF` + `TRANSMIT_RETRIES`, so OpenThread expects the radio to
        // handle them. 802.15.4 defaults (macMaxCSMABackoffs=4, macMaxFrameRetries=3).
        payload[n] = 4; // maxCsmaBackoffs
        n += 1;
        payload[n] = 3; // maxFrameRetries
        n += 1;
        payload[n] = cca as u8; // csmaCaEnabled
        n += 1;
        payload[n] = 1; // isHeaderUpdated (OT core secured the frame)
        n += 1;
        payload[n] = 0; // isARetx
        n += 1;
        payload[n] = 1; // isSecurityProcessed (security done host-side)
        n += 1;
        payload[n..n + 4].copy_from_slice(&0u32.to_le_bytes()); // txDelay
        n += 4;
        payload[n..n + 4].copy_from_slice(&0u32.to_le_bytes()); // txDelayBaseTime
        n += 4;
        payload[n] = channel; // rxChannelAfterTxDone
        n += 1;
        payload[n] = tx_power as u8;
        n += 1;

        // Send STREAM_RAW and take the matched transmit-done response.
        let (_prop, off) = self.send_prop_await(PROP_STREAM_RAW, &payload[..n]).await?;

        // Parse the transmit-done body (from `RadioSpinel::HandleTransmitDone`):
        //   uint_packed status + bool framePending + bool headerUpdated
        //   + [if status OK] the ACK radio frame (same layout as an RX frame).
        // We advertise `TX_ACK`, so OpenThread expects us to return the received
        // ACK here rather than have `MacRadio` synthesize it in software.
        let body_end = self.rx_len;
        let body = &self.rx_frame[off..body_end];

        let Some((status, mut p)) = spinel_uint_decode(body) else {
            return Ok(None);
        };
        // status != OK → the transmit failed (no ACK / channel access). Report as
        // no ACK; OpenThread maps a missing ACK to the appropriate retry/failure.
        let status_ok = status == 0; // SPINEL_STATUS_OK
        if !status_ok {
            return Ok(None);
        }
        // Skip framePending (bool, 1 byte) + headerUpdated (bool, 1 byte).
        if body.len() < p + 2 {
            return Ok(None);
        }
        p += 2;

        // The remaining bytes are the ACK radio frame (if any was received).
        let Some((ack_psdu, ack_rssi)) = parse_radio_frame(&body[p..]) else {
            return Ok(None);
        };

        match ack_psdu_buf {
            Some(buf) => {
                let copy = ack_psdu.len().min(buf.len());
                buf[..copy].copy_from_slice(&ack_psdu[..copy]);
                Ok(Some(PsduMeta {
                    len: copy,
                    channel,
                    rssi: ack_rssi,
                }))
            }
            // The caller didn't ask for the ACK PSDU (didn't expect an ACK), so
            // there is nothing to report even though the transmit succeeded.
            None => Ok(None),
        }
    }

    async fn receive(&mut self, psdu_buf: &mut [u8]) -> Result<PsduMeta, Self::Error> {
        self.ensure_init().await?;
        self.ensure_rx_enabled(true).await?;

        let cmd_is = CMD_PROP_VALUE_IS;

        loop {
            let frame_len = self.recv_frame(Duration::from_secs(3600)).await?;
            let frame = &self.rx_frame[..frame_len];
            let Some((tid, rcmd, rprop, off)) = spinel_parse_header(frame) else {
                continue;
            };

            // Unsolicited STREAM_RAW notification = a received frame.
            if tid == 0 && rcmd == cmd_is && rprop == PROP_STREAM_RAW {
                let Some((psdu, rssi)) = parse_radio_frame(&frame[off..]) else {
                    continue;
                };
                let channel = self.config.as_ref().map(|c| c.channel).unwrap_or(0);

                let copy = psdu.len().min(psdu_buf.len());
                psdu_buf[..copy].copy_from_slice(&psdu[..copy]);

                return Ok(PsduMeta {
                    len: copy,
                    channel,
                    rssi,
                });
            }
            // Other frames (matched responses to a concurrent op, status) — ignore.
        }
    }
}
