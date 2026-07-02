//! RCP-host support: run the OpenThread stack on this MCU while the 802.15.4
//! radio lives on a *separate* chip (an OpenThread **RCP** — Radio Co-Processor)
//! reached over a UART/SPI link using the **spinel** protocol.
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
//! reused from OpenThread's C is the variable-length "packed-uint" codec (via the
//! tiny `spinel_codec.c` shim); everything else (little-endian scalars, the
//! length-prefixed data blob) is done here.
//!
//! `SpinelRadio` builds and parses only *raw* spinel frames; putting the frame on
//! the wire — HDLC byte-stuffing for a UART, or the 5-byte SPI header protocol
//! for SPI — is the job of the [`SpinelTransport`]. Two transports are provided:
//! [`UartSpinelTransport`] and [`SpiSpinelTransport`].

use core::future::Future;

use embassy_time::{Duration, Timer};

use crate::radio::{Capabilities, Config, MacCapabilities, PsduMeta, Radio, RadioErrorKind};
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
// spinel_codec.c bridge (packed-uint codec + a few structural constants).
// ---------------------------------------------------------------------------

extern "C" {
    fn ot_spinel_uint_encode(buf: *mut u8, cap: usize, value: u32) -> i32;
    fn ot_spinel_uint_decode(buf: *const u8, len: usize, out: *mut u32) -> i32;

    fn ot_spinel_header_flag() -> u8;
    fn ot_spinel_cmd_reset() -> u32;
    fn ot_spinel_cmd_prop_value_get() -> u32;
    fn ot_spinel_cmd_prop_value_set() -> u32;
    fn ot_spinel_cmd_prop_value_is() -> u32;
    fn ot_spinel_reset_stack() -> u32;
    fn ot_spinel_status_reset_begin() -> u32;
    fn ot_spinel_status_reset_end() -> u32;
}

/// Encode a spinel packed-uint into `buf`; returns bytes written (or `None`).
fn spinel_uint_encode(buf: &mut [u8], value: u32) -> Option<usize> {
    // SAFETY: `buf`/`buf.len()` describe a valid writable region.
    let n = unsafe { ot_spinel_uint_encode(buf.as_mut_ptr(), buf.len(), value) };
    (n > 0).then_some(n as usize)
}

/// Decode a spinel packed-uint from `buf`; returns `(value, bytes_consumed)`.
fn spinel_uint_decode(buf: &[u8]) -> Option<(u32, usize)> {
    let mut value = 0u32;
    // SAFETY: `buf`/`buf.len()` describe a valid readable region; `value` is valid.
    let n = unsafe { ot_spinel_uint_decode(buf.as_ptr(), buf.len(), &mut value) };
    (n > 0).then_some((value, n as usize))
}

// ---------------------------------------------------------------------------
// Spinel constants (property/command ids from the spinel spec).
// The structural ones come from the C shim; the property ids below are stable
// and taken from `spinel.h` (mirrored here to keep the driver in one place).
// ---------------------------------------------------------------------------

const PROP_LAST_STATUS: u32 = 0;
const PROP_PROTOCOL_VERSION: u32 = 1;
const PROP_CAPS: u32 = 5;
const PROP_HWADDR: u32 = 8;
const PROP_PHY_ENABLED: u32 = 0x20;
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

pub use spi::{SpiSpinelTransport, SpiTransportError};
pub use uart::{UartSpinelTransport, UartTransportError};

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
    out[0] = unsafe { ot_spinel_header_flag() } | (SPINEL_IID << 4) | (tid & 0x0f);
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

/// Radio capabilities we advertise to OpenThread.
///
/// Minimal on purpose: no hardware MAC-offloading (so OpenThread's `MacRadio`
/// wrapper handles ACKs/filtering/security in software and hands us already
/// secured PSDUs), and CCA is done by the RCP as part of `STREAM_RAW`.
const SPINEL_RADIO_CAPS: u16 = 0;
const SPINEL_RADIO_MAC_CAPS: u16 = 0;

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
    /// The RCP is brought up lazily on the first radio operation (the `Radio`
    /// trait has no async constructor). `None` until the startup handshake runs.
    eui64: Option<[u8; 8]>,
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
        let tid = self.alloc_tid();
        let cmd = unsafe { ot_spinel_cmd_prop_value_set() };

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

        self.await_response(tid).await.map(|_| ())
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
        let cmd = unsafe { ot_spinel_cmd_prop_value_set() };

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
        let cmd = unsafe { ot_spinel_cmd_prop_value_get() };

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
        let cmd_is = unsafe { ot_spinel_cmd_prop_value_is() };

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
        let cmd_is = unsafe { ot_spinel_cmd_prop_value_is() };

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
            let cmd = unsafe { ot_spinel_cmd_reset() };
            let reset_arg = unsafe { ot_spinel_reset_stack() };

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
            let begin = unsafe { ot_spinel_status_reset_begin() };
            let end = unsafe { ot_spinel_status_reset_end() };
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

        // Enable the PHY.
        self.set_prop(PROP_PHY_ENABLED, &[1]).await?;

        Ok(())
    }

    /// Wait for an unsolicited `LAST_STATUS` in `[begin, end)` (the RCP reset
    /// acknowledgement). Best-effort with a timeout.
    async fn wait_reset_status(&mut self, begin: u32, end: u32) -> Result<(), RadioErrorKind> {
        let cmd_is = unsafe { ot_spinel_cmd_prop_value_is() };
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

    const CAPS: Capabilities = Capabilities::from_bits_truncate(SPINEL_RADIO_CAPS);
    const MAC_CAPS: MacCapabilities = MacCapabilities::from_bits_truncate(SPINEL_RADIO_MAC_CAPS);

    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error> {
        self.ensure_init().await?;
        self.flush_config(config).await
    }

    async fn transmit(
        &mut self,
        psdu: &[u8],
        cca: bool,
        _ack_psdu_buf: Option<&mut [u8]>,
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
        payload[n] = 0; // maxCsmaBackoffs (host-MAC does retries)
        n += 1;
        payload[n] = 0; // maxFrameRetries
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

        self.set_prop(PROP_STREAM_RAW, &payload[..n]).await?;

        // The transmit ACK/status arrives as the matched response, already
        // consumed by `set_prop`'s `await_response`. Software MAC (MacRadio)
        // handles ACK reception via a subsequent `receive`, so we report no
        // ACK meta here.
        Ok(None)
    }

    async fn receive(&mut self, psdu_buf: &mut [u8]) -> Result<PsduMeta, Self::Error> {
        self.ensure_init().await?;
        self.ensure_rx_enabled(true).await?;

        let cmd_is = unsafe { ot_spinel_cmd_prop_value_is() };

        loop {
            let frame_len = self.recv_frame(Duration::from_secs(3600)).await?;
            let frame = &self.rx_frame[..frame_len];
            let Some((tid, rcmd, rprop, off)) = spinel_parse_header(frame) else {
                continue;
            };

            // Unsolicited STREAM_RAW notification = a received frame.
            if tid == 0 && rcmd == cmd_is && rprop == PROP_STREAM_RAW {
                let body = &frame[off..];
                if body.len() < 2 {
                    continue;
                }
                let plen = u16::from_le_bytes([body[0], body[1]]) as usize;
                if body.len() < 2 + plen {
                    continue;
                }
                let psdu = &body[2..2 + plen];

                // Metadata following the DATA_WLEN: rssi(i8) + noise(i8) + ...
                let meta_off = 2 + plen;
                let rssi = body.get(meta_off).map(|&b| b as i8);
                let channel = self.config.as_ref().map(|c| c.channel).unwrap_or(0);

                let copy = plen.min(psdu_buf.len());
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
