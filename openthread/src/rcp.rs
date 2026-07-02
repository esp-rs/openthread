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
//! **stock, unmodified `ot-rcp` firmware**. Frames are HDLC-framed (RFC 1662,
//! implemented here) and carry a spinel command targeting a property; the config
//! setters map to `PROP_VALUE_SET`s that are flushed just before a transmit /
//! receive, and TX/RX map to the `STREAM_RAW` property. The only piece reused
//! from OpenThread's C is the variable-length "packed-uint" codec (via the tiny
//! `spinel_codec.c` shim); everything else (little-endian scalars, the
//! length-prefixed data blob, the HDLC framing) is done here.

use core::future::Future;

use embassy_time::{Duration, Timer};

use crate::radio::{Capabilities, Config, MacCapabilities, PsduMeta, Radio, RadioErrorKind};
use crate::sys::OT_RADIO_FRAME_MAX_SIZE;

// ---------------------------------------------------------------------------
// SpinelTransport: the user-provided byte pipe to the RCP.
// ---------------------------------------------------------------------------

/// The byte-stream transport to the remote RCP radio (typically a UART, or SPI).
///
/// The [`read`](SpinelTransport::read) / [`write`](SpinelTransport::write)
/// methods mirror [`embedded_io_async::Read`] / [`embedded_io_async::Write`]
/// (both return the number of bytes transferred), so a `SpinelTransport` is a
/// thin layer over any `embedded-io-async` byte stream — e.g. an embassy UART.
pub trait SpinelTransport {
    /// The transport error type.
    type Error: core::fmt::Debug;

    /// Write some bytes to the RCP, returning the number written (`>= 1`). A
    /// short write is allowed; the caller loops to send the rest.
    fn write(&mut self, bytes: &[u8]) -> impl Future<Output = Result<usize, Self::Error>>;

    /// Read bytes from the RCP into `buf`, returning the number read (`>= 1`).
    /// Resolves when at least one byte is available.
    fn read(&mut self, buf: &mut [u8]) -> impl Future<Output = Result<usize, Self::Error>>;
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
// HDLC framing (RFC 1662 byte-stuffing + CRC-16/X.25 FCS).
// ---------------------------------------------------------------------------

const HDLC_FLAG: u8 = 0x7e;
const HDLC_ESCAPE: u8 = 0x7d;
const HDLC_XOR: u8 = 0x20;
const HDLC_FCS_INIT: u16 = 0xffff;
const HDLC_FCS_GOOD: u16 = 0xf0b8;

/// CRC-16/X.25 (reflected 0x1021) FCS, one byte at a time.
fn hdlc_fcs_update(fcs: u16, byte: u8) -> u16 {
    let mut fcs = fcs;
    let mut b = byte;
    for _ in 0..8 {
        if ((fcs ^ (b as u16)) & 0x0001) != 0 {
            fcs = (fcs >> 1) ^ 0x8408;
        } else {
            fcs >>= 1;
        }
        b >>= 1;
    }
    fcs
}

fn hdlc_needs_escape(byte: u8) -> bool {
    matches!(byte, HDLC_FLAG | HDLC_ESCAPE) || byte == 0x11 || byte == 0x13
}

/// HDLC-encode `payload` into `out`, returning the encoded length (or `None` if
/// `out` is too small). Wraps with flags, byte-stuffs, and appends the FCS.
fn hdlc_encode(payload: &[u8], out: &mut [u8]) -> Option<usize> {
    let mut n = 0;

    let push = |out: &mut [u8], n: &mut usize, byte: u8| -> bool {
        if hdlc_needs_escape(byte) {
            if *n + 2 > out.len() {
                return false;
            }
            out[*n] = HDLC_ESCAPE;
            out[*n + 1] = byte ^ HDLC_XOR;
            *n += 2;
        } else {
            if *n + 1 > out.len() {
                return false;
            }
            out[*n] = byte;
            *n += 1;
        }
        true
    };

    if n >= out.len() {
        return None;
    }
    out[n] = HDLC_FLAG;
    n += 1;

    let mut fcs = HDLC_FCS_INIT;
    for &b in payload {
        fcs = hdlc_fcs_update(fcs, b);
        if !push(out, &mut n, b) {
            return None;
        }
    }

    // Append the (complemented) FCS, little-endian, byte-stuffed.
    fcs ^= 0xffff;
    if !push(out, &mut n, (fcs & 0xff) as u8) {
        return None;
    }
    if !push(out, &mut n, (fcs >> 8) as u8) {
        return None;
    }

    if n >= out.len() {
        return None;
    }
    out[n] = HDLC_FLAG;
    n += 1;

    Some(n)
}

/// Incremental HDLC decoder: fed bytes from the transport, yields complete,
/// FCS-checked spinel frames.
struct HdlcDecoder {
    buf: [u8; MAX_SPINEL_FRAME],
    len: usize,
    /// The payload length of the most recently completed frame (set when
    /// `push` returns `Some`), so callers can re-borrow `buf[..len_snapshot]`.
    len_snapshot: usize,
    fcs: u16,
    escaped: bool,
    in_frame: bool,
}

impl HdlcDecoder {
    const fn new() -> Self {
        Self {
            buf: [0; MAX_SPINEL_FRAME],
            len: 0,
            len_snapshot: 0,
            fcs: HDLC_FCS_INIT,
            escaped: false,
            in_frame: false,
        }
    }

    fn reset(&mut self) {
        self.len = 0;
        self.fcs = HDLC_FCS_INIT;
        self.escaped = false;
        self.in_frame = true;
    }

    /// Push one received byte. Returns `Some(len)` when a complete, valid frame
    /// is available in `self.buf[..len - 2]` (payload, FCS stripped).
    fn push(&mut self, byte: u8) -> Option<usize> {
        match byte {
            HDLC_FLAG => {
                let complete = self.in_frame && self.len >= 2 && self.fcs == HDLC_FCS_GOOD;

                let payload_len = self.len.saturating_sub(2);
                self.reset();

                complete.then_some(payload_len)
            }
            HDLC_ESCAPE => {
                self.escaped = true;
                None
            }
            _ => {
                if !self.in_frame {
                    return None;
                }
                let b = if self.escaped {
                    self.escaped = false;
                    byte ^ HDLC_XOR
                } else {
                    byte
                };
                if self.len < self.buf.len() {
                    self.buf[self.len] = b;
                    self.len += 1;
                    self.fcs = hdlc_fcs_update(self.fcs, b);
                } else {
                    // Overflow — drop the frame.
                    self.in_frame = false;
                }
                None
            }
        }
    }
}

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
/// radio:
///
/// ```ignore
/// let radio = SpinelRadio::new(uart);   // `uart: impl SpinelTransport`
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
    /// Scratch buffers.
    tx_frame: [u8; MAX_SPINEL_FRAME],
    hdlc_buf: [u8; MAX_SPINEL_FRAME * 2],
    decoder: HdlcDecoder,
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
            hdlc_buf: [0; MAX_SPINEL_FRAME * 2],
            decoder: HdlcDecoder::new(),
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

    /// Write an already-built spinel frame (HDLC-encode + transport write).
    async fn send_frame(&mut self, frame: &[u8]) -> Result<(), RadioErrorKind> {
        let n = hdlc_encode(frame, &mut self.hdlc_buf).ok_or(RadioErrorKind::TxFailed)?;

        let mut off = 0;
        while off < n {
            let written = self
                .transport
                .write(&self.hdlc_buf[off..n])
                .await
                .map_err(|_| RadioErrorKind::TxFailed)?;
            if written == 0 {
                return Err(RadioErrorKind::TxFailed);
            }
            off += written;
        }
        Ok(())
    }

    /// Read + HDLC-decode until a complete spinel frame arrives, or the timeout
    /// elapses. Returns the decoded frame length; the frame is in
    /// `self.decoder.buf`.
    async fn recv_frame(&mut self, timeout: Duration) -> Result<usize, RadioErrorKind> {
        let mut rx = [0u8; 128];
        let mut timeout_fut = core::pin::pin!(Timer::after(timeout));

        loop {
            let read = {
                let read_fut = self.transport.read(&mut rx);
                let mut read_fut = core::pin::pin!(read_fut);
                match embassy_futures::select::select(&mut read_fut, &mut timeout_fut).await {
                    embassy_futures::select::Either::First(r) => {
                        r.map_err(|_| RadioErrorKind::RxFailed)?
                    }
                    embassy_futures::select::Either::Second(()) => {
                        return Err(RadioErrorKind::RxFailed)
                    }
                }
            };

            for &b in &rx[..read] {
                if let Some(len) = self.decoder.push(b) {
                    return Ok(len);
                }
            }
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
        let len = self.decoder.len_snapshot;
        Ok(f(&self.decoder.buf[off..len]))
    }

    /// Await a response frame with a matching `tid`, dispatching any unsolicited
    /// (`tid == 0`) frames received meanwhile. Returns `(prop, payload_offset)`.
    async fn await_response(&mut self, tid: u8) -> Result<(u32, usize), RadioErrorKind> {
        let cmd_is = unsafe { ot_spinel_cmd_prop_value_is() };

        loop {
            let frame_len = self.recv_frame(RESPONSE_TIMEOUT).await?;
            self.decoder.len_snapshot = frame_len;

            let frame = &self.decoder.buf[..frame_len];
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
            let frame = &self.decoder.buf[..frame_len];
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
    /// the last flush.
    async fn flush_config(&mut self, config: &Config) -> Result<(), RadioErrorKind> {
        let prev = self.config.clone();
        let changed = |get: fn(&Config) -> u64| prev.as_ref().map(get) != Some(get(config));

        if changed(|c| c.channel as u64) {
            self.set_prop(PROP_PHY_CHAN, &[config.channel]).await?;
        }
        if changed(|c| c.power as u8 as u64) {
            self.set_prop(PROP_PHY_TX_POWER, &[config.power as u8])
                .await?;
        }
        if changed(|c| c.promiscuous as u64) {
            self.set_prop(PROP_MAC_PROMISCUOUS_MODE, &[config.promiscuous as u8])
                .await?;
        }
        if changed(|c| c.pan_id.unwrap_or(0xffff) as u64) {
            let p = config.pan_id.unwrap_or(0xffff);
            self.set_prop(PROP_MAC_15_4_PANID, &p.to_le_bytes()).await?;
        }
        if changed(|c| c.short_addr.unwrap_or(0xffff) as u64) {
            let s = config.short_addr.unwrap_or(0xffff);
            self.set_prop(PROP_MAC_15_4_SADDR, &s.to_le_bytes()).await?;
        }
        if changed(|c| c.ext_addr.unwrap_or(0)) {
            let e = config.ext_addr.unwrap_or(0);
            self.set_prop(PROP_MAC_15_4_LADDR, &e.to_le_bytes()).await?;
        }

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
            let frame = &self.decoder.buf[..frame_len];
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
