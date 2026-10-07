//! RCP-host support: run the OpenThread stack on this MCU while the 802.15.4
//! radio lives on a *separate* chip (an OpenThread **RCP** — Radio Co-Processor)
//! reached over a UART/SPI link using the **spinel** protocol.
//!
//! # Hardware validation status
//!
//! The **UART** path ([`SpinelRadio`] + [`UartSpinelTransport`] + [`SerialPort`])
//! is validated end-to-end against a real `ot-rcp` (an nRF52840 dongle over USB
//! CDC-ACM): the host example resets and handshakes the RCP, loads an operational
//! dataset, and drives MLE. The **SPI** transport ([`SpiSpinelTransport`]) is
//! still *compile*-checked only — its full-duplex `accept_len`/`data_len`
//! negotiation and interrupt handling are implemented to spec but not yet
//! observed on a physical SPI link.
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
//! raw-PHY level: MAC-layer security is normally performed by OpenThread's core
//! before a frame reaches the radio (see the [`crate::Radio`] docs), so
//! `SpinelRadio` transmits already-secured PSDUs. Only with a CSL role compiled
//! in (an FTD parenting CSL children) does the RCP secure frames and enhanced
//! ACKs itself, with the MAC keys handed down through [`Radio::set_mac_keys`].
//!
//! # CSL
//!
//! An FTD on a `SpinelRadio` parents CSL children the way OpenThread's own
//! RCP hosts do: the RCP times the frames into the children's receive windows
//! (`TRANSMIT_TIMING`), timestamps received frames, and sends secured enhanced
//! ACKs. The radio clock ([`RadioCaps::clock`]) is the RCP's: its frame
//! timestamps and transmit times cross the link untouched, so the schedule of
//! a CSL child - its last frame's timestamp plus whole CSL periods - is kept in
//! the clock the RCP transmits by. Only "now" is an estimate (the host clock
//! plus the offset to the RCP clock, measured every
//! [`TIME_SYNC_INTERVAL`]), which merely decides how early a frame is handed
//! over. That clock is a plain function, reading a process-wide offset: one
//! `SpinelRadio` per process.
//!
//! A CSL *child* is not possible on an RCP: spinel has no property to hand the
//! RCP the CSL schedule it would have to advertise in its enhanced ACKs (the
//! reference POSIX host does not implement `otPlatRadioEnableCsl` either), so
//! `RECEIVE_TIMING` is never reported.
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
use core::mem::MaybeUninit;

use core::cell::Cell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};

use crate::radio::{
    AckSecurity, Capabilities, Config, MacCapabilities, MacKeys, PsduRxInfo, Radio, RadioCaps,
    RadioClock, RadioErrorKind, SrcMatchConfig,
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
const PROP_MAC_SCAN_STATE: u32 = 0x30;
const PROP_MAC_SCAN_MASK: u32 = 0x31;
const PROP_MAC_SCAN_PERIOD: u32 = 0x32;
const PROP_MAC_ENERGY_SCAN_RESULT: u32 = 0x39;
/// `SPINEL_PROP_RADIO_CAPS` — the RCP's `otRadioCaps` bitmask (packed-uint).
const PROP_RADIO_CAPS: u32 = 0x120b;
const PROP_PHY_CHAN: u32 = 0x21;
const PROP_PHY_TX_POWER: u32 = 0x25;
/// `SPINEL_PROP_PHY_CCA_THRESHOLD` — the RCP's CCA energy-detect threshold, in dBm (int8).
const PROP_PHY_CCA_THRESHOLD: u32 = 0x24;
/// `SPINEL_PROP_PHY_RX_SENSITIVITY` — the RCP's receive sensitivity in dBm.
const PROP_PHY_RX_SENSITIVITY: u32 = 0x27;
const PROP_MAC_15_4_LADDR: u32 = 0x34;
const PROP_MAC_15_4_SADDR: u32 = 0x35;
/// `SPINEL_PROP_MAC_15_4_ALT_SADDR` (`MAC__BEGIN + 12`) — a *second* short
/// address the RCP's hardware filter should also accept. Only meaningful when the
/// RCP advertises `OT_RADIO_CAPS_ALT_SHORT_ADDR`; sent only in that case.
const PROP_MAC_15_4_ALT_SADDR: u32 = 0x3c;
const PROP_MAC_15_4_PANID: u32 = 0x36;
const PROP_MAC_RAW_STREAM_ENABLED: u32 = 0x37;
const PROP_MAC_PROMISCUOUS_MODE: u32 = 0x38;
/// `SPINEL_PROP_MAC_RX_ON_WHEN_IDLE_MODE` — keep the receiver on between TX/RX
/// so the RCP hears asynchronous traffic (MLE, parent responses). The wire
/// value is the negation of [`Config::auto_sleep`].
const PROP_MAC_RX_ON_WHEN_IDLE_MODE: u32 = 0x3b;
/// The RCP's source-match table (`SPINEL_PROP_MAC_SRC_MATCH_*`): whether the
/// ACKs answering data polls take their Frame Pending bit from the table, and
/// the table's short/extended entries.
const PROP_MAC_SRC_MATCH_ENABLED: u32 = 0x1303;
const PROP_MAC_SRC_MATCH_SHORT_ADDRESSES: u32 = 0x1304;
const PROP_MAC_SRC_MATCH_EXTENDED_ADDRESSES: u32 = 0x1305;
const PROP_STREAM_RAW: u32 = 0x71;
/// `SPINEL_PROP_RCP_MAC_KEY`: the MAC keys the RCP secures frames and enhanced
/// ACKs with.
const PROP_RCP_MAC_KEY: u32 = 0x800;
/// `SPINEL_PROP_RCP_MAC_FRAME_COUNTER`: the RCP's MAC frame counter.
const PROP_RCP_MAC_FRAME_COUNTER: u32 = 0x801;
/// `SPINEL_PROP_RCP_TIMESTAMP`: the RCP's clock, in microseconds.
const PROP_RCP_TIMESTAMP: u32 = 0x802;
/// `SPINEL_PROP_RCP_CSL_ACCURACY`: the RCP clock's accuracy, in PPM.
const PROP_RCP_CSL_ACCURACY: u32 = 0x804;
/// `SPINEL_PROP_RCP_CSL_UNCERTAINTY`: the RCP's timing uncertainty, in 10 µs.
const PROP_RCP_CSL_UNCERTAINTY: u32 = 0x805;

/// `SPINEL_STATUS_NO_ACK`: a transmitted frame was not acknowledged.
const STATUS_NO_ACK: u32 = 17;
/// `SPINEL_STATUS_CCA_FAILURE`: a frame was not sent, the channel was busy.
const STATUS_CCA_FAILURE: u32 = 18;
/// `SPINEL_STATUS_STACK_NATIVE__BEGIN`: the RCP's own `otError`s are reported
/// as this plus the error.
const STATUS_STACK_NATIVE_BEGIN: u32 = 15_360;

/// The error a failed transmission reports, for the status the RCP finished
/// it with - as the reference host maps it (`SpinelStatusToOtError`): what
/// OpenThread then does (retransmit, retry in the next CSL window, count a
/// link failure) depends on it, so a failure must never pass for a success.
fn tx_status_error(status: u32) -> RadioErrorKind {
    const NATIVE_NO_ACK: u32 = STATUS_STACK_NATIVE_BEGIN + crate::sys::otError_OT_ERROR_NO_ACK;
    const NATIVE_CHANNEL_ACCESS_FAILURE: u32 =
        STATUS_STACK_NATIVE_BEGIN + crate::sys::otError_OT_ERROR_CHANNEL_ACCESS_FAILURE;

    match status {
        STATUS_NO_ACK | NATIVE_NO_ACK => RadioErrorKind::RxAckTimeout,
        STATUS_CCA_FAILURE | NATIVE_CHANNEL_ACCESS_FAILURE => RadioErrorKind::TxFailed,
        _ => RadioErrorKind::Other,
    }
}

/// `SPINEL_MD_FLAG_ACKED_FP`: the RCP acknowledged a received frame with
/// Frame Pending set.
const MD_FLAG_ACKED_FP: u16 = 0x0010;

/// `SPINEL_MD_FLAG_ACKED_SEC`: the RCP acknowledged a received frame with a
/// secured enhanced ACK.
const MD_FLAG_ACKED_SEC: u16 = 0x0020;

/// How often the offset between the RCP clock and the host clock is measured
/// again - the reference host's `OPENTHREAD_SPINEL_CONFIG_RCP_TIME_SYNC_INTERVAL`.
const TIME_SYNC_INTERVAL: Duration = Duration::from_secs(60);

/// The RCP clock minus the host clock (`embassy-time`), in microseconds, as
/// last measured (`PROP_RCP_TIMESTAMP`); `None` until measured, or if the RCP
/// cannot tell its time. Process-wide, as the radio clock it serves is a plain
/// function (see the module docs).
static RCP_TIME_OFFSET: Mutex<CriticalSectionRawMutex, Cell<Option<i64>>> =
    Mutex::new(Cell::new(None));

/// The RCP time offset, if measured.
fn rcp_time_offset() -> Option<i64> {
    RCP_TIME_OFFSET.lock(Cell::get)
}

/// The radio clock: the RCP's clock, as estimated from the host clock.
fn rcp_now_us() -> u64 {
    let offset = rcp_time_offset().unwrap_or(0);

    (Instant::now().as_micros() as i64 + offset) as u64
}

/// The RCP capability ids we require (a real RCP in raw-MAC mode).
const CAP_CONFIG_RADIO: u32 = 34;
const CAP_MAC_RAW: u32 = 513;

/// The single interface id we use (non-multipan host).
const SPINEL_IID: u8 = 0;

/// `SPINEL_SCAN_STATE_ENERGY` — the [`PROP_MAC_SCAN_STATE`] value starting an
/// energy scan.
const SCAN_STATE_ENERGY: u8 = 2;

// Structural spinel constants, from the (bindgen-generated) `crate::sys`
// bindings of `spinel.h`. Re-typed to the `u32`/`u8` this driver uses (bindgen
// gives them the C enum/macro repr).
const HEADER_FLAG: u8 = crate::sys::SPINEL_HEADER_FLAG as u8;
const CMD_RESET: u32 = crate::sys::SPINEL_CMD_RESET as u32;
const CMD_PROP_VALUE_GET: u32 = crate::sys::SPINEL_CMD_PROP_VALUE_GET as u32;
const CMD_PROP_VALUE_SET: u32 = crate::sys::SPINEL_CMD_PROP_VALUE_SET as u32;
const CMD_PROP_VALUE_IS: u32 = crate::sys::SPINEL_CMD_PROP_VALUE_IS as u32;
const CMD_PROP_VALUE_INSERT: u32 = crate::sys::SPINEL_CMD_PROP_VALUE_INSERT as u32;
const CMD_PROP_VALUE_REMOVE: u32 = crate::sys::SPINEL_CMD_PROP_VALUE_REMOVE as u32;
const CMD_PROP_VALUE_INSERTED: u32 = crate::sys::SPINEL_CMD_PROP_VALUE_INSERTED as u32;
const CMD_PROP_VALUE_REMOVED: u32 = crate::sys::SPINEL_CMD_PROP_VALUE_REMOVED as u32;
const RESET_STACK: u32 = crate::sys::SPINEL_RESET_STACK as u32;
const STATUS_RESET_BEGIN: u32 = crate::sys::SPINEL_STATUS_RESET__BEGIN as u32;
const STATUS_RESET_END: u32 = crate::sys::SPINEL_STATUS_RESET__END as u32;

/// Timeout for a spinel *command* response (a `PROP_VALUE_SET`/`GET` ack, or a
/// reset status). Matches the reference `RadioSpinel::kMaxWaitTime` (2000 ms).
/// This must be generous: on a busy link the ack can queue behind a burst of
/// unsolicited inbound `STREAM_RAW` frames, and each is a fresh wire read.
const RESPONSE_TIMEOUT: Duration = Duration::from_millis(2000);

/// Timeout for a **transmit** to complete (the `STREAM_RAW` transmit-done
/// response). This is much longer than a plain command ack because the RCP does
/// CSMA/CA backoff **and** up to `maxFrameRetries` MAC retransmissions before it
/// reports done — a *broadcast* frame (e.g. an MLE Parent Request, which is
/// never acked) burns every backoff + retry slot first, which on a congested
/// channel comfortably exceeds one second. Matches the reference's
/// `OPENTHREAD_SPINEL_CONFIG_RCP_TX_WAIT_TIME_SECS` (5 s).
const TRANSMIT_TIMEOUT: Duration = Duration::from_secs(5);

/// Max on-the-wire spinel frame (pre-HDLC) we build/parse.
const MAX_SPINEL_FRAME: usize = OT_RADIO_FRAME_MAX_SIZE as usize + 128;

/// Max size of a stashed received-frame body (`STREAM_RAW` payload: the
/// length-prefixed PSDU plus RSSI/noise/flags/PHY-data metadata).
const RX_BODY_CAP: usize = OT_RADIO_FRAME_MAX_SIZE as usize + 32;

/// The default RX-queue depth (the `RX_QUEUE_DEPTH` const generic of
/// [`SpinelRadio`]): how many received frames to buffer while the driver is
/// busy waiting for a command response (see [`SpinelRadio::try_stash_rx`]).
///
/// Eight slots hold more than a saturated 250 kbps channel can deliver during
/// a typical command round-trip (~10-30 ms). Only a transmit stalled for
/// seconds in CSMA backoff on a congested mesh (see [`TRANSMIT_TIMEOUT`])
/// could overflow it — and overflow degrades gracefully (the oldest frame is
/// evicted). Deployments on very busy meshes can raise the depth via the
/// `RX_QUEUE_DEPTH` const generic of [`SpinelRadioResources`] (the radio
/// infers its own depth from the resources handed to [`SpinelRadio::new`]);
/// each slot costs `OT_RADIO_FRAME_MAX_SIZE + 32` (~160) bytes of RAM.
pub const DEFAULT_RX_QUEUE_DEPTH: usize = 8;

/// A stashed received-frame body: the `STREAM_RAW` payload bytes.
type RxFrame = heapless::Vec<u8, RX_BODY_CAP>;

// ---------------------------------------------------------------------------
// Transports: putting a raw spinel frame on a concrete wire.
//
// `SpinelRadio` is transport-agnostic; each of these implements
// `SpinelTransport` for a specific bus and owns that bus's framing (HDLC for a
// UART, the 5-byte header protocol for SPI).
// ---------------------------------------------------------------------------

pub mod spi;
pub mod uart;

pub use spi::{IntPolarity, SpiSpinelTransport, SpiTransportError, SpiTransportResources};
pub use uart::{UartSpinelTransport, UartTransportError, UartTransportResources};

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
/// transmit-done status). Returns the PSDU slice, its RSSI, and the channel it
/// was actually received on.
///
/// Layout (from OpenThread's `RadioSpinel::ParseRadioFrame`):
/// `DATA_WLEN(psdu) + i8 rssi + i8 noise + u16 flags + PHY-data struct + ...`,
/// where the PHY-data struct is a spinel struct (u16-LE length prefix) whose
/// first two bytes are the 802.15.4 channel and LQI. The RX channel matters
/// when the radio is retuned between reception and delivery (e.g. a frame
/// stashed during a command wait, or beacons during an active scan) — the
/// config channel at delivery time may no longer be the reception channel.
/// Missing metadata (a short body) degrades to `None`, and the caller falls
/// back to the config channel.
///
/// The PHY-data struct also carries the reception timestamp (RCP clock), and
/// an RCP with transmit security appends a MAC-data struct: the key index and
/// frame counter of the secured enhanced ACK it answered the frame with.
fn parse_radio_frame(body: &[u8]) -> Option<(&[u8], RxMeta)> {
    if body.len() < 2 {
        return None;
    }
    let plen = u16::from_le_bytes([body[0], body[1]]) as usize;
    if body.len() < 2 + plen {
        return None;
    }
    let psdu = &body[2..2 + plen];

    // Metadata after the PSDU: rssi(1) + noise(1) + flags(2), then the
    // PHY-data struct (channel + lqi + timestamp), the vendor-data struct
    // (receive error) and the optional MAC-data struct (ACK key index + frame
    // counter). A spinel struct is a 2-byte LE length prefix + its contents.
    let meta = &body[2 + plen..];
    let rssi = meta.first().map(|&b| b as i8);
    let flags = meta
        .get(2..4)
        .map(|flags| u16::from_le_bytes([flags[0], flags[1]]))
        .unwrap_or(0);

    let mut structs = meta.get(4..).unwrap_or(&[]);
    let phy = take_struct(&mut structs);
    let _vendor = take_struct(&mut structs);
    let mac = take_struct(&mut structs);
    let len = meta.len() - structs.len();

    let channel = phy.and_then(|phy| phy.first().copied());
    let lqi = phy.and_then(|phy| phy.get(1).copied());
    let timestamp = phy
        .and_then(|phy| phy.get(2..10))
        .map(|ts| u64::from_le_bytes([ts[0], ts[1], ts[2], ts[3], ts[4], ts[5], ts[6], ts[7]]))
        .filter(|ts| *ts != 0);
    let ack_security = mac
        .filter(|_| flags & MD_FLAG_ACKED_SEC != 0)
        .and_then(|mac| {
            Some(AckSecurity {
                key_id: *mac.first()?,
                frame_counter: u32::from_le_bytes(mac.get(1..5)?.try_into().ok()?),
            })
        });

    Some((
        psdu,
        RxMeta {
            rssi,
            channel,
            lqi,
            timestamp,
            ack_security,
            acked_with_frame_pending: flags & MD_FLAG_ACKED_FP != 0,
            len,
        },
    ))
}

/// Split the next spinel struct (a 2-byte LE length prefix + its contents) off
/// the front of `data`.
fn take_struct<'a>(data: &mut &'a [u8]) -> Option<&'a [u8]> {
    let cur = *data;
    let len = u16::from_le_bytes([*cur.first()?, *cur.get(1)?]) as usize;
    let contents = cur.get(2..2 + len)?;
    *data = &cur[2 + len..];

    Some(contents)
}

/// The metadata the RCP reports with a received frame (see
/// [`parse_radio_frame`]).
struct RxMeta {
    rssi: Option<i8>,
    channel: Option<u8>,
    lqi: Option<u8>,
    /// When the frame was received, in the RCP clock.
    timestamp: Option<u64>,
    /// The secured enhanced ACK the RCP answered the frame with, if any.
    ack_security: Option<AckSecurity>,
    /// Whether the RCP answered the frame with an ACK with Frame Pending set.
    acked_with_frame_pending: bool,
    /// How many bytes the metadata took after the PSDU (with its length).
    len: usize,
}

/// Append a (possibly NUL-terminated) UTF-8 blob `src` into `out` starting at
/// `at`, returning the new total length. Used to collect diag command output.
#[cfg(feature = "diag")]
fn copy_utf8(src: &[u8], out: &mut [u8], at: usize) -> usize {
    let end = src.iter().position(|&b| b == 0).unwrap_or(src.len());
    let copy = end.min(out.len().saturating_sub(at));
    out[at..at + copy].copy_from_slice(&src[..copy]);
    at + copy
}

/// Log a spinel frame's header, so a link that misbehaves can be read off a
/// `RUST_LOG=openthread=trace` run rather than guessed at. Deliberately just
/// the header plus the payload length: whole frames at this level would bury
/// the conversation in radio traffic.
fn trace_frame(direction: &str, frame: &[u8]) {
    match spinel_parse_header(frame) {
        Some((tid, cmd, prop, off)) if prop == PROP_LAST_STATUS => {
            // The status value is the whole story of a `LAST_STATUS` frame -
            // "1 byte" without it hides exactly the failures worth seeing.
            let status = spinel_uint_decode(&frame[off..])
                .map(|(status, _)| status)
                .unwrap_or(u32::MAX);

            trace!(
                "{} tid {} cmd 0x{:x} LAST_STATUS {}",
                direction,
                tid,
                cmd,
                status,
            );
        }
        Some((tid, cmd, prop, off)) => trace!(
            "{} tid {} cmd 0x{:x} prop 0x{:x} ({} bytes)",
            direction,
            tid,
            cmd,
            prop,
            frame.len().saturating_sub(off),
        ),
        None => trace!("{} unparseable ({} bytes)", direction, frame.len()),
    }
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
///
/// `ENERGY_SCAN` is part of the baseline even though it is not strictly part of
/// the transmit contract: the scan runs on the *RCP's* MAC sub-layer (via the
/// `MAC_SCAN_*` properties), which measures with the radio hardware's energy
/// detector — or, failing that, samples real RSSI on the RCP itself — so
/// forwarding is the only path to real measurements (the host cannot sample a
/// remote radio's RSSI synchronously). An RCP that cannot scan at all rejects
/// the scan-state write, which [`SpinelRadio::energy_scan`] surfaces as an
/// error (reported to OpenThread as an "invalid RSSI" scan result).
const SPINEL_RADIO_CAPS: Capabilities = Capabilities::ACK_TIMEOUT
    .union(Capabilities::CSMA_BACKOFF)
    .union(Capabilities::TRANSMIT_RETRIES)
    .union(Capabilities::ENERGY_SCAN);

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
    .union(MacCapabilities::RX_ACK)
    .union(MacCapabilities::PROMISCUOUS)
    .union(MacCapabilities::SRC_MATCH);

/// The resources (buffers) needed by a [`SpinelRadio`].
///
/// A separate type so that the (large) buffers can be allocated separately
/// from the radio itself — e.g. in a `static` — rather than travel by value
/// inside `SpinelRadio` through constructor returns and into the future that
/// runs the stack, risking transient stack blow-ups on small MCUs.
///
/// `new` is `const`, and the buffers start their life as `MaybeUninit`, so a
/// `SpinelRadioResources` can be statically-allocated (e.g. in a
/// `static_cell::ConstStaticCell`) without any stack traffic; they are
/// initialized in-place by [`SpinelRadio::new`].
///
/// The `RX_QUEUE_DEPTH` const generic sizes the queue for inbound frames that
/// arrive while a command round-trip is in flight (see
/// [`DEFAULT_RX_QUEUE_DEPTH`] for how to size it). It is erased from the
/// `SpinelRadio` borrowing these resources.
pub struct SpinelRadioResources<const RX_QUEUE_DEPTH: usize = DEFAULT_RX_QUEUE_DEPTH> {
    /// Scratch buffer for the raw spinel frame being built for transmission.
    tx_frame: MaybeUninit<[u8; MAX_SPINEL_FRAME]>,
    /// The most recently received raw spinel frame.
    rx_frame: MaybeUninit<[u8; MAX_SPINEL_FRAME]>,
    /// Received-frame bodies stashed while a command response is awaited.
    rx_queue: MaybeUninit<heapless::Deque<RxFrame, RX_QUEUE_DEPTH>>,
    /// The radio's state (source-match table, last config, etc.).
    state: MaybeUninit<SpinelRadioState>,
}

impl<const RX_QUEUE_DEPTH: usize> SpinelRadioResources<RX_QUEUE_DEPTH> {
    /// Create a new `SpinelRadioResources` instance.
    pub const fn new() -> Self {
        Self {
            tx_frame: MaybeUninit::uninit(),
            rx_frame: MaybeUninit::uninit(),
            rx_queue: MaybeUninit::uninit(),
            state: MaybeUninit::uninit(),
        }
    }

    /// Initialize the resources, as they start their life as `MaybeUninit` so
    /// as to avoid mem-moves. Returns the buffers, with the queue's compile-time
    /// capacity erased to a [`heapless::DequeView`].
    fn init(
        &mut self,
    ) -> (
        &mut [u8; MAX_SPINEL_FRAME],
        &mut [u8; MAX_SPINEL_FRAME],
        &mut heapless::deque::DequeView<RxFrame>,
        &mut SpinelRadioState,
    ) {
        (
            self.tx_frame.write([0; MAX_SPINEL_FRAME]),
            self.rx_frame.write([0; MAX_SPINEL_FRAME]),
            self.rx_queue.write(heapless::Deque::new()).as_mut_view(),
            self.state.write(SpinelRadioState {
                src_match: SrcMatchConfig::new(),
                src_match_flushed: SrcMatchConfig::new(),
                config: None,
            }),
        )
    }
}

impl<const RX_QUEUE_DEPTH: usize> Default for SpinelRadioResources<RX_QUEUE_DEPTH> {
    fn default() -> Self {
        Self::new()
    }
}

/// A [`crate::Radio`] implementation that drives a remote 802.15.4 radio (an
/// OpenThread RCP) over a [`SpinelTransport`] using the spinel protocol.
///
/// Hand it to [`OpenThread::run`](crate::OpenThread::run) exactly like a local
/// radio, wrapping the wire in the matching [`SpinelTransport`]. The buffers
/// live in a separately-allocated (e.g. static) [`SpinelRadioResources`]:
///
/// ```ignore
/// static RESOURCES: ConstStaticCell<SpinelRadioResources> =
///     ConstStaticCell::new(SpinelRadioResources::new());
///
/// // or SpiSpinelTransport::new(spi, int, polarity, ...)
/// let transport = UartSpinelTransport::new(uart, ...);
/// let radio = SpinelRadio::new(transport, RESOURCES.take());
/// ot.run(radio).await
/// ```
pub struct SpinelRadio<'a, T> {
    transport: T,
    /// The RCP is brought up on the first radio operation (or eagerly via
    /// [`Radio::init`]). `None` until the startup handshake runs.
    eui64: Option<[u8; 8]>,
    /// The PHY capabilities read from the RCP's `PROP_RADIO_CAPS` during the
    /// handshake. Until the handshake runs it holds the fixed baseline
    /// ([`SPINEL_RADIO_CAPS`]); afterwards it is the RCP's reported set.
    caps: Capabilities,
    /// The channel the RCP is currently tuned to (`PROP_PHY_CHAN`), and the
    /// CCA threshold it currently has (`PROP_PHY_CCA_THRESHOLD`).
    channel: u8,
    /// The CCA threshold the RCP currently has (`PROP_PHY_CCA_THRESHOLD`).
    cca_threshold: i8,
    /// The RCP's own defaults for the transmit power and the CCA threshold,
    /// read once during the handshake and reported through [`RadioCaps`].
    default_tx_power: i8,
    /// The RCP's own defaults for the transmit power and the CCA threshold,
    /// read once during the handshake and reported through [`RadioCaps`].
    default_cca_threshold: i8,
    /// The receive sensitivity read from the RCP's `PHY_RX_SENSITIVITY`
    /// during the handshake; the crate-wide default until then (and for RCP
    /// firmwares that do not implement the property).
    sensitivity: i8,
    /// The source-match table in `state` is not yet pushed to the RCP: the
    /// trait's delivery is synchronous, the spinel writes are not, so the
    /// push happens on the next async operation (see `flush_src_match`).
    src_match_dirty: bool,
    /// Whether raw-stream (RX) is currently enabled on the RCP.
    rx_enabled: bool,
    /// When the RCP time offset is due to be measured (again).
    time_sync_due: Instant,
    /// The RCP's CSL timing figures (`PROP_RCP_CSL_ACCURACY` /
    /// `PROP_RCP_CSL_UNCERTAINTY`), read during the handshake; `u8::MAX`
    /// (unknown) for RCP firmwares without CSL.
    csl_accuracy_ppm: u8,
    /// See `csl_accuracy_ppm`.
    csl_uncertainty: u8,
    /// The speed of the link to the RCP, in bits per second (see
    /// [`Self::with_bus_speed`]).
    bus_speed: u32,
    /// The latency of the link to the RCP, in microseconds (see
    /// [`Self::with_bus_latency`]).
    bus_latency_us: u32,
    /// Next transaction id (1..=15, 0 is reserved for unsolicited notifications).
    next_tid: u8,
    /// Scratch buffer for the raw spinel frame being built for transmission.
    tx_frame: &'a mut [u8; MAX_SPINEL_FRAME],
    /// The most recently received raw spinel frame, and its length. Response
    /// parsers borrow `rx_frame[..rx_len]` after a `recv_frame`.
    rx_frame: &'a mut [u8; MAX_SPINEL_FRAME],
    rx_len: usize,
    /// Received-frame bodies that arrived (as unsolicited `STREAM_RAW`
    /// notifications) while the driver was waiting for a command response, so
    /// they cannot be dropped. `receive` drains this before reading the wire.
    ///
    /// This matters because the RCP has already MAC-ACKed a unicast before
    /// forwarding it, so a frame dropped here would never be retransmitted at
    /// the MAC layer — only slow upper-layer retries could recover it. The
    /// reference `SpinelDriver` keeps the same queue (its `MultiFrameBuffer`
    /// of frames saved while `WaitResponse` runs).
    ///
    /// A `DequeView`, so the queue depth chosen via [`SpinelRadioResources`]
    /// does not generify this type.
    rx_queue: &'a mut heapless::deque::DequeView<RxFrame>,
    /// The source-match tables and config shadow, borrowed from the
    /// resources - see [`SpinelRadioState`].
    state: &'a mut SpinelRadioState,
}

impl<'a, T> SpinelRadio<'a, T>
where
    T: SpinelTransport,
{
    /// Create a new `SpinelRadio` over `transport`, with its buffers borrowed
    /// from `resources` (whose `RX_QUEUE_DEPTH` — see
    /// [`DEFAULT_RX_QUEUE_DEPTH`] — sizes the RX queue). The RCP is
    /// initialized on the first radio operation.
    pub fn new<const RX_QUEUE_DEPTH: usize>(
        transport: T,
        resources: &'a mut SpinelRadioResources<RX_QUEUE_DEPTH>,
    ) -> Self {
        let (tx_frame, rx_frame, rx_queue, state) = resources.init();

        Self {
            transport,
            eui64: None,
            caps: SPINEL_RADIO_CAPS,
            channel: 11,
            cca_threshold: RadioCaps::DEFAULT_CCA_THRESHOLD,
            default_tx_power: RadioCaps::DEFAULT_TX_POWER,
            default_cca_threshold: RadioCaps::DEFAULT_CCA_THRESHOLD,
            sensitivity: RadioCaps::DEFAULT_RECEIVE_SENSITIVITY,
            src_match_dirty: false,
            rx_enabled: false,
            time_sync_due: Instant::from_ticks(0),
            csl_accuracy_ppm: u8::MAX,
            csl_uncertainty: u8::MAX,
            bus_speed: 0,
            bus_latency_us: 0,
            next_tid: 1,
            tx_frame,
            rx_frame,
            rx_len: 0,
            rx_queue,
            state,
        }
    }

    /// Tell the radio the speed of the link to the RCP, in payload bits per
    /// second - for a UART, its baud rate less the start and stop bits (8/10
    /// of it). An FTD needs it to hand the frames for its CSL children over
    /// early enough to cross the link in time (see [`RadioCaps::bus_speed`]);
    /// `0` (the default) treats the link as instant.
    #[must_use]
    pub fn with_bus_speed(mut self, bits_per_second: u32) -> Self {
        self.bus_speed = bits_per_second;
        self
    }

    /// Tell the radio the latency of the link to the RCP on top of its speed,
    /// in microseconds: e.g. a USB serial bridge, which moves data at the
    /// host's polling interval (see [`RadioCaps::bus_latency_us`]).
    #[must_use]
    pub fn with_bus_latency(mut self, latency_us: u32) -> Self {
        self.bus_latency_us = latency_us;
        self
    }

    /// Measure the offset between the RCP clock and the host clock, if due.
    ///
    /// Best-effort: an RCP that cannot tell its time leaves the radio without
    /// timestamps and timed transmit (see [`Self::ensure_init`]).
    async fn ensure_time_sync(&mut self) {
        if Instant::now() < self.time_sync_due {
            return;
        }

        // The parameter is a dummy timestamp, so that the request is as long
        // as the response and the two link delays cancel out (as the
        // reference host does it); the RCP time then lies halfway between
        // sending and receiving.
        let sent = Instant::now();
        let remote = self
            .get_prop_with(PROP_RCP_TIMESTAMP, &0u64.to_le_bytes(), |payload| {
                payload.get(..8).map(|ts| {
                    u64::from_le_bytes([ts[0], ts[1], ts[2], ts[3], ts[4], ts[5], ts[6], ts[7]])
                })
            })
            .await;
        let received = Instant::now();

        self.time_sync_due = received + TIME_SYNC_INTERVAL;

        match remote {
            Ok(Some(remote)) => {
                let local = (sent.as_micros() + received.as_micros()) / 2;

                RCP_TIME_OFFSET.lock(|offset| offset.set(Some(remote as i64 - local as i64)));
            }
            _ => debug!("RCP: could not read its clock"),
        }
    }

    /// If the just-received frame in `rx_frame[..frame_len]` is an *unsolicited*
    /// received-radio-frame notification (`tid == 0`, `PROP_VALUE_IS`,
    /// `STREAM_RAW`), stash its body in the RX queue so a later [`Self::receive`]
    /// can return it, and report `true`. Called from the command-response wait
    /// loops ([`Self::await_response`], [`Self::drain_acks`]) so inbound frames
    /// are never dropped while a command is in flight.
    ///
    /// If the queue is full the oldest frame is evicted (newest-wins) — a
    /// deliberate bound; sustained inability to drain means the consumer is not
    /// calling `receive` fast enough, in which case dropping the stalest frame is
    /// the least-bad option.
    fn try_stash_rx(&mut self, frame_len: usize) -> bool {
        let frame = &self.rx_frame[..frame_len];
        let Some((tid, rcmd, rprop, off)) = spinel_parse_header(frame) else {
            return false;
        };
        if tid != 0 || rcmd != CMD_PROP_VALUE_IS || rprop != PROP_STREAM_RAW {
            return false;
        }

        let body = &self.rx_frame[off..frame_len];
        let mut stashed = RxFrame::new();
        // Truncate to capacity (a valid 802.15.4 frame body always fits).
        let n = body.len().min(RX_BODY_CAP);
        // `extend_from_slice` on a fixed Vec cannot fail for `n <= capacity`.
        let _ = stashed.extend_from_slice(&body[..n]);

        if self.rx_queue.is_full() {
            let _ = self.rx_queue.pop_front();
        }
        let _ = self.rx_queue.push_back(stashed);
        true
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
        trace_frame("RCP <-", frame);

        self.transport
            .send(frame)
            .await
            .map_err(|_| RadioErrorKind::TxFailed)
    }

    /// Receive one complete raw spinel frame into `self.rx_frame`, or fail on
    /// `timeout`. Returns the frame length (also stashed in `self.rx_len`);
    /// callers parse `self.rx_frame[..len]`.
    async fn recv_frame(&mut self, timeout: Duration) -> Result<usize, RadioErrorKind> {
        let len = {
            let recv_fut = self.transport.recv(&mut self.rx_frame[..]);
            let mut recv_fut = core::pin::pin!(recv_fut);
            let mut timeout_fut = core::pin::pin!(Timer::after(timeout));

            match embassy_futures::select::select(&mut recv_fut, &mut timeout_fut).await {
                embassy_futures::select::Either::First(r) => {
                    r.map_err(|_| RadioErrorKind::RxFailed)?
                }
                embassy_futures::select::Either::Second(()) => {
                    return Err(RadioErrorKind::RxFailed)
                }
            }
        };
        self.rx_len = len;

        trace_frame("RCP ->", &self.rx_frame[..len]);

        Ok(len)
    }

    /// Send a `PROP_VALUE_SET` with a raw payload and await its echoed
    /// `PROP_VALUE_IS` acknowledgement (matched by TID).
    async fn set_prop(&mut self, prop: u32, payload: &[u8]) -> Result<(), RadioErrorKind> {
        self.send_prop_await(prop, payload, RESPONSE_TIMEOUT)
            .await
            .map(|_| ())
    }

    /// Push a pending source-match table to the RCP, if any.
    ///
    /// Each address family goes as one whole-array `VALUE_SET`.
    async fn flush_src_match(&mut self) -> Result<(), RadioErrorKind> {
        if !self.src_match_dirty {
            return Ok(());
        }

        // Per-entry INSERT/REMOVE, with an empty whole-table SET only for
        // clear-all: exactly the wire shapes OpenThread's own hosts use,
        // and therefore the only ones every RCP firmware actually exercises.
        //
        // `src_match_flushed` mirrors what the RCP has, entry by entry, so a
        // failed op leaves the mirror truthful and the dirty flag makes the
        // next operation prologue retry the remainder.

        let target = self.state.src_match.clone();

        // Removals first, freeing table slots for the additions.
        if target.short_addrs.is_empty() && !self.state.src_match_flushed.short_addrs.is_empty() {
            self.set_prop(PROP_MAC_SRC_MATCH_SHORT_ADDRESSES, &[])
                .await?;
            self.state.src_match_flushed.short_addrs.clear();
        } else {
            for index in (0..self.state.src_match_flushed.short_addrs.len()).rev() {
                let addr = self.state.src_match_flushed.short_addrs[index];
                if !target.short_addrs.contains(&addr) {
                    self.modify_prop(
                        CMD_PROP_VALUE_REMOVE,
                        PROP_MAC_SRC_MATCH_SHORT_ADDRESSES,
                        &addr.to_le_bytes(),
                    )
                    .await?;
                    self.state.src_match_flushed.short_addrs.swap_remove(index);
                }
            }
        }

        if target.ext_addrs.is_empty() && !self.state.src_match_flushed.ext_addrs.is_empty() {
            self.set_prop(PROP_MAC_SRC_MATCH_EXTENDED_ADDRESSES, &[])
                .await?;
            self.state.src_match_flushed.ext_addrs.clear();
        } else {
            for index in (0..self.state.src_match_flushed.ext_addrs.len()).rev() {
                let addr = self.state.src_match_flushed.ext_addrs[index];
                if !target.ext_addrs.contains(&addr) {
                    self.modify_prop(
                        CMD_PROP_VALUE_REMOVE,
                        PROP_MAC_SRC_MATCH_EXTENDED_ADDRESSES,
                        &addr.to_be_bytes(),
                    )
                    .await?;
                    self.state.src_match_flushed.ext_addrs.swap_remove(index);
                }
            }
        }

        for addr in &target.short_addrs {
            if !self.state.src_match_flushed.short_addrs.contains(addr) {
                self.modify_prop(
                    CMD_PROP_VALUE_INSERT,
                    PROP_MAC_SRC_MATCH_SHORT_ADDRESSES,
                    &addr.to_le_bytes(),
                )
                .await?;
                let _ = self.state.src_match_flushed.short_addrs.push(*addr);
            }
        }

        for addr in &target.ext_addrs {
            if !self.state.src_match_flushed.ext_addrs.contains(addr) {
                self.modify_prop(
                    CMD_PROP_VALUE_INSERT,
                    PROP_MAC_SRC_MATCH_EXTENDED_ADDRESSES,
                    &addr.to_be_bytes(),
                )
                .await?;
                let _ = self.state.src_match_flushed.ext_addrs.push(*addr);
            }
        }

        if target.enabled != self.state.src_match_flushed.enabled {
            self.set_prop(PROP_MAC_SRC_MATCH_ENABLED, &[target.enabled as u8])
                .await?;
            self.state.src_match_flushed.enabled = target.enabled;
        }

        self.src_match_dirty = false;

        Ok(())
    }

    /// Send a `PROP_VALUE_INSERT`/`PROP_VALUE_REMOVE` for a single table entry
    /// and await its acknowledgement (`PROP_VALUE_INSERTED`/`REMOVED`; a
    /// `LAST_STATUS` reply carries the failure).
    async fn modify_prop(
        &mut self,
        cmd: u32,
        prop: u32,
        payload: &[u8],
    ) -> Result<(), RadioErrorKind> {
        let tid = self.alloc_tid();

        let frame_len = {
            let mut n = spinel_frame_prefix(&mut self.tx_frame[..], tid, cmd, prop)
                .ok_or(RadioErrorKind::TxFailed)?;
            if n + payload.len() > self.tx_frame.len() {
                return Err(RadioErrorKind::TxFailed);
            }
            self.tx_frame[n..n + payload.len()].copy_from_slice(payload);
            n += payload.len();
            n
        };

        trace_frame("RCP <-", &self.tx_frame[..frame_len]);
        self.transport
            .send(&self.tx_frame[..frame_len])
            .await
            .map_err(|_| RadioErrorKind::TxFailed)?;

        let (rprop, off) = self.await_response(tid, RESPONSE_TIMEOUT).await?;

        if rprop == PROP_LAST_STATUS {
            let status = spinel_uint_decode(&self.rx_frame[off..self.rx_len])
                .map(|(status, _)| status)
                .unwrap_or(0);

            if status != 0 {
                debug!(
                    "RCP: {} of prop 0x{:x} refused, LAST_STATUS {}",
                    if cmd == CMD_PROP_VALUE_INSERT {
                        "INSERT"
                    } else {
                        "REMOVE"
                    },
                    prop,
                    status
                );

                return Err(RadioErrorKind::Other);
            }
        }

        Ok(())
    }

    /// Send a `PROP_VALUE_SET` with a raw payload and await its matched
    /// `PROP_VALUE_IS` response, returning `(prop, payload_offset)` into
    /// `self.rx_frame` (length `self.rx_len`). Most callers only need the ack and
    /// use [`Self::set_prop`]; `transmit` uses this to read the transmit-done
    /// body (status + ACK frame).
    ///
    /// `timeout` bounds the wait for the matched response: a plain config ack
    /// uses [`RESPONSE_TIMEOUT`], while `transmit` passes the longer
    /// [`TRANSMIT_TIMEOUT`] because a transmit-done can lag by seconds while the
    /// RCP does CSMA backoff + MAC retries.
    async fn send_prop_await(
        &mut self,
        prop: u32,
        payload: &[u8],
        timeout: Duration,
    ) -> Result<(u32, usize), RadioErrorKind> {
        let tid = self.alloc_tid();
        let cmd = CMD_PROP_VALUE_SET;

        let frame_len = {
            let mut n = spinel_frame_prefix(&mut self.tx_frame[..], tid, cmd, prop)
                .ok_or(RadioErrorKind::TxFailed)?;
            if n + payload.len() > self.tx_frame.len() {
                return Err(RadioErrorKind::TxFailed);
            }
            self.tx_frame[n..n + payload.len()].copy_from_slice(payload);
            n += payload.len();
            n
        };

        // Send straight out of `self.tx_frame`: the transport borrow and the
        // frame borrow are disjoint fields.
        trace_frame("RCP <-", &self.tx_frame[..frame_len]);
        self.transport
            .send(&self.tx_frame[..frame_len])
            .await
            .map_err(|_| RadioErrorKind::TxFailed)?;

        self.await_response(tid, timeout).await
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
        self.get_prop_with(prop, &[], f).await
    }

    /// [`Self::get_prop`], with a parameter for the property.
    async fn get_prop_with<R>(
        &mut self,
        prop: u32,
        param: &[u8],
        f: impl FnOnce(&[u8]) -> R,
    ) -> Result<R, RadioErrorKind> {
        let tid = self.alloc_tid();
        let cmd = CMD_PROP_VALUE_GET;

        let mut frame_len = spinel_frame_prefix(&mut self.tx_frame[..], tid, cmd, prop)
            .ok_or(RadioErrorKind::TxFailed)?;
        if frame_len + param.len() > self.tx_frame.len() {
            return Err(RadioErrorKind::TxFailed);
        }
        self.tx_frame[frame_len..frame_len + param.len()].copy_from_slice(param);
        frame_len += param.len();
        trace_frame("RCP <-", &self.tx_frame[..frame_len]);
        self.transport
            .send(&self.tx_frame[..frame_len])
            .await
            .map_err(|_| RadioErrorKind::TxFailed)?;

        let off = self.await_prop(tid, prop).await?;
        let len = self.rx_len;

        Ok(f(&self.rx_frame[off..len]))
    }

    /// Await the response to `tid` that actually carries `prop`.
    ///
    /// A `PROP_VALUE_GET` is answered either with the property itself or with
    /// `LAST_STATUS` - the RCP's way of saying it cannot serve the request,
    /// which is an error for this property (and, for the optional reads,
    /// simply means "use the default").
    async fn await_prop(&mut self, tid: u8, prop: u32) -> Result<usize, RadioErrorKind> {
        loop {
            let (rprop, off) = self.await_response(tid, RESPONSE_TIMEOUT).await?;

            if rprop == prop {
                return Ok(off);
            }

            if rprop == PROP_LAST_STATUS {
                let status = spinel_uint_decode(&self.rx_frame[off..self.rx_len])
                    .map(|(status, _)| status)
                    .unwrap_or(0);

                // Not an error in itself - a refusal is a normal protocol
                // outcome, and the caller decides whether it matters (the
                // best-effort reads fall back to a default, and say so).
                debug!("RCP: property 0x{:x} refused, LAST_STATUS {}", prop, status);

                return Err(RadioErrorKind::Other);
            }

            warn!(
                "RCP: awaiting property 0x{:x} on tid {}, got 0x{:x} - ignoring",
                prop, tid, rprop
            );
        }
    }

    /// Await a single response frame with a matching `tid`, dispatching any
    /// unsolicited (`tid == 0`) frames received meanwhile. Returns `(prop,
    /// payload_offset)` into `self.rx_frame` (whose length is `self.rx_len`).
    ///
    /// `timeout` bounds *each* wire read; every stashed inbound frame restarts
    /// the wait, so a busy link does not time the response out prematurely.
    async fn await_response(
        &mut self,
        tid: u8,
        timeout: Duration,
    ) -> Result<(u32, usize), RadioErrorKind> {
        loop {
            let frame_len = self.recv_frame(timeout).await?;

            // Stash any inbound radio frame that arrives while we wait, rather
            // than dropping it (OpenThread transmits near-continuously).
            if self.try_stash_rx(frame_len) {
                continue;
            }

            let frame = &self.rx_frame[..frame_len];
            let Some((rtid, rcmd, rprop, off)) = spinel_parse_header(frame) else {
                continue;
            };

            // `PROP_VALUE_IS` answers GET/SET (and carries `LAST_STATUS`
            // errors); `INSERTED`/`REMOVED` answer the per-entry table
            // modifications.
            if rtid == tid
                && (rcmd == CMD_PROP_VALUE_IS
                    || rcmd == CMD_PROP_VALUE_INSERTED
                    || rcmd == CMD_PROP_VALUE_REMOVED)
            {
                return Ok((rprop, off));
            }
            // A mismatched command response — ignore.
        }
    }

    /// Drain the acknowledgements for a *set* of outstanding transaction ids,
    /// one `PROP_VALUE_IS` per TID (arriving in any order, possibly coalesced in
    /// a single transport read — important for SPI). Returns once every TID in
    /// `pending` has been acknowledged, or errors on the response timeout.
    ///
    /// The value payloads of the acks are ignored (these are the echoed SET
    /// confirmations); only their arrival is required. Inbound radio frames seen
    /// meanwhile are stashed (see [`Self::try_stash_rx`]), matching
    /// [`Self::await_response`].
    async fn drain_acks(&mut self, mut pending: TidSet) -> Result<(), RadioErrorKind> {
        let cmd_is = CMD_PROP_VALUE_IS;

        while !pending.is_empty() {
            let frame_len = self.recv_frame(RESPONSE_TIMEOUT).await?;

            if self.try_stash_rx(frame_len) {
                continue;
            }

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

            // RESET is a bare command; the "prop" slot below carries the reset
            // kind as a packed-uint argument.
            let n = spinel_frame_prefix(&mut self.tx_frame[..], tid, cmd, reset_arg)
                .ok_or(RadioErrorKind::Other)?;
            trace_frame("RCP <-", &self.tx_frame[..n]);
            self.transport
                .send(&self.tx_frame[..n])
                .await
                .map_err(|_| RadioErrorKind::TxFailed)?;

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
        //
        // Best-effort: this property is an OpenThread extension, and stock RCP
        // firmware that predates it (or omits it) answers `PROP_NOT_FOUND`.
        let caps_bits = self
            .get_prop(PROP_RADIO_CAPS, |payload| {
                spinel_uint_decode(payload).map(|(v, _)| v).unwrap_or(0)
            })
            .await
            .unwrap_or_else(|_| {
                info!("RCP does not report RADIO_CAPS; using the baseline only");
                0
            });
        // Never `RECEIVE_TIMING`, whatever the RCP has: a CSL child needs the
        // RCP to advertise its schedule in its enhanced ACKs, and spinel has
        // no property to hand it that schedule (see the module docs).
        self.caps = (Capabilities::from_bits_truncate(caps_bits as u16) | SPINEL_RADIO_CAPS)
            .difference(Capabilities::RECEIVE_TIMING);

        info!(
            "RCP radio caps: 0x{:04x} (reported 0x{:08x} + baseline 0x{:04x})",
            self.caps.bits(),
            caps_bits,
            SPINEL_RADIO_CAPS.bits()
        );

        // Read the RCP's receive sensitivity. Best-effort.
        match self
            .get_prop(PROP_PHY_RX_SENSITIVITY, |payload| {
                payload.first().map(|&s| s as i8)
            })
            .await
        {
            Ok(Some(sensitivity)) => self.sensitivity = sensitivity,
            _ => {
                info!(
                    "RCP does not report PHY_RX_SENSITIVITY; using the default {} dBm",
                    self.sensitivity
                );
            }
        }

        // The RCP's power-on transmit power and CCA threshold (both dBm, int8)
        // become this radio's reported defaults.
        //
        // Best-effort, like the sensitivity read above.
        match self
            .get_prop(PROP_PHY_TX_POWER, |payload| {
                payload.first().map(|&p| p as i8)
            })
            .await
        {
            Ok(Some(power)) => self.default_tx_power = power,
            _ => info!(
                "RCP does not report PHY_TX_POWER; using the default {} dBm",
                self.default_tx_power
            ),
        }

        match self
            .get_prop(PROP_PHY_CCA_THRESHOLD, |payload| {
                payload.first().map(|&t| t as i8)
            })
            .await
        {
            Ok(Some(threshold)) => {
                self.default_cca_threshold = threshold;
                self.cca_threshold = threshold;
            }
            _ => info!(
                "RCP does not report PHY_CCA_THRESHOLD; using the default {} dBm",
                self.default_cca_threshold
            ),
        }

        // The RCP's CSL timing figures, which an FTD reports to its CSL
        // children. Best-effort: only RCP firmwares with CSL have them.
        if let Ok(Some(accuracy)) = self
            .get_prop(PROP_RCP_CSL_ACCURACY, |payload| payload.first().copied())
            .await
        {
            self.csl_accuracy_ppm = accuracy;
        }
        if let Ok(Some(uncertainty)) = self
            .get_prop(PROP_RCP_CSL_UNCERTAINTY, |payload| payload.first().copied())
            .await
        {
            self.csl_uncertainty = uncertainty;
        }

        // Timed transmit and frame timestamps cross the link in the RCP's
        // clock, so they need its offset to the host clock.
        self.time_sync_due = Instant::from_ticks(0);
        self.ensure_time_sync().await;
        if rcp_time_offset().is_none() {
            self.caps.remove(Capabilities::TRANSMIT_TIMING);
        }

        // Enable the PHY.
        self.set_prop(PROP_PHY_ENABLED, &[1]).await?;

        Ok(())
    }

    /// Wait for an unsolicited `LAST_STATUS` in `[begin, end)` (the RCP reset
    /// acknowledgement). Best-effort with a timeout.
    ///
    /// No RX stashing here (unlike [`Self::await_response`]): the RCP has just
    /// been reset, so its raw stream is disabled and no `STREAM_RAW` frames can
    /// arrive during this wait.
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
        let prev = self.state.config.clone();
        let changed = |get: fn(&Config) -> u64| prev.as_ref().map(get) != Some(get(config));

        // Materialize each changed property's little-endian payload into a local
        // so its slice stays valid for the whole burst, then stage the
        // `(prop, payload)` pairs. At most seven properties, so a fixed array +
        // length avoids any allocation.
        let promisc = [config.promiscuous as u8];
        let rx_on_when_idle = [!config.auto_sleep as u8];
        let pan_id = config.pan_id.unwrap_or(0xffff).to_le_bytes();
        let short_addr = config.short_addr.unwrap_or(0xffff).to_le_bytes();
        // Alternate short address: `0xfffe` (`OT_RADIO_INVALID_SHORT_ADDR`) is the
        // "no alternate / clear" wire value the RCP expects.
        let alt_short_addr = config.alt_short_addr.unwrap_or(0xfffe).to_le_bytes();
        // The spinel `MAC_15_4_LADDR` property carries the extended address in
        // *reversed* byte order relative to the bytes `otPlatRadioSetExtendedAddress`
        // hands the platform — which is what `Config::ext_addr` holds, as
        // `u64::from_le_bytes` of those bytes (see `platform.rs`). The reference
        // POSIX host reverses before encoding (`otPlatRadioSetExtendedAddress` in
        // `posix/platform/radio.cpp`), so big-endian is the wire order. The RCP's
        // hardware address filter — and hence all unicast reception — depends on
        // this order.
        let ext_addr = config.ext_addr.unwrap_or(0).to_be_bytes();

        let mut batch: [(u32, &[u8]); 8] = [(0, &[]); 8];
        let mut count = 0;

        if changed(|c| c.promiscuous as u64) {
            batch[count] = (PROP_MAC_PROMISCUOUS_MODE, &promisc);
            count += 1;
        }
        if changed(|c| c.auto_sleep as u64) {
            // NOTE: some RCP firmwares (e.g. the nRF `ot-rcp`) do not implement
            // this property and reply with an error LAST_STATUS, which we ignore
            // (the RCP then keeps its default rx-when-idle behaviour). Harmless.
            batch[count] = (PROP_MAC_RX_ON_WHEN_IDLE_MODE, &rx_on_when_idle);
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
        // Alternate short address — only if the RCP advertises the capability
        // (mirrors upstream `RadioSpinel::SetAlternateShortAddress`, which gates
        // the property set on `OT_RADIO_CAPS_ALT_SHORT_ADDR`). Stock RCPs without
        // it simply never receive the property.
        if self.caps.contains(Capabilities::ALT_SHORT_ADDR)
            && changed(|c| c.alt_short_addr.unwrap_or(0xfffe) as u64)
        {
            batch[count] = (PROP_MAC_15_4_ALT_SADDR, &alt_short_addr);
            count += 1;
        }
        if changed(|c| c.ext_addr.unwrap_or(0)) {
            batch[count] = (PROP_MAC_15_4_LADDR, &ext_addr);
            count += 1;
        }

        self.set_props(batch[..count].iter().copied()).await?;

        self.state.config = Some(config.clone());
        Ok(())
    }

    /// Ensure raw-stream RX is enabled (so the RCP forwards received frames).
    async fn ensure_channel(&mut self, channel: u8) -> Result<(), RadioErrorKind> {
        if self.channel != channel {
            self.set_prop(PROP_PHY_CHAN, &[channel]).await?;
            self.channel = channel;
        }

        Ok(())
    }

    /// Push a CCA threshold (dBm) to the RCP.
    async fn ensure_cca_threshold(&mut self, threshold: i8) -> Result<(), RadioErrorKind> {
        if self.cca_threshold != threshold {
            self.set_prop(PROP_PHY_CCA_THRESHOLD, &[threshold as u8])
                .await?;
            self.cca_threshold = threshold;
        }

        Ok(())
    }

    async fn ensure_rx_enabled(&mut self, enabled: bool) -> Result<(), RadioErrorKind> {
        if self.rx_enabled != enabled {
            self.set_prop(PROP_MAC_RAW_STREAM_ENABLED, &[enabled as u8])
                .await?;
            self.rx_enabled = enabled;
        }
        Ok(())
    }

    /// Send an `ot-rcp` **manufacturing / RF diagnostics** command (`diag …`) to
    /// the radio co-processor and collect its textual reply into `out`, returning
    /// the number of bytes written.
    ///
    /// This is a bench / bring-up utility for exercising the RCP's *radio
    /// hardware* directly — RF tone output (`diag cw`), packet-error-rate tests
    /// (`diag send` / `diag stats`), channel and power setup, etc. It is **not**
    /// part of normal Thread operation:
    ///
    /// - The RCP firmware must be built with diagnostics support
    ///   (`OPENTHREAD_CONFIG_DIAG_ENABLE`); a stock non-diag `ot-rcp` replies with
    ///   an error string.
    /// - `diag start` puts the radio into a mode in which it does **not** perform
    ///   normal Thread TX/RX. Because this takes `&mut self`, it cannot be called
    ///   while [`OpenThread::run`](crate::OpenThread::run) owns the radio — so it
    ///   is naturally a *before-`run`* tool. Run `diag stop` before handing the
    ///   radio to the stack.
    ///
    /// `command` is the full diag command line (e.g. `"diag channel 20"`), sent
    /// verbatim over `SPINEL_PROP_NEST_STREAM_MFG`. Output is collected from the
    /// matched reply plus any further output lines that arrive within
    /// [`RESPONSE_TIMEOUT`] (some commands stream several lines), concatenated
    /// into `out` and truncated to its length.
    #[cfg(feature = "diag")]
    pub async fn diag(&mut self, command: &str, out: &mut [u8]) -> Result<usize, RadioErrorKind> {
        self.ensure_init().await?;

        let prop = crate::sys::SPINEL_PROP_NEST_STREAM_MFG as u32;

        // `SPINEL_DATATYPE_UTF8_S` is a NUL-terminated string: command bytes + NUL.
        let mut payload = [0u8; MAX_SPINEL_FRAME];
        let n = command.len();
        if n + 1 > payload.len() {
            return Err(RadioErrorKind::TxFailed);
        }
        payload[..n].copy_from_slice(command.as_bytes());
        payload[n] = 0;

        // Send as a PROP_VALUE_SET; the matched response carries the first line.
        let (_p, off) = self
            .send_prop_await(prop, &payload[..=n], RESPONSE_TIMEOUT)
            .await?;
        let mut written = copy_utf8(&self.rx_frame[off..self.rx_len], out, 0);

        // Drain any further streamed output lines (unsolicited
        // PROP_VALUE_IS(NEST_STREAM_MFG)) until the RCP goes quiet. No RX
        // stashing here (unlike `await_response`): `diag` is a before-`run`
        // tool (see above), so the raw stream has never been enabled and no
        // `STREAM_RAW` frames can arrive.
        while written < out.len() {
            let Ok(len) = self.recv_frame(RESPONSE_TIMEOUT).await else {
                break; // timeout => no more output
            };
            let Some((_tid, rcmd, rprop, o)) = spinel_parse_header(&self.rx_frame[..len]) else {
                continue;
            };
            if rcmd == CMD_PROP_VALUE_IS && rprop == prop {
                written = copy_utf8(&self.rx_frame[o..len], out, written);
            }
        }

        Ok(written)
    }
}

impl<T> Radio for SpinelRadio<'_, T>
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
            receive_sensitivity: self.sensitivity,
            default_tx_power: self.default_tx_power,
            default_cca_threshold: self.default_cca_threshold,
            // The RCP's clock, once its offset to the host clock is known (see
            // the module docs).
            clock: rcp_time_offset().map(|_| RadioClock(rcp_now_us)),
            csl_accuracy_ppm: self.csl_accuracy_ppm,
            csl_uncertainty: self.csl_uncertainty,
            bus_speed: self.bus_speed,
            bus_latency_us: self.bus_latency_us,
        })
    }

    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error> {
        self.ensure_init().await?;
        self.flush_src_match().await?;
        self.flush_config(config).await
    }

    async fn set_receive(&mut self, channel: u8) -> Result<(), Self::Error> {
        self.ensure_init().await?;
        self.flush_src_match().await?;

        // The RCP receives on the channel property; the raw stream is what
        // makes it forward the frames to us.
        self.ensure_channel(channel).await?;
        self.ensure_rx_enabled(true).await
    }

    async fn set_sleep(&mut self) -> Result<(), Self::Error> {
        self.ensure_init().await?;

        // Stop the RCP from streaming frames up. This is the closest thing to
        // "park" the spinel raw-MAC surface offers: `PROP_MAC_RAW_STREAM_ENABLED`
        // is what the upstream POSIX host toggles too.
        self.ensure_rx_enabled(false).await
    }

    async fn set_src_match_config(&mut self, entries: &SrcMatchConfig) -> Result<(), Self::Error> {
        self.state.src_match = entries.clone();
        self.src_match_dirty = true;

        // Push right away when the link is already up; before the handshake
        // the dirty flag keeps it pending and the operation prologues flush
        // it (which also re-delivers the table across an RCP re-init).
        if self.eui64.is_some() {
            self.flush_src_match().await?;
        }

        Ok(())
    }

    async fn set_mac_keys(&mut self, keys: Option<&MacKeys>) -> Result<(), Self::Error> {
        // Only an RCP that secures frames itself has a use for the keys. Taking
        // them away is not a spinel operation; the RCP keeps the last ones,
        // which it uses only for frames it is asked to secure.
        let Some(keys) = keys.filter(|_| self.caps.contains(Capabilities::TRANSMIT_SEC)) else {
            return Ok(());
        };

        self.ensure_init().await?;

        // Key ID mode (as OpenThread encodes it in the security control
        // field, which is what the RCP hands its own radio platform), key
        // index, then the previous, current and next keys, each as
        // data-with-length.
        let mut payload = [0u8; 2 + 3 * (2 + 16)];
        payload[0] = keys.key_id_mode << 3;
        payload[1] = keys.key_id;
        for (index, key) in [&keys.prev, &keys.curr, &keys.next].into_iter().enumerate() {
            let at = 2 + index * 18;
            payload[at..at + 2].copy_from_slice(&16u16.to_le_bytes());
            payload[at + 2..at + 18].copy_from_slice(key);
        }

        self.set_prop(PROP_RCP_MAC_KEY, &payload).await
    }

    async fn set_mac_frame_counter(
        &mut self,
        frame_counter: u32,
        if_larger: bool,
    ) -> Result<(), Self::Error> {
        if !self.caps.contains(Capabilities::TRANSMIT_SEC) {
            return Ok(());
        }

        self.ensure_init().await?;

        let mut payload = [0u8; 5];
        payload[..4].copy_from_slice(&frame_counter.to_le_bytes());
        payload[4] = if_larger as u8;

        self.set_prop(PROP_RCP_MAC_FRAME_COUNTER, &payload).await
    }

    async fn energy_scan(&mut self, channel: u8, duration_millis: u16) -> Result<i8, Self::Error> {
        self.ensure_init().await?;

        // Configure and start the scan on the RCP, checking each ack: an RCP
        // that cannot scan at all (its radio lacks an energy detector and its
        // firmware was built without the software-scan fallback) rejects the
        // scan-state write with a `LAST_STATUS` error, and that must fail the
        // scan rather than leave us waiting for a result that never comes.
        let period = duration_millis.to_le_bytes();

        for (prop, payload) in [
            (PROP_MAC_SCAN_MASK, &[channel][..]),
            (PROP_MAC_SCAN_PERIOD, &period[..]),
            (PROP_MAC_SCAN_STATE, &[SCAN_STATE_ENERGY][..]),
        ] {
            let (rprop, _off) = self
                .send_prop_await(prop, payload, RESPONSE_TIMEOUT)
                .await?;
            if rprop != prop {
                // The ack did not echo the property we set - typically a
                // `LAST_STATUS` carrying an error such as "unimplemented".
                warn!(
                    "Energy scan: RCP rejected the 0x{:02x} property write",
                    prop
                );
                return Err(RadioErrorKind::Other);
            }
        }

        // Wait for the unsolicited per-channel scan result notification,
        // stashing any radio frames received meanwhile (see `try_stash_rx`).
        //
        // If this future is dropped before the result arrives (e.g. the radio
        // runner preempts the scan with a new command), the stray notification
        // is simply ignored by the other wire-read loops later.
        let timeout = RESPONSE_TIMEOUT + Duration::from_millis(duration_millis as u64);

        loop {
            let frame_len = self.recv_frame(timeout).await?;

            if self.try_stash_rx(frame_len) {
                continue;
            }

            let frame = &self.rx_frame[..frame_len];
            let Some((_tid, rcmd, rprop, off)) = spinel_parse_header(frame) else {
                continue;
            };

            if rcmd == CMD_PROP_VALUE_IS && rprop == PROP_MAC_ENERGY_SCAN_RESULT {
                // Payload ("Cc"): channel (u8) + max RSSI (i8).
                let body = &frame[off..];
                if body.len() < 2 {
                    return Err(RadioErrorKind::Other);
                }

                return Ok(body[1] as i8);
            }
            // Other frames (e.g. the scan-state-back-to-idle notification) — ignore.
        }
    }

    async fn transmit(
        &mut self,
        psdu: &mut [u8],
        psdu_tx: &mut crate::PsduTxInfo,
        channel: u8,
        power: i8,
        cca_threshold: Option<i8>,
        ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduRxInfo>, Self::Error> {
        self.ensure_init().await?;
        self.flush_src_match().await?;
        self.ensure_time_sync().await;

        // The frame carries its own channel and power below, and where the
        // receiver goes afterwards (`rxChannelAfterTxDone`): back to the channel
        // it receives on. So the RCP's channel property is left alone - a frame
        // on another channel (to a CSL child listening on its own CSL channel)
        // must not leave the receiver there, deaf to the network channel, until
        // the next receive command crosses the link. The CCA threshold has no
        // per-frame slot at all - it is a property, pushed only when it moves.
        if let Some(threshold) = cca_threshold {
            self.ensure_cca_threshold(threshold).await?;
        }

        let tx_power = power;

        let secured = psdu.first().is_some_and(|fcf| fcf & 0x08 != 0);

        // A frame timed into a CSL child's receive window: the RCP times it, as
        // a delay from about now - all in the RCP's clock, which is the radio
        // clock the stack timed the frame by. One that could not cross the
        // link in time any more goes out right away instead, as OpenThread's
        // own timing would send it.
        let (tx_delay_base, tx_delay) = psdu_tx
            .tx_at_us
            .filter(|_| rcp_time_offset().is_some())
            .and_then(|tx_at_us| {
                let now = rcp_now_us() as i64;
                let at = tx_at_us as i64;

                // The frame's way to the RCP (with the spinel and HDLC
                // overhead), plus a margin for the RCP to schedule it.
                let transfer_us = if self.bus_speed > 0 {
                    (psdu.len() as i64 + 32) * 8 * 1_000_000 / self.bus_speed as i64
                } else {
                    0
                } + self.bus_latency_us as i64;

                let timed =
                    (at - now > transfer_us + 500).then_some((now as u32, (at - now) as u32));

                if timed.is_none() {
                    debug!(
                        "Timed frame {} us late for the RCP, sending it right away",
                        transfer_us + 500 - (at - now)
                    );
                }

                timed
            })
            .unwrap_or((0, 0));

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
        // handle them: as many backoffs as OpenThread wants (802.15.4's
        // macMaxCSMABackoffs=4 unless it says), and macMaxFrameRetries=3. A
        // timed frame gets no retries: a retry would miss the window, and
        // OpenThread aims the retransmission at the next one itself.
        payload[n] = psdu_tx.max_csma_backoffs.unwrap_or(4); // maxCsmaBackoffs
        n += 1;
        payload[n] = if tx_delay != 0 { 0 } else { 3 }; // maxFrameRetries
        n += 1;
        payload[n] = cca_threshold.is_some() as u8; // csmaCaEnabled
        n += 1;
        payload[n] = psdu_tx.header_updated as u8; // isHeaderUpdated
        n += 1;
        // isARetx: also set for any secured frame the host has secured already,
        // to keep the RCP's hands off its MAC header. RCP firmwares with a
        // transmit-security engine (e.g. the nRF `ot-rcp`, `ot-nrf528xx`
        // `radio.c` `otPlatRadioTransmit`) overwrite the frame counter and key
        // index of every secured key-id-mode-1 frame with their *own*
        // counter/key-id state — without consulting
        // `isHeaderUpdated`/`isSecurityProcessed` — unless the frame is marked
        // as a retransmission. For a frame secured on the host, a re-stamped
        // counter/key-id no longer matches the MIC, and every receiver silently
        // drops the frame after its radio has already acknowledged it.
        payload[n] = (psdu_tx.retransmission || (psdu_tx.security_processed && secured)) as u8;
        n += 1;
        payload[n] = psdu_tx.security_processed as u8; // isSecurityProcessed
        n += 1;
        payload[n..n + 4].copy_from_slice(&tx_delay.to_le_bytes()); // txDelay
        n += 4;
        payload[n..n + 4].copy_from_slice(&tx_delay_base.to_le_bytes()); // txDelayBaseTime
        n += 4;
        payload[n] = self.channel; // rxChannelAfterTxDone
        n += 1;
        payload[n] = tx_power as u8;
        n += 1;

        // Send STREAM_RAW and take the matched transmit-done response. Use the
        // long transmit timeout: a broadcast frame (no ACK) burns all CSMA
        // backoffs + MAC retries before the RCP reports done, which can take
        // seconds on a congested channel — see [`TRANSMIT_TIMEOUT`].
        let (_prop, off) = self
            .send_prop_await(PROP_STREAM_RAW, &payload[..n], TRANSMIT_TIMEOUT)
            .await?;

        // Parse the transmit-done body (from `RadioSpinel::HandleTransmitDone`):
        //   uint_packed status + bool framePending + bool headerUpdated
        //   + [if status OK] the ACK radio frame (same layout as an RX frame)
        //   + [if the RCP finished the frame's header] its key index (u8) and
        //     frame counter (u32).
        // We advertise `TX_ACK`, so OpenThread expects us to return the received
        // ACK here rather than have `MacRadio` synthesize it in software.
        let body_end = self.rx_len;
        let body = &self.rx_frame[off..body_end];

        let Some((status, p)) = spinel_uint_decode(body) else {
            return Ok(None);
        };
        // framePending (bool, 1 byte) + headerUpdated (bool, 1 byte).
        let Some(header_updated) = body.get(p + 1).map(|&updated| updated != 0) else {
            return Ok(None);
        };
        let mut rest = &body[p + 2..];

        // status != OK → the transmit failed (no ACK / channel access), and
        // there is no ACK frame; reported as an error below, once the frame
        // counter it used is handed back.
        let status_ok = status == 0; // SPINEL_STATUS_OK
        if !status_ok && tx_delay != 0 {
            debug!("Timed frame failed on the RCP: spinel status {}", status);
        }
        let ack = if status_ok {
            parse_radio_frame(rest).map(|(ack_psdu, meta)| {
                let used = ack_psdu.len() + 2 + meta.len;
                (ack_psdu, meta, used)
            })
        } else {
            None
        };
        if let Some((_, _, used)) = &ack {
            rest = rest.get(*used..).unwrap_or(&[]);
        }

        // The RCP assigned the frame its counter and key index: write them into
        // the frame we hold, so that OpenThread reads the ones used and a
        // retransmission repeats them. (The frame stays as we sent it
        // otherwise - the RCP does not hand back the secured frame - so it is
        // not `security_processed`: a retransmission is secured by the RCP again.)
        if !psdu_tx.header_updated && header_updated && secured {
            if let (Some(&key_id), Some(counter)) = (rest.first(), rest.get(1..5)) {
                let counter = u32::from_le_bytes([counter[0], counter[1], counter[2], counter[3]]);

                let mut frame: crate::sys::otRadioFrame = unsafe { core::mem::zeroed() };
                frame.mPsdu = psdu.as_mut_ptr();
                frame.mLength = psdu.len() as _;

                unsafe {
                    crate::sys::otMacFrameSetKeyId(&mut frame, key_id);
                    crate::sys::otMacFrameSetFrameCounter(&mut frame, counter);
                }

                psdu_tx.header_updated = true;
            }
        }

        if !status_ok {
            return Err(tx_status_error(status));
        }

        let Some((ack_psdu, ack_meta, _)) = ack else {
            return Ok(None);
        };

        match ack_psdu_buf {
            Some(buf) => {
                let copy = ack_psdu.len().min(buf.len());
                buf[..copy].copy_from_slice(&ack_psdu[..copy]);
                Ok(Some(PsduRxInfo {
                    len: copy,
                    channel: ack_meta.channel.unwrap_or(channel),
                    rssi: ack_meta.rssi,
                    lqi: ack_meta.lqi,
                    timestamp_us: ack_meta.timestamp,
                    ack_security: None,
                    acked_with_frame_pending: None,
                }))
            }
            // The caller didn't ask for the ACK PSDU (didn't expect an ACK), so
            // there is nothing to report even though the transmit succeeded.
            None => Ok(None),
        }
    }

    async fn receive(&mut self, psdu_buf: &mut [u8]) -> Result<PsduRxInfo, Self::Error> {
        self.ensure_init().await?;
        self.flush_src_match().await?;
        self.ensure_time_sync().await;
        // Normally already done by `set_receive`; re-asserted here because a
        // `receive` may also follow a transmit, and because it is cheap (the
        // property is only written when it actually changes).
        self.ensure_rx_enabled(true).await?;

        // Fallback for frames whose metadata lacks the PHY-data struct; the
        // parsed per-frame channel is preferred (a stashed frame may have been
        // received on a different channel than the one the RCP is tuned to
        // now, e.g. during an active scan).
        let cfg_channel = self.channel;

        // First return any frame that was stashed while we were busy waiting for
        // a command response (see `try_stash_rx`). This is the common case —
        // inbound frames usually arrive during a transmit.
        while let Some(stashed) = self.rx_queue.pop_front() {
            if let Some((psdu, meta)) = parse_radio_frame(&stashed) {
                let copy = psdu.len().min(psdu_buf.len());
                psdu_buf[..copy].copy_from_slice(&psdu[..copy]);
                return Ok(PsduRxInfo {
                    len: copy,
                    channel: meta.channel.unwrap_or(cfg_channel),
                    rssi: meta.rssi,
                    lqi: meta.lqi,
                    timestamp_us: meta.timestamp,
                    ack_security: meta.ack_security,
                    acked_with_frame_pending: Some(meta.acked_with_frame_pending),
                });
            }
            // Unparseable stashed frame — skip and try the next.
        }

        // Nothing queued: read the wire until an unsolicited `STREAM_RAW` arrives.
        loop {
            let frame_len = self.recv_frame(Duration::from_secs(3600)).await?;
            let frame = &self.rx_frame[..frame_len];
            let Some((tid, rcmd, rprop, off)) = spinel_parse_header(frame) else {
                continue;
            };

            // Unsolicited STREAM_RAW notification = a received frame.
            if tid == 0 && rcmd == CMD_PROP_VALUE_IS && rprop == PROP_STREAM_RAW {
                let Some((psdu, meta)) = parse_radio_frame(&frame[off..]) else {
                    continue;
                };

                let copy = psdu.len().min(psdu_buf.len());
                psdu_buf[..copy].copy_from_slice(&psdu[..copy]);

                return Ok(PsduRxInfo {
                    len: copy,
                    channel: meta.channel.unwrap_or(cfg_channel),
                    rssi: meta.rssi,
                    lqi: meta.lqi,
                    timestamp_us: meta.timestamp,
                    ack_security: meta.ack_security,
                    acked_with_frame_pending: Some(meta.acked_with_frame_pending),
                });
            }
            // Other frames (matched responses to a concurrent op, status) — ignore.
        }
    }
}

/// The driver's larger-than-a-register state, kept in the resources rather
/// than in [`SpinelRadio`] itself so the radio value - held across `await`
/// points by the radio loop's futures - stays small: the two source-match
/// tables (~170 bytes each) and the last-applied [`Config`].
struct SpinelRadioState {
    /// The latest source-match table from the stack; `src_match_dirty` in the
    /// radio marks it not yet pushed to the RCP.
    src_match: SrcMatchConfig,
    /// The source-match table as the RCP currently has it: what the
    /// per-entry INSERT/REMOVE flush has successfully applied so far. The
    /// diff between this and `src_match` is what a flush sends.
    src_match_flushed: SrcMatchConfig,
    /// Last-applied config; used to only re-send changed properties.
    config: Option<Config>,
}
