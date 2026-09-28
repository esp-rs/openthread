//! IEEE 802.15.4 PHY Radio trait and associated types for OpenThread.
//!
//! `openthread` operates the radio in terms of this trait, which is implemented by the actual radio driver.

#![allow(clippy::unnecessary_cast)]

use core::fmt::Debug;

use openthread_sys::{
    OT_RADIO_CAPS_ALT_SHORT_ADDR, OT_RADIO_CAPS_RX_ON_WHEN_IDLE, OT_RADIO_CAPS_TRANSMIT_FRAME_POWER,
};

use crate::fmt::bitflags;
use crate::sys::{
    OT_RADIO_CAPS_ACK_TIMEOUT, OT_RADIO_CAPS_CSMA_BACKOFF, OT_RADIO_CAPS_ENERGY_SCAN,
    OT_RADIO_CAPS_RECEIVE_TIMING, OT_RADIO_CAPS_SLEEP_TO_TX, OT_RADIO_CAPS_TRANSMIT_RETRIES,
    OT_RADIO_CAPS_TRANSMIT_SEC, OT_RADIO_CAPS_TRANSMIT_TIMING,
};

pub use mac::*;
pub use proxy::*;

// Concrete [`Radio`] implementations for the supported radio hardware /
// deployments. Each is gated on the feature that enables it.
//
// - `esp` / `nrf`: the 802.15.4 radio is local to this MCU (SoC deployment).
// - `spinel`: the radio lives on a *separate* chip (an OpenThread RCP) reached
//   over a UART/SPI spinel link — an RCP-host deployment. (The feature is named
//   `rcp` after that deployment role; the module is named `spinel` after the
//   wire protocol it speaks.)
#[cfg(feature = "esp-radio")]
pub mod esp;
mod mac;
#[cfg(feature = "embassy-nrf")]
pub mod nrf;
mod proxy;
#[cfg(feature = "rcp")]
pub mod spinel;

/// The error kind for radio errors.
// TODO: Fill in with extra variants
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RadioErrorKind {
    /// Invalid TX frame
    TxInvalid,
    /// Invalid RX frame
    RxInvalid,
    /// Receiving failed
    RxFailed,
    /// Transmitting failed
    TxFailed,
    /// Receiving failed due to sending an ACK frame failed
    TxAckFailed,
    /// Transmitting failed due to receiving an ACK frame failed
    RxAckFailed,
    /// Receiving failed due to timeout when preparing an ACK frame
    TxAckTimeout,
    /// Transmitting failed due to no ACK received
    RxAckTimeout,
    /// Transmitting failed due to invalid ACK received
    RxAckInvalid,
    /// Other radio error
    Other,
}

/// The error type for radio errors.
pub trait RadioError: Debug {
    /// The kind of error.
    fn kind(&self) -> RadioErrorKind;
}

impl RadioError for RadioErrorKind {
    fn kind(&self) -> RadioErrorKind {
        *self
    }
}

bitflags! {
    /// Radio capabilities - a mirror of the C `otRadioCaps` flags, reported
    /// verbatim to the OpenThread stack via `otPlatRadioGetCaps`.
    ///
    /// Not all of these are PHY-level: several
    /// (ACK timeout, CSMA backoff, frame security, auto-sleep) describe MAC-layer
    /// intelligence that a capable radio driver owns below the [`Radio`] trait.
    #[repr(transparent)]
    #[derive(Default)]
    #[cfg_attr(not(feature = "defmt"), derive(Debug, Copy, Clone, Eq, PartialEq, Hash))]
    pub struct Capabilities: u16 /*: otRadioCaps - defmt::bitflags! can't grok this*/ {
        /// Radio supports ACK timeout for transmitted frames.
        const ACK_TIMEOUT = OT_RADIO_CAPS_ACK_TIMEOUT as u16;
        /// Radio supports energy scan.
        const ENERGY_SCAN = OT_RADIO_CAPS_ENERGY_SCAN as u16;
        /// Radio supports automatic retransmission of unacknowledged frames.
        const TRANSMIT_RETRIES = OT_RADIO_CAPS_TRANSMIT_RETRIES as u16;
        /// Radio supports CSMA/CA backoff for frame transmission.
        const CSMA_BACKOFF = OT_RADIO_CAPS_CSMA_BACKOFF as u16;
        /// Radio supports direct transition from sleep to TX.
        const SLEEP_TO_TX = OT_RADIO_CAPS_SLEEP_TO_TX as u16;
        /// Radio finishes the frames it transmits itself: frame counter,
        /// CSL IE and AES-CCM* at transmit time (see [`TxInfo`]), with the
        /// keys from [`Radio::set_mac_keys`]. A CSL child needs this from its
        /// radio to advertise an accurate phase; for every other radio the
        /// glue does the same work in software before `transmit`.
        const TRANSMIT_SEC = OT_RADIO_CAPS_TRANSMIT_SEC as u16;
        /// Radio supports precise TX timing.
        const TRANSMIT_TIMING = OT_RADIO_CAPS_TRANSMIT_TIMING as u16;
        /// Radio supports precise RX timing.
        const RECEIVE_TIMING = OT_RADIO_CAPS_RECEIVE_TIMING as u16;
        /// Radio supports autonomous receiver power-off during idle periods.
        /// Requested by OpenThread explicitly via [`Config::auto_sleep`].
        const AUTO_SLEEP = OT_RADIO_CAPS_RX_ON_WHEN_IDLE as u16;
        /// Radio supports setting the transmit frame power.
        const TRANSMIT_FRAME_POWER = OT_RADIO_CAPS_TRANSMIT_FRAME_POWER as u16;
        /// Radio supports alternative short address.
        const ALT_SHORT_ADDR = OT_RADIO_CAPS_ALT_SHORT_ADDR as u16;
    }
}

bitflags! {
    /// Radio MAC capabilities: the parts of IEEE 802.15.4 MAC processing that
    /// the radio driver owns natively - whether in hardware, in driver
    /// software, or a mix is the driver's concern, not this crate's.
    ///
    /// `MacCapabilities::all()` means a fully offloaded MAC (no software
    /// emulation needed), `MacCapabilities::empty()` a bare PHY-like radio that
    /// needs the complete soft-MAC. Anything short of [`Self::REQUIRED`] has to
    /// be wrapped by the user in a [`MacRadio`], which emulates the difference
    /// in software.
    #[repr(transparent)]
    #[derive(Default)]
    #[cfg_attr(not(feature = "defmt"), derive(Debug, Copy, Clone, Eq, PartialEq, Hash))]
    pub struct MacCapabilities: u16 {
        /// Radio supports automatic reception of ACKs for transmitted frames.
        const TX_ACK = 0x01;
        /// Radio supports automatic sending of ACKs for received frames.
        const RX_ACK = 0x02;
        /// Radio supports promiscuous mode.
        const PROMISCUOUS = 0x04;
        /// Radio supports filtering of PHY frames by their PAN ID in the MAC payload.
        const FILTER_PAN_ID = 0x08;
        /// Radio supports filtering of PHY frames by their short address in the MAC payload.
        const FILTER_SHORT_ADDR = 0x10;
        /// Radio supports filtering of PHY frames by their extended address in the MAC payload.
        const FILTER_EXT_ADDR = 0x20;
        /// The radio's ACK engine honors the source-address-match table when deciding whether to raise the
        /// Frame Pending bit.
        ///
        /// A radio doing its own RX ACKs *without* this capability should answer every data poll FP = 1.
        const SRC_MATCH = 0x40;
    }
}

impl MacCapabilities {
    /// The MAC-offload set a radio must provide to be driven by the OpenThread stack.
    ///
    /// This is everything except the two capabilities a software layer above
    /// the radio cannot supply, and whose absence costs a diagnostic or an
    /// optimization rather than correct Thread operation:
    ///
    /// - [`SRC_MATCH`](Self::SRC_MATCH): the source-match table only matters to
    ///   whoever sends the ACKs, so a radio doing its own RX ACKs without one
    ///   answers every data poll with Frame Pending set - protocol-safe
    ///   over-promising that nothing above it can improve on.
    /// - [`PROMISCUOUS`](Self::PROMISCUOUS): a wrapper can only *add* filtering
    ///   to what a radio delivers, never recover frames the radio's own filter
    ///   already dropped. A radio that filters in hardware but cannot be told
    ///   to stop simply cannot sniff - and sniffing is not part of operating a
    ///   Thread network.
    ///
    /// A radio reporting less than this - a bare PHY, typically - must be
    /// wrapped by the user in a [`MacRadio`], which emulates the missing pieces
    /// in software.
    pub const REQUIRED: Self = Self::all()
        .difference(Self::SRC_MATCH)
        .difference(Self::PROMISCUOUS);

    /// Panic unless these capabilities cover [`Self::REQUIRED`].
    pub(crate) fn assert_required(&self) {
        assert!(
            self.contains(Self::REQUIRED),
            "Radio is missing MAC capabilities required by OpenThread: {:?}. \
             Wrap it in a `MacRadio` to have them emulated in software.",
            Self::REQUIRED.difference(*self)
        );
    }
}

/// The full capability set of a radio, as reported by [`Radio::init`].
///
/// Both halves are discovered at runtime (a radio may only learn them by talking
/// to the hardware — e.g. a remote co-processor reporting them during its startup
/// handshake), which is why they are returned from `init` rather than declared as
/// consts:
/// - [`phy`](RadioCaps::phy): the PHY capabilities, reported to the OpenThread C
///   stack (`otPlatRadioGetCaps`).
/// - [`mac`](RadioCaps::mac): the MAC-offloading capabilities. Any of these the
///   radio lacks are emulated in software by the `MacRadio` wrapper (which reads
///   this set at runtime).
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RadioCaps {
    /// The PHY capabilities.
    pub phy: Capabilities,
    /// The MAC-offloading capabilities.
    pub mac: MacCapabilities,
    /// The radio's receive sensitivity, in dBm.
    pub receive_sensitivity: i8,
    /// The radio's default transmit power, in dBm.
    pub default_tx_power: i8,
    /// The radio's default CCA threshold, in dBm.
    pub default_cca_threshold: i8,
    /// The radio's microsecond clock, if it has one. Required for
    /// [`Capabilities::RECEIVE_TIMING`]; without it OpenThread's own (coarser)
    /// clock is used for the timing it reports to peers.
    pub clock: Option<RadioClock>,
    /// The accuracy of the radio clock, in PPM, as reported to a CSL parent so
    /// it can size the window it transmits in. `u8::MAX` if unknown (the
    /// parent then assumes the worst).
    pub csl_accuracy_ppm: u8,
    /// The fixed uncertainty (jitter) of the radio's timed receive, in units of
    /// 10 µs, as reported to a CSL parent. `u8::MAX` if unknown.
    pub csl_uncertainty: u8,
}

impl RadioCaps {
    /// The OpenThread core's own default receive sensitivity (dBm), for
    /// drivers that do not (yet) report a hardware-specific figure.
    pub const DEFAULT_RECEIVE_SENSITIVITY: i8 = -110;

    /// A default CCA threshold used when constructing default `RadioCaps`.
    pub const DEFAULT_CCA_THRESHOLD: i8 = -60;

    /// A default transmit power used when constructing default `RadioCaps`.
    pub const DEFAULT_TX_POWER: i8 = 12;
}

impl Default for RadioCaps {
    fn default() -> Self {
        Self {
            phy: Capabilities::empty(),
            mac: MacCapabilities::empty(),
            receive_sensitivity: Self::DEFAULT_RECEIVE_SENSITIVITY,
            default_tx_power: Self::DEFAULT_TX_POWER,
            default_cca_threshold: Self::DEFAULT_CCA_THRESHOLD,
            clock: None,
            csl_accuracy_ppm: u8::MAX,
            csl_uncertainty: u8::MAX,
        }
    }
}

/// Radio configuration.
#[derive(Debug, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Allow the radio to autonomously power its receiver down during idle
    /// periods, instead of keeping it in RX.
    /// Disregarded unless the radio advertises [`Capabilities::AUTO_SLEEP`],
    /// otherwise emulated by OpenThread itself by issuing explicit "go to sleep"
    /// commands to the radio.
    ///
    /// Auto-sleep is only relevant and enabled for MTD devices which are battery
    /// powered and need to conserve power.
    pub auto_sleep: bool,
    /// Promiscuous mode (receive all frames regardless of address filtering)
    /// Disregarded if the radio is not capable of operating in promiscuous mode
    /// and emulated by [`MacRadio`].
    pub promiscuous: bool,
    /// PAN ID filter
    /// Disregarded if the radio is not capable of filtering by PAN ID
    /// and emulated by [`MacRadio`].
    pub pan_id: Option<u16>,
    /// Short address filter
    /// Disregarded if the radio is not capable of filtering by short address
    /// and emulated by [`MacRadio`].
    pub short_addr: Option<u16>,
    /// Alternate short address filter.
    ///
    /// A *second* short address the radio should also accept frames for, in
    /// addition to [`short_addr`](Config::short_addr). Used by an FTD during a
    /// child-to-router role transition, when it is briefly reachable at both its
    /// old (child) and new (router) RLOC16 (OpenThread sets it via
    /// `otPlatRadioSetAlternateShortAddress` and clears it ~8s later). `None`
    /// means "no alternate" — the common case.
    ///
    /// Honored by the software [`MacRadio`] filter and by radios that can match a
    /// second short address; disregarded by radios whose hardware/co-processor
    /// short-address filter accepts only a single address (see each driver).
    pub alt_short_addr: Option<u16>,
    /// Extended address filter
    /// Disregarded if the radio is not capable of filtering by extended address
    /// and emulated by [`MacRadio`].
    pub ext_addr: Option<u64>,
}

impl Config {
    /// Create a new default configuration.
    pub const fn new() -> Self {
        Self {
            auto_sleep: false,
            promiscuous: false,
            pan_id: None,
            short_addr: None,
            alt_short_addr: None,
            ext_addr: None,
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

/// Capacity of the [`SrcMatchConfig`] table.
/// Sized after OpenThread's default max-children count (10) with headroom.
///
/// On overflow the glue answers `OT_ERROR_NO_BUFS`, which OpenThread handles by
/// falling back to frame-pending-on-every-ack for the un-tracked children.
pub const SRC_MATCH_CAPACITY: usize = 16;

/// The source-address-match table, i.e. the set of sleepy children the stack
/// currently has pending indirect frames for.
#[derive(Debug, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SrcMatchConfig {
    /// Whether source matching is active. When `false`, polls are answered
    /// FP = 1 regardless of the tables.
    pub enabled: bool,
    /// The short (RLOC16) entries.
    pub short_addrs: heapless::Vec<u16, SRC_MATCH_CAPACITY>,
    /// The extended (EUI-64) entries.
    pub ext_addrs: heapless::Vec<u64, SRC_MATCH_CAPACITY>,
}

impl SrcMatchConfig {
    pub const fn new() -> Self {
        Self {
            enabled: false,
            short_addrs: heapless::Vec::new(),
            ext_addrs: heapless::Vec::new(),
        }
    }

    /// The Frame Pending answer for an ack-requesting MAC command frame
    /// arriving from `src_short` / `src_ext`.
    pub fn ack_frame_pending(&self, src_short: u16, src_ext: u64) -> bool {
        !self.enabled || self.short_addrs.contains(&src_short) || self.ext_addrs.contains(&src_ext)
    }
}

impl Default for SrcMatchConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Meta-data associated with the received IEEE 802.15.4 frame
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PsduMeta {
    /// Length of the PSDU in the frame
    pub len: usize,
    /// Channel on which the frame was received
    pub channel: u8,
    /// Received Signal Strength Indicator (RSSI) in dBm
    /// (if the radio supports appending it at the end of the frame, or `None` otherwise)
    pub rssi: Option<i8>,
    /// Link Quality Indicator (LQI) of the received frame, as reported by the
    /// radio; `None` if the radio does not report one, in which case the
    /// OpenThread glue synthesizes an LQI from the RSSI.
    pub lqi: Option<u8>,
    /// When the frame arrived: the time the end of its SFD (i.e. the start of
    /// the PHR) was at the antenna, in microseconds of the radio clock
    /// ([`RadioCaps::clock`]). `None` if the radio does not timestamp frames,
    /// in which case the OpenThread glue stamps the frame at delivery.
    ///
    /// Consumed by CSL synchronization (and Link Metrics / time-sync IEs), so
    /// a radio advertising [`Capabilities::RECEIVE_TIMING`] must report it.
    pub timestamp_us: Option<u64>,
    /// The security material the radio used for the *secured enhanced ACK* it
    /// sent for this frame, if it sent one. `None` for an immediate ACK, an
    /// unsecured enhanced ACK, or no ACK at all.
    ///
    /// OpenThread secures the data frames in software with its own frame
    /// counter, while the enhanced ACKs of a CSL child are secured by the radio
    /// with the same key: reporting the counter each ACK consumed is what
    /// keeps the two from colliding. A radio advertising
    /// [`Capabilities::RECEIVE_TIMING`] must report it.
    pub ack_security: Option<AckSecurity>,
}

/// What OpenThread knows about a frame it hands to [`Radio::transmit`], and
/// what the radio may have done to it.
///
/// A radio advertising [`Capabilities::TRANSMIT_SEC`] finishes the frame at
/// transmit time: unless `header_updated`, it assigns the frame counter (and
/// key index) and, when `csl_present`, fills the CSL IE with the phase of its
/// next receive window as of the moment the frame goes on the air; unless
/// `security_processed`, it then secures the frame (AES-CCM*) with the key
/// it was handed through [`Radio::set_mac_keys`]. It writes the finished
/// frame back into the PSDU buffer and sets `header_updated`.
///
/// Every other radio receives frames that are already finished and secured
/// by the OpenThread glue, with both flags set, and can ignore this.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TxInfo {
    /// The frame is already secured (or needs no security).
    pub security_processed: bool,
    /// The frame counter and the CSL IE are already assigned - a
    /// retransmission, or a frame the glue finished itself.
    pub header_updated: bool,
    /// The frame carries a CSL IE (a CSL child's own transmissions).
    pub csl_present: bool,
    /// The frame is a retransmission of a frame sent before.
    pub retransmission: bool,
}

/// The security material a radio used for a secured enhanced ACK
/// (see [`PsduMeta::ack_security`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AckSecurity {
    /// The frame counter of the ACK.
    pub frame_counter: u32,
    /// The key index (key ID mode 1) the ACK was secured with.
    pub key_id: u8,
}

/// A microsecond clock shared by everything timing-related in a radio: its
/// frame timestamps ([`PsduMeta::timestamp_us`]), its timed receive
/// ([`Radio::receive_at`]) and the CSL sample times ([`CslConfig`]).
///
/// A plain function, because OpenThread reads the clock synchronously
/// (`otPlatRadioGetNow`) from contexts where the `Radio` instance - owned by
/// the radio runner - is not reachable. A radio backed by a global driver
/// (the usual case for an SoC radio) simply hands out its driver's time
/// function.
#[derive(Clone, Copy)]
pub struct RadioClock(pub fn() -> u64);

impl RadioClock {
    /// The current time, in microseconds.
    pub fn now_us(&self) -> u64 {
        (self.0)()
    }
}

impl core::fmt::Debug for RadioClock {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("RadioClock")
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for RadioClock {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "RadioClock")
    }
}

// Two clocks are the same clock if they are the same function. The lint is
// about *identity* comparisons being unreliable across codegen units, which
// does not matter here: a false "different" only means a spurious re-apply.
#[allow(unpredictable_function_pointer_comparisons)]
impl PartialEq for RadioClock {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::fn_addr_eq(self.0, other.0)
    }
}

impl Eq for RadioClock {}

impl core::hash::Hash for RadioClock {
    fn hash<H: core::hash::Hasher>(&self, state: &mut H) {
        (self.0 as usize).hash(state)
    }
}

/// The CSL (Coordinated Sampled Listening) schedule of a Synchronized Sleepy
/// End Device, as OpenThread configures it in the radio.
///
/// The radio needs it to fill the CSL IE of the enhanced ACKs it sends to the
/// parent: the IE tells the parent when the child's next sample window is,
/// which is how the two stay synchronized without the child ever polling.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CslConfig {
    /// The CSL period, in units of 10 symbols (160 µs); `0` means CSL is off.
    pub period: u32,
    /// The short address of the parent (the peer the CSL IE is for).
    pub short_addr: u16,
    /// The extended address of the parent.
    pub ext_addr: u64,
    /// The time of the next CSL sample window, in microseconds of the radio
    /// clock (its low 32 bits, as OpenThread hands it out); the anchor from
    /// which the radio computes the CSL phase in each enhanced ACK. Updated by
    /// OpenThread before every sample window.
    pub sample_time_us: u32,
}

impl CslConfig {
    /// CSL off.
    pub const fn new() -> Self {
        Self {
            period: 0,
            short_addr: 0xfffe,
            ext_addr: 0,
            sample_time_us: 0,
        }
    }

    /// Whether CSL is enabled.
    pub const fn enabled(&self) -> bool {
        self.period != 0
    }
}

impl Default for CslConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// The MAC keys OpenThread hands the radio, so that the radio can secure the
/// enhanced ACKs it sends on the child's behalf (see
/// [`PsduMeta::ack_security`]).
///
/// All three keys use key ID mode 1: `key_id` is the index of `curr`, and
/// `prev` / `next` carry the neighboring indices (wrapping within `1..=128`),
/// so that an ACK to a frame secured with the previous or next rotation of the
/// network key can still be secured.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct MacKeys {
    /// The key ID mode (always 1 for Thread).
    pub key_id_mode: u8,
    /// The key index of `curr`.
    pub key_id: u8,
    /// The previous key.
    pub prev: [u8; 16],
    /// The current key.
    pub curr: [u8; 16],
    /// The next key.
    pub next: [u8; 16],
}

impl MacKeys {
    /// The key index of `prev`.
    pub const fn prev_key_id(&self) -> u8 {
        if self.key_id <= 1 {
            128
        } else {
            self.key_id - 1
        }
    }

    /// The key index of `next`.
    pub const fn next_key_id(&self) -> u8 {
        if self.key_id >= 128 {
            1
        } else {
            self.key_id + 1
        }
    }
}

// The key material never goes to a log.
impl core::fmt::Debug for MacKeys {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("MacKeys")
            .field("key_id_mode", &self.key_id_mode)
            .field("key_id", &self.key_id)
            .finish_non_exhaustive()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for MacKeys {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "MacKeys {{ key_id_mode: {}, key_id: {}, .. }}",
            self.key_id_mode,
            self.key_id
        )
    }
}

/// The IEEE 802.15.4 PHY Radio trait.
///
/// While the trait models the PHY layer of the radio, it might implement some "MAC-offloading"
/// capabilities as well - namely - the ability to automatically send and receive ACK frames,
/// the ability to filter received frames by PAN ID, short address, and extended address and others.
///
/// The stack requires all of [`MacCapabilities::REQUIRED`] from the radio it is
/// handed. An implementation offering less must be wrapped by the user in a
/// [`MacRadio`], which emulates the missing capabilities in software; the notes
/// below on what an implementation "can be a no-op for" are written from that
/// standpoint.
///
/// # Contract
///
/// OpenThread drives the radio as a small state machine - Sleep, Receive,
/// and then Energy Scan and the transmit sequence as an excursion out of
/// Receive - and holds the platform to semantics that are easy to violate
/// from the signatures alone (see `docs/radio-contract.md`).
///
/// An implementation MUST uphold:
/// 1. **`transmit` is the complete transmit sequence** per the declared
///    capabilities - CSMA/CCA, the transmission, and, for frames requesting
///    one, the ACK wait - resolved without the caller concurrently polling
///    `receive`. The matching ACK is consumed by `transmit` and reported in
///    its result; it must never surface via `receive`.
///
///    `transmit` is only allowed to be a pure "send this frame" without waiting
///    for an ACK when `MacCapabilities::TX_ACK` is NOT set - such a radio must
///    be wrapped in a [`MacRadio`], which polyfills the full semantics of
///    "`transmit` is the complete transmit sequence".
///
/// 2. **Reception outlives the calls**: frames arriving while no `receive`
///    is pending - including during `transmit`'s listening phases - are
///    neither lost nor silently consumed; they are delivered by subsequent
///    `receive` calls (typically from a driver-internal queue). A bounded
///    queue that drops on overflow is acceptable saturation behavior.
///
///    `receive` is only allowed to be a pure "wait and receive a frame now" -
///    without sending any ACKs and without accumulating anything outside the
///    `receive` method ONLY when BOTH `MacCapabilities::RX_ACK` and
///    `MacCapabilities::TX_ACK` are NOT set. After all, it is `transmit`'s
///    ACK wait that forces accumulation of RX frames outside of `receive`
///    in the first place. Such a radio must be wrapped in a [`MacRadio`],
///    which polyfills the full semantics of "Reception outlives the calls".
///
/// 3. **Cancellation is a sanctioned abort**: the `transmit` future may be
///    dropped mid-sequence (OpenThread aborts an ACK wait this way). The
///    frame may already be on the air; the radio must simply return to
///    receiving.
///
/// 4. **A sleeping radio misses frames** ([`Radio::set_sleep`]): frames arriving
///    while asleep are dropped, not buffered for later - a sleepy child
///    provably missing traffic is protocol behavior, not lost data.
///
/// The trait is NOT required to support the following operations:
/// - Re-sending a TX frame if the ACK frame was not received; this is done by OpenThread
/// - Dropping a duplicate RX frame; this is done by OpenThread
/// - MAC layer security; this is done by OpenThread
pub trait Radio {
    /// The error type for radio operations.
    type Error: RadioError;

    /// Bring the radio up and report its full [`RadioCaps`] (PHY + MAC-offload).
    ///
    /// Called once, before any [`set_config`](Radio::set_config) /
    /// [`transmit`](Radio::transmit) / [`receive`](Radio::receive), and before
    /// the OpenThread stack is first pumped — so the returned capabilities are
    /// cached and used (the PHY set is reported to the stack via
    /// `otPlatRadioGetCaps`; the MAC set drives the `MacRadio` wrapper's
    /// software-emulation decisions).
    ///
    /// This is the single source of *all* the radio's capabilities. Both the PHY
    /// and MAC sets are discovered here at runtime, because a radio may only learn
    /// them by talking to the hardware: a local SoC radio simply returns its
    /// fixed, statically-known set, while a radio backed by a remote co-processor
    /// performs a startup handshake and returns whatever that co-processor reports
    /// (which — for e.g. hardware crypto offload — cannot be known at compile
    /// time).
    async fn init(&mut self) -> Result<RadioCaps, Self::Error>;

    /// Set the radio configuration.
    ///
    /// NOTE:
    /// Can be a no-op or partial application for radios which are supporting only a subset of `MacCapabilities`,
    /// but such radios must be wrapped by the user in a [`MacRadio`] then.
    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error>;

    /// Set the radio source match configuration.
    ///
    /// NOTE:
    /// Can be a no-op for radios which are not supporting `MacCapabilities::SRC_MATCH`,
    /// but such radios must be wrapped by the user in a [`MacRadio`] then.
    async fn set_src_match_config(&mut self, config: &SrcMatchConfig) -> Result<(), Self::Error>;

    /// Set the radio to receive mode on `channel`.
    ///
    /// Arguments
    /// - `channel`: The channel to set the radio to receive on.
    ///
    /// NOTE:
    /// Can be a no-op for radios which are not supporting `MacCapabilities::TX_ACK` and `MacCapabilities::RX_ACK`,
    /// and are therefore not able to receive frames while waiting for an ACK frame in `transmit`.
    /// These radios naturally don't have an internal queue of received frames, so they don't need to be set to
    /// receive mode to receive frames, as they only do so when the `receive` method is called anyway.
    /// Can be a no-op for such radios, but they must be wrapped by the user in a [`MacRadio`] then.
    async fn set_receive(&mut self, channel: u8) -> Result<(), Self::Error>;

    /// Set the radio to sleep mode.
    ///
    /// NOTE:
    /// Can be a no-op for radios which are not supporting `MacCapabilities::TX_ACK` and `MacCapabilities::RX_ACK`,
    /// and are therefore not able to receive frames while waiting for an ACK frame in `transmit`.
    /// These radios naturally don't have an internal queue of received frames, so they don't need to be set to sleep
    /// mode to save power, as they only receive when the `receive` method is called anyway.
    async fn set_sleep(&mut self) -> Result<(), Self::Error>;

    /// Schedule a receive window: turn the receiver on for `channel` at `start_us`
    /// (radio clock, see [`RadioCaps::clock`]) for `duration_us`, then off again.
    ///
    /// This is how a CSL child samples the channel at its parent's transmit
    /// times. Frames received during the window are delivered through
    /// [`receive`](Radio::receive), which the runner keeps calling meanwhile -
    /// and which must NOT force the receiver on beyond the window (a radio that
    /// enters receive mode from `receive` has to notice it is in a timed window).
    /// The window is cancelled by [`set_sleep`](Radio::set_sleep) /
    /// [`set_receive`](Radio::set_receive) / [`transmit`](Radio::transmit).
    ///
    /// Only called on radios advertising [`Capabilities::RECEIVE_TIMING`]; the
    /// default is for the others and does nothing.
    async fn receive_at(
        &mut self,
        channel: u8,
        start_us: u64,
        duration_us: u32,
    ) -> Result<(), Self::Error> {
        let _ = (channel, start_us, duration_us);

        warn!("Timed receive is not supported by this radio");

        Ok(())
    }

    /// Set the CSL schedule the radio advertises in its enhanced ACKs.
    ///
    /// Only called on radios advertising [`Capabilities::RECEIVE_TIMING`], and
    /// then again before every sample window (the sample time moves). The
    /// default is for the others and does nothing.
    async fn set_csl(&mut self, csl: &CslConfig) -> Result<(), Self::Error> {
        let _ = csl;

        Ok(())
    }

    /// Hand the radio the MAC keys it secures its enhanced ACKs with, or take
    /// them away (`None`).
    ///
    /// Called on every radio whenever the network key rotates; only a radio
    /// that secures ACKs itself needs them. The default does nothing.
    async fn set_mac_keys(&mut self, keys: Option<&MacKeys>) -> Result<(), Self::Error> {
        let _ = keys;

        Ok(())
    }

    /// Set the MAC frame counter the radio secures its next enhanced ACK with
    /// (only if larger than its current one, when `if_larger` is set).
    ///
    /// OpenThread pushes its own counter here, so the ACK counters never run
    /// behind the data-frame counters. The default does nothing.
    async fn set_mac_frame_counter(
        &mut self,
        frame_counter: u32,
        if_larger: bool,
    ) -> Result<(), Self::Error> {
        let _ = (frame_counter, if_larger);

        Ok(())
    }

    /// Perform an energy scan on `channel`: measure the energy observed over
    /// `duration_millis` and return the maximum RSSI, in dBm.
    ///
    /// A radio that cannot measure channel energy keeps this default
    /// implementation, which completes immediately reporting "no measurement"
    /// (the 802.15.4 "invalid RSSI" value, +127 dBm); OpenThread omits such
    /// channels from the energy scan results, so a scan on such a radio
    /// cleanly yields *no* results rather than fake readings.
    ///
    /// Arguments
    /// - `channel`: The channel to perform the energy scan on.
    /// - `duration_millis`: The duration of the energy scan in milliseconds.
    ///
    /// NOTE: OpenThread's energy scan requests are always routed here,
    /// regardless of whether the radio reports [`Capabilities::ENERGY_SCAN`]
    /// (see the initial-`radio_caps` discussion in `lib.rs`: OpenThread
    /// snapshots the radio capabilities before the actual `Radio` instance is
    /// known, and its software-sampling fallback needs a synchronous RSSI
    /// read, which is unimplementable on top of this async trait).
    async fn energy_scan(&mut self, channel: u8, duration_millis: u16) -> Result<i8, Self::Error> {
        let _ = (channel, duration_millis);

        Ok(crate::sys::OT_RADIO_RSSI_INVALID as i8)
    }

    /// Transmit a radio frame.
    ///
    /// If the radio _does_ support `MacCapabilities::TX_ACK`:
    /// - The implementation of this method should automatically wait for an ACK frame to be received and return
    ///   the meta-data associated with the received ACK frame;
    /// - The implementation of this method should auto-ACK and accumulate any frames received while waiting for
    ///   the ACK frame and return them on subsequent `receive` calls. Note that this does mean that the radio should
    ///   support `MacCapabilities::RX_ACK` as well. Support for one but not the other is typically not very useful.
    ///
    /// Arguments:
    /// - `psdu`: The PSDU to transmit as part of the frame. A radio advertising
    ///   [`Capabilities::TRANSMIT_SEC`] finishes it in place (see [`TxInfo`]).
    /// - `tx`: What is already done to the frame, and what the radio did to it.
    /// - `channel`: The channel to transmit the frame on.
    /// - `cca_threshold`: The CCA threshold to use before transmitting the frame. If `None`, CCA is not performed.
    /// - `ack_psdu_buf`: The buffer to store the received ACK PSDU if the radio is capable of reporting received ACKs.
    ///
    /// Returns:
    /// - The meta-data associated with the received ACK frame if the radio is capable of reporting received ACKs
    ///   and an ACK was expected and received for the transmitted frame.
    async fn transmit(
        &mut self,
        psdu: &mut [u8],
        tx: &mut TxInfo,
        channel: u8,
        power: i8,
        cca_threshold: Option<i8>,
        ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Self::Error>;

    /// Retrieve an already received radio frame, or wait for one to arrive.
    ///
    /// A frame might already be received and waiting in the radio's internal RX queue when the radio implementation has MAC
    /// offloading capabilities (`MacCapabilities`) and therefore maintains an internal queue of received frames filled on IRQ.
    ///
    /// If the radio is sleeping, and the radio's internal RX queue (if any) is empty, the method will wait indefinitely.
    ///
    /// This method _must_ be cancellation-safe in that if the future returned by `receive` is dropped, the radio should _not_ drop
    /// already received frames. Dropping a frame which is still in the process of being received is allowed.
    ///
    /// Arguments:
    /// - `psdu_buf`: The buffer to store the received PSDU.
    ///
    /// Returns:
    /// - The meta-data associated with the received frame.
    async fn receive(&mut self, psdu_buf: &mut [u8]) -> Result<PsduMeta, Self::Error>;
}

impl<T> Radio for &mut T
where
    T: Radio,
{
    type Error = T::Error;

    async fn init(&mut self) -> Result<RadioCaps, Self::Error> {
        T::init(self).await
    }

    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error> {
        T::set_config(self, config).await
    }

    async fn set_src_match_config(&mut self, entries: &SrcMatchConfig) -> Result<(), Self::Error> {
        T::set_src_match_config(self, entries).await
    }

    async fn energy_scan(&mut self, channel: u8, duration_millis: u16) -> Result<i8, Self::Error> {
        T::energy_scan(self, channel, duration_millis).await
    }

    async fn set_receive(&mut self, channel: u8) -> Result<(), Self::Error> {
        T::set_receive(self, channel).await
    }

    async fn set_sleep(&mut self) -> Result<(), Self::Error> {
        T::set_sleep(self).await
    }

    async fn transmit(
        &mut self,
        psdu: &mut [u8],
        tx: &mut TxInfo,
        channel: u8,
        power: i8,
        cca_threshold: Option<i8>,
        ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Self::Error> {
        T::transmit(self, psdu, tx, channel, power, cca_threshold, ack_psdu_buf).await
    }

    async fn receive(&mut self, psdu_buf: &mut [u8]) -> Result<PsduMeta, Self::Error> {
        T::receive(self, psdu_buf).await
    }

    async fn receive_at(
        &mut self,
        channel: u8,
        start_us: u64,
        duration_us: u32,
    ) -> Result<(), Self::Error> {
        T::receive_at(self, channel, start_us, duration_us).await
    }

    async fn set_csl(&mut self, csl: &CslConfig) -> Result<(), Self::Error> {
        T::set_csl(self, csl).await
    }

    async fn set_mac_keys(&mut self, keys: Option<&MacKeys>) -> Result<(), Self::Error> {
        T::set_mac_keys(self, keys).await
    }

    async fn set_mac_frame_counter(
        &mut self,
        frame_counter: u32,
        if_larger: bool,
    ) -> Result<(), Self::Error> {
        T::set_mac_frame_counter(self, frame_counter, if_larger).await
    }
}
