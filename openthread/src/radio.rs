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

/// Carrier sense or Energy Detection (ED) mode.
#[derive(Debug, Default, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Cca {
    /// Carrier sense
    #[default]
    Carrier,
    /// Energy Detection / Energy Above Threshold
    Ed {
        /// Energy measurements above this value mean that the channel is assumed to be busy.
        /// Note the measurement range is 0..0xFF - where 0 means that the received power was
        /// less than 10 dB above the selected receiver sensitivity. This value is not given in dBm,
        /// but can be converted. See the nrf52840 Product Specification Section 6.20.12.4
        /// for details.
        ed_threshold: u8,
    },
    /// Carrier sense or Energy Detection
    CarrierOrEd { ed_threshold: u8 },
    /// Carrier sense and Energy Detection
    CarrierAndEd { ed_threshold: u8 },
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
        /// Radio supports frame security processing (encryption/decryption).
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
    /// Any capability *missing* here is emulated in software by wrapping the
    /// radio in [`MacRadio`]: `MacCapabilities::all()` means a fully
    /// offloaded MAC (no software emulation needed), `MacCapabilities::none()`
    /// a bare PHY-like radio that gets the complete soft-MAC.
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
        /// The radio's own ACK engine honors the source-address-match table
        /// ([`Radio::update_src_match`] reaches whatever decides the ACKs'
        /// Frame Pending bit - a hardware pending table, RCP firmware, etc).
        ///
        /// Only meaningful together with [`RX_ACK`](Self::RX_ACK): the table
        /// matters solely to whoever sends the ACKs. A radio doing its own
        /// RX ACKs *without* this capability answers every data poll FP = 1
        /// (protocol-safe over-promising) - and nothing above it can do
        /// better, since the ACKs are out of software's hands.
        const SRC_MATCH = 0x40;
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
    ///
    /// Reported to OpenThread via `otPlatRadioGetReceiveSensitivity`, which
    /// uses it as the noise floor for grading neighbor links.
    pub receive_sensitivity: i8,
}

impl RadioCaps {
    /// The OpenThread core's own default receive sensitivity (dBm), for
    /// drivers that do not (yet) report a hardware-specific figure.
    pub const DEFAULT_RECEIVE_SENSITIVITY: i8 = -110;
}

impl Default for RadioCaps {
    fn default() -> Self {
        Self {
            phy: Capabilities::default(),
            mac: MacCapabilities::default(),
            receive_sensitivity: Self::DEFAULT_RECEIVE_SENSITIVITY,
        }
    }
}

/// Capacity of each address family's table in [`SrcMatchEntries`]: sized
/// after OpenThread's default max-children count (10) with headroom. On
/// overflow the glue answers `OT_ERROR_NO_BUFS`, which OpenThread handles by
/// falling back to frame-pending-on-every-ack for the un-tracked children.
pub const SRC_MATCH_CAPACITY: usize = 16;

/// The source-address-match table (the `otPlatRadio*SrcMatch*` platform
/// surface): the set of sleepy children the stack currently has pending
/// indirect frames for.
///
/// The MAC-level contract it serves: the ACK answering a child's data poll
/// carries the Frame Pending bit iff data is queued for that child - that bit
/// is what tells the child to keep its receiver on for the delivery. With
/// matching `enabled` and the poll's source absent from the table, the ACK
/// answers FP = 0 and the child returns to sleep immediately; with matching
/// disabled, every poll is answered FP = 1 (the conservative default the C
/// contract prescribes).
///
/// Whoever sends the ACKs consults this table: [`MacRadio`] for the software
/// MAC, the hardware/co-processor tables for radios with native RX-ACK
/// offload (see [`Radio::update_src_match`]).
#[derive(Debug, Clone, Eq, PartialEq, Default)]
pub struct SrcMatchEntries {
    /// Whether source matching is active. When `false`, polls are answered
    /// FP = 1 regardless of the tables.
    pub enabled: bool,
    /// The short (RLOC16) entries.
    pub short_addrs: heapless::Vec<u16, SRC_MATCH_CAPACITY>,
    /// The extended (EUI-64) entries.
    pub ext_addrs: heapless::Vec<u64, SRC_MATCH_CAPACITY>,
}

impl SrcMatchEntries {
    /// The Frame Pending answer for an ack-requesting MAC command frame
    /// arriving from `src_short` / `src_ext`.
    pub fn ack_frame_pending(&self, src_short: u16, src_ext: u64) -> bool {
        !self.enabled || self.short_addrs.contains(&src_short) || self.ext_addrs.contains(&src_ext)
    }
}

/// Radio configuration.
#[derive(Debug, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Channel number
    pub channel: u8,
    /// Transmit power in dBm
    pub power: i8,
    /// Clear channel assessment (CCA) mode
    pub cca: Cca,
    /// TBD
    pub sfd: u8,
    /// Whether the radio is in receive mode.
    ///
    /// This is the radio's Receive-vs-Sleep state (`otPlatRadioReceive` /
    /// `otPlatRadioSleep`), and therefore also the driver's power-management
    /// hook: a driver that can power its receiver down should do so when this
    /// turns `false`, and bring it back up when it turns `true`.
    ///
    /// Frames arriving while it is `false` MUST be dropped, not buffered: the
    /// stack relies on a parked radio genuinely missing traffic (a sleepy
    /// child's parent buffers for it on exactly that assumption - see
    /// `docs/radio-contract.md`, C6). A driver whose RX path keeps filling
    /// autonomously while parked (hardware FIFOs, IRQ handlers, simulation
    /// event pumps) must therefore discard whatever accumulated, at the
    /// latest when this turns `true` again.
    ///
    /// Disregarded if the radio does not have any MAC offloading capabilities
    /// and therefore is not capable of receiving frames autonomously, and emulated by [`MacRadio`].
    pub receive: bool,
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
            channel: 11,
            // Run with max power by default
            // TODO: Figure out how to have this specified by the user
            power: 20,
            cca: Cca::Carrier,
            sfd: 0,
            receive: false,
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
}

/// The IEEE 802.15.4 PHY Radio trait.
///
/// While the trait models the PHY layer of the radio, it might implement some "MAC-offloading"
/// capabilities as well - namely - the ability to automatically send and receive ACK frames,
/// the ability to filter received frames by PAN ID, short address, and extended address and others.
///
/// If some of these capabilities are not available, this crate emulates them in
/// software (via the [`MacRadio`] wrapper).
///
/// # Contract
///
/// OpenThread drives the radio as a small state machine - Sleep, Receive,
/// Energy Scan, and the transmit sequence as an excursion out of Receive -
/// and holds the platform to semantics that are easy to violate from the
/// signatures alone (see `docs/radio-contract.md`).
///
/// An implementation MUST uphold:
/// 1. **`transmit` is the complete transmit sequence** per the declared
///    capabilities - CSMA/CCA, the transmission, and, for frames requesting
///    one, the ACK wait - resolved without the caller concurrently polling
///    `receive`. The matching ACK is consumed by `transmit` and reported in
///    its result; it must never surface via `receive`.
///
///    `transmit` is only allowed to be a pure "send this frame" without waiting
///    for an ACK when `MacCapabilities::TX_ACK` is NOT set. In that case, the
///    full semantics of "`transmit` is the complete transmit sequence" are
///    automatically polyfilled by this crate, via the [`MacRadio`] software
///    emulation.
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
///    in the first place. In case when `MacCapabilities::RX_ACK` is not set,
///    the full semantics of "Reception outlives the calls" are automatically
///    polyfilled by this crate, via the [`MacRadio`] software emulation.
///
/// 3. **Cancellation is a sanctioned abort**: the `transmit` future may be
///    dropped mid-sequence (OpenThread aborts an ACK wait this way). The
///    frame may already be on the air; the radio must simply return to
///    receiving.
///
/// 4. **A sleeping radio misses frames** ([`Radio::sleep`]): frames arriving
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
    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error>;

    /// Update the radio's source-address-match table (see
    /// [`SrcMatchEntries`] for the contract it serves).
    ///
    /// Called by the crate whenever the stack mutates the table, with the
    /// complete new table each time (never incrementally). Synchronous by
    /// design - it is pure bookkeeping; a radio that must push the table to
    /// distant hardware (e.g. an RCP's source-match properties) should stash
    /// the snapshot and flush it on its next async operation.
    ///
    /// The default does nothing, which is correct for a bare radio whose
    /// ACKs are emulated above it (`MacRadio` keeps and consults its own
    /// copy). A radio doing its own RX ACKs should apply the table to
    /// whatever decides its ACKs' Frame Pending bit; ignoring it means every
    /// data poll is answered FP = 1, which is protocol-safe but keeps sleepy
    /// children listening needlessly after each poll.
    async fn set_src_match(&mut self, entries: &SrcMatchEntries) -> Result<(), Self::Error> {
        let _ = entries;

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
    /// NOTE: OpenThread's energy scan requests are always routed here,
    /// regardless of whether the radio reports [`Capabilities::ENERGY_SCAN`]
    /// (see the initial-`radio_caps` discussion in `lib.rs`: OpenThread
    /// snapshots the radio capabilities before the actual `Radio` instance is
    /// known, and its software-sampling fallback needs a synchronous RSSI
    /// read, which is unimplementable on top of this async trait).
    async fn energy_scan(&mut self, duration_millis: u16) -> Result<i8, Self::Error> {
        let _ = duration_millis;

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
    /// - `psdu`: The PSDU to transmit as part of the frame.
    /// - `cca`: Whether to perform clear channel assessment (CCA) before transmitting the frame.
    /// - `ack_psdu_buf`: The buffer to store the received ACK PSDU if the radio is capable of reporting received ACKs.
    ///
    /// Returns:
    /// - The meta-data associated with the received ACK frame if the radio is capable of reporting received ACKs
    ///   and an ACK was expected and received for the transmitted frame.
    async fn transmit(
        &mut self,
        psdu: &[u8],
        cca: bool,
        ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Self::Error>;

    /// Retrieve an already received radio frame, or wait for one to arrive.
    ///
    /// A frame might already be received and waiting in the radio's internal queue when the radio implementation declares
    /// `MacCapabilities::TX_ACK` and the `transmit` method did get one or multiple RX frames while waiting for an ACK frame
    /// to be received. In that case, the implementation of this method might return such an already received frame instead
    /// of waiting for a new one to arrive.
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

    async fn set_src_match(&mut self, entries: &SrcMatchEntries) -> Result<(), Self::Error> {
        T::set_src_match(self, entries).await
    }

    async fn energy_scan(&mut self, duration_millis: u16) -> Result<i8, Self::Error> {
        T::energy_scan(self, duration_millis).await
    }

    async fn transmit(
        &mut self,
        psdu: &[u8],
        cca: bool,
        ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Self::Error> {
        T::transmit(self, psdu, cca, ack_psdu_buf).await
    }

    async fn receive(&mut self, psdu_buf: &mut [u8]) -> Result<PsduMeta, Self::Error> {
        T::receive(self, psdu_buf).await
    }
}
