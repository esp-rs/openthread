//! IEEE 802.15.4 PHY Radio trait and associated types for OpenThread.
//!
//! `openthread` operates the radio in terms of this trait, which is implemented by the actual radio driver.

use core::cell::Cell;
use core::fmt::Debug;
use core::future::Future;
use core::mem::MaybeUninit;
use core::pin::pin;

use embassy_futures::select::{select, Either};

use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex};
use embassy_sync::signal::Signal;
use embassy_sync::zerocopy_channel::{Channel, Receiver, Sender};

use embassy_time::Instant;

use mac::MacHeader;

use crate::fmt::{bitflags, Bytes};
use crate::sys::{
    otRadioCaps, OT_RADIO_CAPS_ACK_TIMEOUT, OT_RADIO_CAPS_CSMA_BACKOFF, OT_RADIO_CAPS_ENERGY_SCAN,
    OT_RADIO_CAPS_RECEIVE_TIMING, OT_RADIO_CAPS_SLEEP_TO_TX, OT_RADIO_CAPS_TRANSMIT_RETRIES,
    OT_RADIO_CAPS_TRANSMIT_SEC, OT_RADIO_CAPS_TRANSMIT_TIMING, OT_RADIO_FRAME_MAX_SIZE,
};

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
    /// Radio PHY capabilities.
    ///
    /// Reported to the OpenThread stack via `otPlatRadioGetCaps`.
    #[repr(transparent)]
    #[derive(Default)]
    #[cfg_attr(not(feature = "defmt"), derive(Debug, Copy, Clone, Eq, PartialEq, Hash))]
    pub struct Capabilities: u8 /*: otRadioCaps - defmt::bitflags! can't grok this*/ {
        /// Radio supports ACK timeout for transmitted frames.
        const ACK_TIMEOUT = OT_RADIO_CAPS_ACK_TIMEOUT as u8;
        /// Radio supports energy scan.
        const ENERGY_SCAN = OT_RADIO_CAPS_ENERGY_SCAN as u8;
        /// Radio supports automatic retransmission of unacknowledged frames.
        const TRANSMIT_RETRIES = OT_RADIO_CAPS_TRANSMIT_RETRIES as u8;
        /// Radio supports CSMA/CA backoff for frame transmission.
        const CSMA_BACKOFF = OT_RADIO_CAPS_CSMA_BACKOFF as u8;
        /// Radio supports direct transition from sleep to TX.
        const SLEEP_TO_TX = OT_RADIO_CAPS_SLEEP_TO_TX as u8;
        /// Radio supports frame security processing (encryption/decryption).
        const TRANSMIT_SEC = OT_RADIO_CAPS_TRANSMIT_SEC as u8;
        /// Radio supports precise TX timing.
        const TRANSMIT_TIMING = OT_RADIO_CAPS_TRANSMIT_TIMING as u8;
        /// Radio supports precise RX timing.
        const RECEIVE_TIMING = OT_RADIO_CAPS_RECEIVE_TIMING as u8;
    }
}

bitflags! {
    /// Radio MAC capabilities.
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
    /// Promiscuous mode (receive all frames regardless of address filtering)
    /// Disregarded if the radio is not capable of operating in promiscuous mode.
    pub promiscuous: bool,
    /// Receive during idle state
    /// Disregarded if the radio is not capable of receiving during idle state.
    pub rx_when_idle: bool,
    /// PAN ID filter
    /// Disregarded if the radio is not capable of filtering by PAN ID.
    pub pan_id: Option<u16>,
    /// Short address filter
    /// Disregarded if the radio is not capable of filtering by short address.
    pub short_addr: Option<u16>,
    /// Extended address filter
    /// Disregarded if the radio is not capable of filtering by extended address.
    pub ext_addr: Option<u64>,
}

impl Config {
    /// Create a new default configuration.
    pub const fn new() -> Self {
        Self {
            channel: 11,
            power: 8,
            cca: Cca::Carrier,
            sfd: 0,
            promiscuous: false,
            rx_when_idle: false,
            pan_id: None,
            short_addr: None,
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
/// capabilities as well - namely - the ability to automatically send and receive ACK frames
/// and the ability to filter received frames by PAN ID, short address, and extended address.
///
/// If some of these capabilities are not available, `OpenThread` will emulate those in software.
///
/// The trait is used to abstract the radio hardware and provide a common interface for the radio
/// operations. It needs to support the following operations:
/// - Get the radio capabilities (phy and mac ones)
/// - Set the radio configuration
/// - Transmit a radio frame and (optionally) wait for an ACK frame (if the transmitted frame requires an ACK)
/// - Receive a radio frame and (optionally) send an ACK frame (if the received frame requires an ACK)
/// - Optionally, drop received radio frames if they do not match the filter criteria (PAN ID, short address, extended address)
///
/// The trait is NOT required to support the following operations:
/// - Re-sending a TX frame if the ACK frame was not received; this is done by OpenThread
/// - Dropping a duplicate RX frame; this is done by OpenThread
/// - MAC layer security; this is done by OpenThread
pub trait Radio {
    /// The error type for radio operations.
    type Error: RadioError;

    /// Radio capabilities.
    const CAPS: Capabilities;

    /// Radio "MAC-offloading" capabilities.
    /// If some of these are missing, `OpenThread` will emulate them in software.
    const MAC_CAPS: MacCapabilities;

    /// Set the radio configuration.
    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error>;

    // TODO
    //fn sleep(&mut self);

    // TODO
    //fn rssi(&mut self) -> i8;

    // TODO
    //fn receive_sensitivity(&mut self) -> i8;

    // TODO
    //fn set_enabled(&mut self, enabled: bool) -> Result<(), Self::Error>;

    // TODO
    //fn energy_scan(&mut self, channel: u8, duration: u16) -> Result<(), Self::Error>;

    /// Transmit a radio frame.
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

    /// Receive a radio frame.
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

    const CAPS: Capabilities = T::CAPS;

    const MAC_CAPS: MacCapabilities = T::MAC_CAPS;

    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error> {
        T::set_config(self, config).await
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

/// An error type for the enhanced radio.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum MacRadioError<T> {
    /// Invalid TX frame
    TxInvalid,
    /// Invalid RX frame
    RxInvalid,
    /// Receiving failed due to sending an ACK frame failed
    TxAckFailed(T),
    /// Transmitting failed due to receiving an ACK frame failed
    RxAckFailed(T),
    /// Receiving failed due to timeout when preparing an ACK frame
    TxAckTimeout,
    /// Transmitting failed due to no ACK received
    RxAckTimeout,
    /// Transmitting failed due to invalid ACK received
    RxAckInvalid,
    /// Error coming from the wrapped radio
    Io(T),
}

impl<T> RadioError for MacRadioError<T>
where
    T: RadioError,
{
    fn kind(&self) -> RadioErrorKind {
        match self {
            Self::TxInvalid => RadioErrorKind::TxInvalid,
            Self::RxInvalid => RadioErrorKind::RxInvalid,
            Self::RxAckInvalid => RadioErrorKind::RxAckInvalid,
            Self::TxAckFailed(_) => RadioErrorKind::TxAckFailed,
            Self::RxAckFailed(_) => RadioErrorKind::RxAckFailed,
            Self::TxAckTimeout => RadioErrorKind::TxAckTimeout,
            Self::RxAckTimeout => RadioErrorKind::RxAckTimeout,
            Self::Io(e) => e.kind(),
        }
    }
}

#[cfg(feature = "defmt")]
impl<T> defmt::Format for MacRadioError<T>
where
    T: RadioError,
{
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(fmt, "{}", self.kind())
    }
}

/// An enhanced (MAC) radio that can optionally send and receive ACKs for transmitted frames
/// as well as optionally do address filtering.
pub struct MacRadio<R, T> {
    /// The wrapped radio.
    radio: R,
    /// The timer implementation to use.
    /// Necessary to properly time:
    /// - How long to wait for an ACK for a transmitted frame
    /// - How long to wait before sending an ACK for a received frame
    ///
    /// The above is only relevant if the `MacRadio` is instructed to
    /// receive TX ACKs in software and/or to send RX ACKs in software.
    ///
    /// Should be with a high precision of ideally < 10us.
    timer: T,
    /// Whether the radio is in promiscuous mode.
    promiscuous: bool,
    /// A buffer for the MAC header of the received or transmitted frame.
    /// (For filtering and ACKs)
    mac_header: MacHeader,
    /// The buffer for the ACK PSDU, if the `MacRadio` is instructed
    /// to send or receive ACKs in software.
    // TODO: Inject from outside
    ack_psdu_buf: [u8; OT_RADIO_FRAME_MAX_SIZE as _],
    /// The PAN ID to filter by, if the filter policy allows it.
    pan_id: u16,
    /// The short address to filter by, if the filter policy allows it.
    short_addr: u16,
    /// The extended address to filter by, if the filter policy allows it.
    ext_addr: u64,
}

impl<R, T> MacRadio<R, T>
where
    R: Radio,
    T: MacRadioTimer,
{
    /// The waiting timeout for a TX ACK to be received.
    /// See https://github.com/openthread/openthread/blob/9398342b49a03cd140fae910b81cddf9084293a0/src/core/mac/sub_mac.hpp#L528
    /// Currently much longer than what specified there
    const TX_ACK_WAIT_US: u64 = 500 * 1000; //16 * 1000 * 10;
    /// The waiting timeout for an RX ACK to be sent.
    // TODO: Should be 190us, but we need to be more precise
    // and not use `embassy-time` with the NRF...
    const RX_ACK_SEND_US: u64 = 10;

    /// Create a new enhanced MAC radio.
    ///
    /// Arguments:
    /// - `radio`: The radio to wrap.
    /// - `delay`: The delay implementation to use. Should be with a high precision of ideally < 10us
    /// - `ack_policy`: The ACK policy to use.
    /// - `filter_policy`: The filter policy to use.
    pub fn new(radio: R, timer: T) -> Self {
        Self {
            radio,
            timer,
            mac_header: MacHeader::new(),
            ack_psdu_buf: [0; OT_RADIO_FRAME_MAX_SIZE as _],
            promiscuous: false,
            pan_id: MacHeader::BROADCAST_PAN_ID,
            short_addr: MacHeader::BROADCAST_SHORT_ADDR,
            ext_addr: MacHeader::BROADCAST_EXT_ADDR,
        }
    }
}

impl<R, T> Radio for MacRadio<R, T>
where
    R: Radio,
    T: MacRadioTimer,
{
    type Error = MacRadioError<R::Error>;

    const CAPS: Capabilities = R::CAPS;

    const MAC_CAPS: MacCapabilities = R::MAC_CAPS;

    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error> {
        self.radio
            .set_config(config)
            .await
            .map_err(Self::Error::Io)?;

        self.promiscuous = config.promiscuous;
        self.pan_id = config.pan_id.unwrap_or(MacHeader::BROADCAST_PAN_ID);
        self.short_addr = config.short_addr.unwrap_or(MacHeader::BROADCAST_SHORT_ADDR);
        self.ext_addr = config.ext_addr.unwrap_or(MacHeader::BROADCAST_EXT_ADDR);

        Ok(())
    }

    async fn transmit(
        &mut self,
        psdu: &[u8],
        cca: bool,
        ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Self::Error> {
        trace!("MacRadio, about to transmit");

        if R::MAC_CAPS.contains(MacCapabilities::TX_ACK) {
            let result = self
                .radio
                .transmit(psdu, cca, ack_psdu_buf)
                .await
                .map_err(Self::Error::Io);

            trace!("MacRadio, transmitted");

            result
        } else {
            self.radio
                .transmit(psdu, cca, None)
                .await
                .map_err(Self::Error::Io)?;

            let sent_at = self.timer.now();

            self.mac_header.load(psdu).ok_or(MacRadioError::TxInvalid)?;

            if self.mac_header.needs_ack() {
                let psdu_seq = self.mac_header.seq;

                trace!("MacRadio, about to receive transmit ACK");

                let result = {
                    let mut ack = pin!(self.radio.receive(&mut self.ack_psdu_buf));
                    let mut timeout = pin!(self.timer.wait(sent_at + Self::TX_ACK_WAIT_US));

                    select(&mut ack, &mut timeout).await
                };

                let ack_meta = match result {
                    Either::First(result) => result.map_err(Self::Error::RxAckFailed)?,
                    Either::Second(_) => {
                        trace!("MacRadio, transmit ACK timeout");

                        Err(Self::Error::RxAckTimeout)?
                    }
                };

                let ack_psdu = &self.ack_psdu_buf[..ack_meta.len];
                self.mac_header
                    .load(ack_psdu)
                    .ok_or(MacRadioError::RxAckInvalid)?;

                if !self.mac_header.ack_for(psdu_seq) {
                    Err(MacRadioError::RxAckInvalid)?;
                }

                if let Some(ack_psdu_buf) = ack_psdu_buf {
                    ack_psdu_buf[..ack_psdu.len()].copy_from_slice(ack_psdu);
                }

                trace!("MacRadio, transmitted with ACK");

                //Ok(Some(ack_meta)) TODO
                Ok(None)
            } else {
                trace!("MacRadio, transmitted without ACK");

                Ok(None)
            }
        }
    }

    async fn receive(&mut self, psdu_buf: &mut [u8]) -> Result<PsduMeta, Self::Error> {
        loop {
            trace!("MacRadio, about to receive");

            let psdu_meta = self
                .radio
                .receive(psdu_buf)
                .await
                .map_err(Self::Error::Io)?;

            let ack_at = self.timer.now() + Self::RX_ACK_SEND_US;

            let psdu = &psdu_buf[..psdu_meta.len];

            trace!("MacRadio, received: {}, meta: {:?}", Bytes(psdu), psdu_meta);

            if R::MAC_CAPS != MacCapabilities::all() {
                if self.mac_header.load(psdu).is_none() {
                    trace!(
                        "MacRadio, received frame with invalid MAC header, dropping: {}",
                        Bytes(psdu)
                    );
                    continue;
                }

                if !R::MAC_CAPS.contains(MacCapabilities::PROMISCUOUS) && !self.promiscuous {
                    if !R::MAC_CAPS.contains(MacCapabilities::FILTER_PAN_ID)
                        && self.mac_header.pan_id != MacHeader::BROADCAST_PAN_ID
                        && self.mac_header.pan_id != self.pan_id
                    {
                        trace!(
                            "MacRadio, filtering out frame: {}, PAN ID does not match",
                            Bytes(psdu)
                        );
                        continue;
                    }

                    if !R::MAC_CAPS.contains(MacCapabilities::FILTER_SHORT_ADDR)
                        && self.mac_header.dst_short_addr != MacHeader::BROADCAST_SHORT_ADDR
                        && self.mac_header.dst_short_addr != self.short_addr
                    {
                        trace!(
                            "MacRadio, filtering out frame: {}, short address does not match",
                            Bytes(psdu)
                        );
                        continue;
                    }

                    if !R::MAC_CAPS.contains(MacCapabilities::FILTER_EXT_ADDR)
                        && self.mac_header.dst_ext_addr != MacHeader::BROADCAST_EXT_ADDR
                        && self.mac_header.dst_ext_addr != self.ext_addr
                    {
                        trace!(
                            "MacRadio, filtering out frame: {}, extended address does not match",
                            Bytes(psdu)
                        );
                        continue;
                    }

                    if !R::MAC_CAPS.contains(MacCapabilities::RX_ACK) && self.mac_header.needs_ack()
                    {
                        let ack_len = self.mac_header.prep_ack(&mut self.ack_psdu_buf);
                        let ack_psdu = &mut self.ack_psdu_buf[..ack_len];

                        trace!("MacRadio, about to transmit ACK: {}", Bytes(ack_psdu));

                        if self.timer.now() < ack_at {
                            self.timer.wait(ack_at).await;
                        }

                        self.radio
                            .transmit(ack_psdu, false /*TODO: or true?*/, None)
                            .await
                            .map_err(Self::Error::TxAckFailed)?;
                    }
                }
            }

            trace!("MacRadio, received frame: {}", Bytes(psdu));

            break Ok(psdu_meta);
        }
    }
}

/// A high-res timer trait that is necessary for the `MacRadio` to send ACKs
/// at the right time.
///
/// The timer should have a microsecond, or a few microseconds' resolution.
pub trait MacRadioTimer {
    /// Returns the current time - in microseconds - from some predefined period in time
    /// The returned time should be monotonic.
    fn now(&mut self) -> u64;

    /// Waits until the current time becomes equal or greater than the given time.
    ///
    /// If the current time is already equal or greater than the given time,
    /// the function should return immediately, without waiting.
    ///
    /// Arguments:
    /// - `time`: The time - in microseconds - to wait until.
    async fn wait(&mut self, at: u64);
}

impl<T> MacRadioTimer for &mut T
where
    T: MacRadioTimer,
{
    fn now(&mut self) -> u64 {
        T::now(self)
    }

    async fn wait(&mut self, at: u64) {
        T::wait(self, at).await
    }
}

/// An implementation of `MacRadioTimer` that uses the `embassy_time` crate.
///
/// Note that this implementation might NOT be appropriate when the concrete
/// `embassy-time` implementation is not having a high-enough resolution.
pub struct EmbassyTimeTimer;

impl MacRadioTimer for EmbassyTimeTimer {
    fn now(&mut self) -> u64 {
        Instant::now().as_micros()
    }

    async fn wait(&mut self, at: u64) {
        embassy_time::Timer::at(Instant::from_micros(at)).await;
    }
}

/// The resources for the radio proxy.
pub struct ProxyRadioResources {
    request_buf: MaybeUninit<[ProxyRadioRequest; 1]>,
    response_buf: MaybeUninit<[ProxyRadioResponse; 1]>,
    state: MaybeUninit<ProxyRadioState<'static>>,
}

impl ProxyRadioResources {
    /// Create a new set of radio proxy resources.
    pub const fn new() -> Self {
        Self {
            request_buf: MaybeUninit::uninit(),
            response_buf: MaybeUninit::uninit(),
            state: MaybeUninit::uninit(),
        }
    }
}

impl Default for ProxyRadioResources {
    fn default() -> Self {
        Self::new()
    }
}

/// A type that allows to offload the execution (TX/RX) of the actual PHY `Radio` impl
/// to a separate - possibly higher-priority - executor.
///
/// Running the PHY radio in a separate higher priority executor is particularly desirable in the cases where it
/// cannot do MAC-offloading (ACKs and filtering) in hardware, and hence the `MacRadio` wrapper is used to handle
/// these tasks in software. Due to timing constraints with ACKs and filtering, this task should have a higher
/// priority than all other `OpenThread`-related tasks.
///
/// This is achieved by splitting the radio into two types:
/// - `ProxyRadio`, which is a radio proxy that implements the `Radio` trait and is to be used by the main execution
///   by passing it to `OpenThread::run`
/// - `PhyRadioRunner`, which is `Send` and therefore can be sent to a separate executor - to run the radio.
///   Invoke `PhyRadioRunner::run(<the-phy-radio>, <delay-provider>).await` in that separate executor.
pub struct ProxyRadio<'a, const CAPS: otRadioCaps> {
    /// The request channel to the PHY radio
    request: Sender<'a, CriticalSectionRawMutex, ProxyRadioRequest>,
    /// The response channel from the PHY radio
    response: Receiver<'a, CriticalSectionRawMutex, ProxyRadioResponse>,
    /// Whether the current response was cancelled
    /// If set to `true`, this means we have to drop the pending response from the driver
    cancelled: Cell<bool>,
    /// The signal to indicate a new request to the PHY radio, so that
    /// the PHY radio can cancel the current request (if any) and start processing the new one
    cancel: &'a Signal<CriticalSectionRawMutex, ()>,
    /// The current radio configuration
    config: Config,
}

impl<'a, const CAPS: otRadioCaps> ProxyRadio<'a, CAPS> {
    const INIT_REQUEST: [ProxyRadioRequest; 1] = [ProxyRadioRequest::new()];
    const INIT_RESPONSE: [ProxyRadioResponse; 1] = [ProxyRadioResponse::new()];

    /// Create a new `ProxyRadio` and its `PhyRadioRunner` instances.
    ///
    /// Arguments:
    /// - `resources`: The radio proxy resources
    pub fn new(resources: &'a mut ProxyRadioResources) -> (Self, PhyRadioRunner<'a>) {
        resources.request_buf.write(Self::INIT_REQUEST);
        resources.response_buf.write(Self::INIT_RESPONSE);

        let request_buf = unsafe { resources.request_buf.assume_init_mut() };
        let response_buf = unsafe { resources.response_buf.assume_init_mut() };

        resources.state.write(ProxyRadioState::new(
            unsafe {
                core::mem::transmute::<
                    &mut [ProxyRadioRequest; 1],
                    &'static mut [ProxyRadioRequest; 1],
                >(request_buf)
            },
            unsafe {
                core::mem::transmute::<
                    &mut [ProxyRadioResponse; 1],
                    &'static mut [ProxyRadioResponse; 1],
                >(response_buf)
            },
        ));

        let state = unsafe { resources.state.assume_init_mut() };

        state.split::<CAPS>()
    }

    /// Clear any cancelled response from the driver
    async fn process_cancelled(&mut self) {
        if self.cancelled.get() {
            self.response.receive().await;

            self.cancelled.set(false);
            self.response.receive_done();

            trace!("ProxyRadio, cancelled response cleared");
        }
    }
}

impl<const CAPS: otRadioCaps> Radio for ProxyRadio<'_, CAPS> {
    type Error = RadioErrorKind;

    const CAPS: Capabilities = Capabilities::from_bits_truncate(CAPS);

    // ... because the actual PHY radio on the other side
    // of the pipe will be wrapped with `MacRadio` if it cannot do ACKs and filtering in hardware
    const MAC_CAPS: MacCapabilities = MacCapabilities::all();

    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error> {
        // There is no separate command for updating the configuration
        // The updated configuration is always valid for the next request
        self.config = config.clone();
        Ok(())
    }

    async fn transmit(
        &mut self,
        psdu: &[u8],
        cca: bool,
        ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Self::Error> {
        trace!("ProxyRadio, about to transmit: {}", Bytes(psdu));

        self.process_cancelled().await;

        {
            let req = self.request.send().await;

            req.tx = true;
            req.config = self.config.clone();
            req.cca = cca;
            req.psdu.clear();
            unwrap!(req.psdu.extend_from_slice(psdu));

            trace!("ProxyRadio, transmit request sent: {:?}", req);

            self.request.send_done();
        }

        self.cancelled.set(true);
        let _guard = scopeguard::guard((), |_| {
            if self.cancelled.get() {
                trace!("ProxyRadio, transmit request cancelled");
                self.cancel.signal(())
            }
        });

        trace!("ProxyRadio, waiting for transmit response");

        let resp = self.response.receive().await;

        trace!("ProxyRadio, transmit response received: {:?}", resp);

        let psdu_meta = (ack_psdu_buf.is_some() && !resp.psdu.is_empty()).then_some(PsduMeta {
            len: resp.psdu.len(),
            channel: resp.psdu_channel,
            rssi: resp.psdu_rssi,
        });

        if let Some(ack_psdu_buf) = ack_psdu_buf {
            if psdu_meta.is_some() {
                ack_psdu_buf[..resp.psdu.len()].copy_from_slice(&resp.psdu);
            } else {
                ack_psdu_buf.fill(0);
            }
        }

        let result = resp.result.map(|_| psdu_meta);

        self.cancelled.set(false);
        self.response.receive_done();

        trace!("ProxyRadio, transmit response done");

        result
    }

    async fn receive(&mut self, psdu_buf: &mut [u8]) -> Result<PsduMeta, Self::Error> {
        trace!("ProxyRadio, about to receive");

        self.process_cancelled().await;

        {
            let req = self.request.send().await;

            req.tx = false;
            req.config = self.config.clone();
            req.psdu.clear();

            trace!("ProxyRadio, receive request sent: {:?}", req);

            self.request.send_done();
        }

        self.cancelled.set(true);
        let _guard = scopeguard::guard((), |_| {
            if self.cancelled.get() {
                trace!("ProxyRadio, receive request cancelled");
                self.cancel.signal(());
            }
        });

        trace!("ProxyRadio, waiting for receive response");

        let resp = self.response.receive().await;

        trace!("ProxyRadio, receive response received: {:?}", resp);

        match resp.result {
            Ok(()) => {
                let len = resp.psdu.len();
                psdu_buf[..len].copy_from_slice(&resp.psdu);

                let psdu_meta = PsduMeta {
                    len,
                    channel: resp.psdu_channel,
                    rssi: resp.psdu_rssi,
                };

                self.cancelled.set(false);
                self.response.receive_done();

                Ok(psdu_meta)
            }
            Err(e) => {
                self.cancelled.set(false);
                self.response.receive_done();
                Err(e)
            }
        }
    }
}

/// A type modeling the running of the PHY radio - the other side of the `ProxyRadio` pipe.
pub struct PhyRadioRunner<'a> {
    /// The request channel from the proxy radio
    request: Receiver<'a, CriticalSectionRawMutex, ProxyRadioRequest>,
    /// The response channel to the proxy radio
    response: Sender<'a, CriticalSectionRawMutex, ProxyRadioResponse>,
    /// The signal to indicate a new request from the proxy radio
    /// which means we have to cancel processing the current request (if any)
    cancel: &'a Signal<CriticalSectionRawMutex, ()>,
}

impl PhyRadioRunner<'_> {
    /// Run the PHY radio.
    ///
    /// Arguments:
    /// - `radio`: The PHY radio to run.
    /// - `delay`: The delay implementation to use.
    pub async fn run<R, T>(&mut self, radio: R, delay: T) -> !
    where
        R: Radio,
        T: MacRadioTimer,
    {
        let mut radio = MacRadio::new(radio, delay);

        debug!("PhyRadioRunner, running");

        loop {
            {
                let request = self.request.receive().await;

                if Self::process(&mut radio, request, &mut self.response, self.cancel)
                    .await
                    .is_some()
                {
                    trace!("PhyRadioRunner, processing done");
                } else {
                    // Processing was cancelled by a new request, need to send an "interrupted" response

                    let response = self.response.send().await;

                    response.psdu.clear();
                    response.result = Err(RadioErrorKind::Other);
                    response.psdu_channel = 0;
                    response.psdu_rssi = None;

                    self.response.send_done();

                    trace!("PhyRadioRunner, processing cancelled");
                }

                self.request.receive_done();
            }
        }
    }

    // Process a single request (TX or RX) by first updating the driver configuration
    // (driver should skip that if the new configuration is the same as the current one),
    // and then transmitting or receiving the frame.
    //
    // Updating the configuration, as well as the TX/RX operation might be cancelled at
    // any moment, if a new request arrives.
    async fn process<T>(
        mut radio: T,
        request: &mut ProxyRadioRequest,
        response_sender: &mut Sender<'_, impl RawMutex, ProxyRadioResponse>,
        cancel: &Signal<impl RawMutex, ()>,
    ) -> Option<()>
    where
        T: Radio,
    {
        let response = Self::with_cancel(response_sender.send(), cancel).await?;

        response.psdu.clear();
        response.psdu_channel = 0;
        response.psdu_rssi = None;

        trace!("PhyRadioRunner, processing request: {:?}", request);

        // Always first set the configuration relevant for the current TX/RX request
        // The PHY driver should have intelligence to skip the configuration update if the new
        // configuration is the same as the current one
        let result = Self::with_cancel(radio.set_config(&request.config), cancel)
            .await?
            .map_err(|e| e.kind());

        trace!("PhyRadioRunner, configuration set: {:?}", result);

        let result = if result.is_err() {
            // Setting driver configuration resulted in an error, so skip the rest of the processing
            result
        } else {
            unwrap!(response.psdu.resize_default(response.psdu.capacity()));

            let result = if request.tx {
                Self::with_cancel(
                    radio.transmit(&request.psdu, request.cca, Some(&mut response.psdu)),
                    cancel,
                )
                .await?
                .map_err(|e| e.kind())
            } else {
                Self::with_cancel(radio.receive(&mut response.psdu), cancel)
                    .await?
                    .map_err(|e| e.kind())
                    .map(Some)
            };

            if let Ok(Some(psdu_meta)) = &result {
                response.psdu.truncate(psdu_meta.len);
                response.psdu_channel = psdu_meta.channel;
                response.psdu_rssi = psdu_meta.rssi;
            } else {
                // No frame returned, so clear the response fields
                response.psdu.clear();
            }

            result.map(|_| ())
        };

        response.result = result;

        trace!("PhyRadioRunner, processed response: {:?}", response);

        response_sender.send_done();

        Some(())
    }

    async fn with_cancel<F>(fut: F, cancel: &Signal<impl RawMutex, ()>) -> Option<F::Output>
    where
        F: Future,
    {
        match select(fut, cancel.wait()).await {
            Either::First(result) => Some(result),
            Either::Second(_) => None,
        }
    }
}

// Should be safe because while not (yet) marked formally as such, zerocopy-channel's
// `Receiver` and `Sender` are `Send`, as long as the critical section is `Send` + `Sync`
// (which is the case as we use `CriticalSectionRawMutex`), and the `ProxyRadioRequest` and
// `ProxyRadioResponse` are `Send` (which is the case).
//
// The signals are obviously `Send` + `Sync`.
unsafe impl Send for PhyRadioRunner<'_> {}

const PSDU_LEN: usize = OT_RADIO_FRAME_MAX_SIZE as _;

/// The state of the proxy radio
///
/// This state is borrowed and shared between
/// the two ends of the pipe: the proxy radio, and the PHY radio runner.
struct ProxyRadioState<'a> {
    /// The request channel to the PHY radio
    request: Channel<'a, CriticalSectionRawMutex, ProxyRadioRequest>,
    /// The response channel from the PHY radio
    response: Channel<'a, CriticalSectionRawMutex, ProxyRadioResponse>,
    /// The signal to indicate a new request to the PHY radio
    cancel: Signal<CriticalSectionRawMutex, ()>,
}

impl<'a> ProxyRadioState<'a> {
    /// Create a new proxy radio state.
    ///
    /// Arguments:
    /// - `request_buf`: The request buffer
    /// - `response_buf`: The response buffer
    fn new(
        request_buf: &'a mut [ProxyRadioRequest; 1],
        response_buf: &'a mut [ProxyRadioResponse; 1],
    ) -> Self {
        Self {
            request: Channel::new(request_buf),
            response: Channel::new(response_buf),
            cancel: Signal::new(),
        }
    }

    /// Split the state into the proxy radio and the PHY radio runner.
    fn split<const CAPS: otRadioCaps>(&mut self) -> (ProxyRadio<'_, CAPS>, PhyRadioRunner<'_>) {
        let (request_sender, request_receiver) = self.request.split();
        let (response_sender, response_receiver) = self.response.split();

        (
            ProxyRadio {
                request: request_sender,
                response: response_receiver,
                cancelled: Cell::new(false),
                cancel: &self.cancel,
                config: Config::new(),
            },
            PhyRadioRunner {
                request: request_receiver,
                response: response_sender,
                cancel: &self.cancel,
            },
        )
    }
}

/// A proxy radio request.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct ProxyRadioRequest {
    /// Transmit or receive
    tx: bool,
    /// Whether to perform clear channel assessment (CCA) before transmitting the frame
    /// (only relevant for TX requests, should be ignored for RX requests)
    cca: bool,
    /// The radio configuration for the TX/RX operation
    config: Config,
    /// The PSDU to transmit for the TX operation
    psdu: heapless::Vec<u8, PSDU_LEN>,
}

impl ProxyRadioRequest {
    /// Create a new empty proxy radio request.
    const fn new() -> Self {
        Self {
            tx: false,
            cca: false,
            config: Config::new(),
            psdu: heapless::Vec::new(),
        }
    }
}

/// A proxy radio response.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct ProxyRadioResponse {
    /// The result of the TX/RX operation
    result: Result<(), RadioErrorKind>,
    /// The received PSDU, if the operation was successful:
    /// - For TX: the received ACK PSDU (might be empty)
    /// - For RX: the received frame PSDU
    psdu: heapless::Vec<u8, PSDU_LEN>,
    /// The channel on which the frame was received:
    /// - For TX: the channel on which the ACK frame was received
    /// - For RX: the channel on which the regular frame was received
    psdu_channel: u8,
    /// The RSSI of the received frame, if the radio supports appending it at the end of the frame:
    /// - For TX: the RSSI of the received ACK frame
    /// - For RX: the RSSI of the received frame
    psdu_rssi: Option<i8>,
}

impl ProxyRadioResponse {
    /// Create a new empty proxy radio response.
    const fn new() -> Self {
        Self {
            result: Ok(()),
            psdu: heapless::Vec::new(),
            psdu_channel: 0,
            psdu_rssi: None,
        }
    }
}

/// A minimal set of utilities for parsing the IEEE 802.15.4 MAC header
/// for the purposes of MAC filtering and RX/TX ACK processing.
mod mac {
    /// A parsed IEEE 802.15.4 MAC header.
    pub struct MacHeader {
        /// Frame Control Field (FCF)
        pub fcf: u16,
        /// Sequence number
        pub seq: u8,
        /// PAN ID. 0xffff if the Frame does not contain a PAN ID
        /// or if the PAN ID is the broadcast PAN ID
        pub pan_id: u16,
        /// Destination short address
        /// 0xffff if the Frame does not contain a short address
        /// or if the short address is the broadcast short address
        pub dst_short_addr: u16,
        /// Destination extended address
        /// 0xffffffffffffffff if the Frame does not contain an extended address
        /// or if the extended address is the broadcast extended address
        pub dst_ext_addr: u64,
    }

    impl MacHeader {
        /// The length of an Imm-ACK PSDU.
        pub const ACK_PSDU_LEN: usize = Self::FCF_LEN + Self::SEQ_LEN + Self::CRC_LEN;

        /// The broadcast PAN ID.
        pub const BROADCAST_PAN_ID: u16 = u16::MAX;
        /// The broadcast short address.
        pub const BROADCAST_SHORT_ADDR: u16 = u16::MAX;
        /// The broadcast extended address.
        pub const BROADCAST_EXT_ADDR: u64 = u64::MAX;

        const FCF_LEN: usize = 2;
        const SEQ_LEN: usize = 1;
        const CRC_LEN: usize = 2;

        const FCF_OFFSET: usize = 0;
        const SEQ_OFFSET: usize = Self::FCF_LEN;
        const ADDRS_OFFSET: usize = Self::SEQ_OFFSET + Self::SEQ_LEN;

        const FCF_FRAME_TYPE_MASK: u16 = 0x07;
        const FCF_FRAME_TYPE_ACK: u16 = 0x02;
        #[allow(unused)]
        const FCF_SECURITY_BIT: u16 = 1 << 3;
        #[allow(unused)]
        const FCF_PENDING_BIT: u16 = 1 << 4;
        const FCF_ACK_REQ_BIT: u16 = 1 << 5;
        #[allow(unused)]
        const FCF_PAN_ID_COMPRESSION_MASK: u16 = 1 << 6;
        const FCF_FRAME_DST_ADDR_MODE_SHIFT: u16 = 10;
        const FCF_FRAME_DST_ADDR_MODE_MASK: u16 = 0x03 << Self::FCF_FRAME_DST_ADDR_MODE_SHIFT;
        const FCF_FRAME_VERSION_SHIFT: u16 = 12;
        const FCF_FRAME_VERSION_MASK: u16 = 0x03 << Self::FCF_FRAME_VERSION_SHIFT;
        #[allow(unused)]
        const FCF_FRAME_SRC_ADDR_MODE_SHIFT: u16 = 14;
        #[allow(unused)]
        const FCF_FRAME_SRC_ADDR_MODE_MASK: u16 = 0x03 << Self::FCF_FRAME_DST_ADDR_MODE_SHIFT;

        /// Create a new empty MAC header.
        pub const fn new() -> Self {
            Self {
                fcf: 0,
                seq: 0,
                pan_id: 0,
                dst_short_addr: 0,
                dst_ext_addr: 0,
            }
        }

        /// Load the MAC header from a PSDU.
        /// Returns `Some(())` if the MAC header was successfully loaded.
        ///
        /// This method will fail if the frame version or type is unknown (reserved)
        /// or if the PSDU is too short.
        #[inline(always)]
        pub fn load(&mut self, psdu: &[u8]) -> Option<()> {
            Self::ensure_len(psdu, Self::ADDRS_OFFSET + Self::CRC_LEN)?;

            self.fcf =
                u16::from_le_bytes(unwrap!(psdu[Self::FCF_OFFSET..Self::SEQ_OFFSET].try_into()));
            self.seq = psdu[Self::SEQ_OFFSET];

            let _frame_type = FrameType::get(self.fcf)?;
            let _frame_version = FrameVersion::get(self.fcf)?;
            let dst_addr_mode = FrameAddrMode::get_dst(self.fcf)?;

            match dst_addr_mode {
                FrameAddrMode::NotPresent => {
                    self.pan_id = Self::BROADCAST_PAN_ID;
                    self.dst_short_addr = Self::BROADCAST_SHORT_ADDR;
                    self.dst_ext_addr = Self::BROADCAST_EXT_ADDR;
                }
                FrameAddrMode::Short => {
                    Self::ensure_len(psdu, Self::ADDRS_OFFSET + 2 + 2 + Self::CRC_LEN)?;

                    self.pan_id = u16::from_le_bytes(unwrap!(psdu[3..5].try_into()));
                    self.dst_short_addr = u16::from_le_bytes(unwrap!(psdu[5..7].try_into()));
                    self.dst_ext_addr = Self::BROADCAST_EXT_ADDR;
                }
                FrameAddrMode::Extended => {
                    Self::ensure_len(psdu, Self::ADDRS_OFFSET + 2 + 8 + Self::CRC_LEN)?;

                    self.pan_id = u16::from_le_bytes(unwrap!(psdu[3..5].try_into()));
                    // See platform.rs, `otPlatRadioSetExtendedAddress` impl
                    self.dst_ext_addr = u64::from_be_bytes(unwrap!(psdu[5..13].try_into()));
                    self.dst_short_addr = Self::BROADCAST_SHORT_ADDR;
                }
            }

            Some(())
        }

        /// Return `true` if the frame needs an ACK.
        #[inline(always)]
        pub fn needs_ack(&self) -> bool {
            (self.fcf & Self::FCF_ACK_REQ_BIT) != 0
        }

        /// Prepare an ACK PSDU.
        /// Assumes that the parsed frame header indicates that ACK is necessary (`self.needs_ack` returns `true`)
        #[inline(always)]
        pub fn prep_ack(&self, ack_buf: &mut [u8]) -> usize {
            assert!(ack_buf.len() >= Self::ACK_PSDU_LEN);

            let ack_fcf = Self::FCF_FRAME_TYPE_ACK
                | (self.fcf & Self::FCF_FRAME_VERSION_MASK)
                //| (src_fcf & Self::FCF_PENDING_MASK)
                ;

            ack_buf[0] = ack_fcf.to_le_bytes()[0];
            ack_buf[1] = ack_fcf.to_le_bytes()[1];
            ack_buf[2] = self.seq;
            ack_buf[3] = 0; // CRC, will be filled-in by the PHY driver
            ack_buf[4] = 0; // CRC, will be filled-in by the PHY driver

            Self::ACK_PSDU_LEN
        }

        /// Return `true` if the frame is an ACK frame and is an ACK for the given source sequence number.
        #[inline(always)]
        pub fn ack_for(&self, src_seq: u8) -> bool {
            matches!(unwrap!(FrameType::get(self.fcf)), FrameType::Ack) && src_seq == self.seq
        }

        #[inline(always)]
        fn ensure_len(psdu: &[u8], len: usize) -> Option<()> {
            (psdu.len() >= len).then_some(())
        }
    }

    /// The supported IEEE 802.15.4 frame versions
    #[derive(Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    enum FrameVersion {
        IEEE802154_2003,
        IEEE802154_2006,
    }

    impl FrameVersion {
        /// Get the frame version from the FCF.
        ///
        /// If the version is not supported, returns `None`.
        #[inline(always)]
        fn get(fcf: u16) -> Option<Self> {
            match (fcf & MacHeader::FCF_FRAME_VERSION_MASK) >> MacHeader::FCF_FRAME_VERSION_SHIFT {
                0 => Some(Self::IEEE802154_2003),
                1 => Some(Self::IEEE802154_2006),
                _ => None,
            }
        }
    }

    /// The supported IEEE 802.15.4 frame types
    #[derive(Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    enum FrameType {
        Beacon,
        Data,
        Ack,
        Command,
    }

    impl FrameType {
        /// Get the frame type from the FCF.
        ///
        /// If the type is not supported, returns `None`.
        #[inline(always)]
        fn get(fcf: u16) -> Option<Self> {
            match fcf & MacHeader::FCF_FRAME_TYPE_MASK {
                0 => Some(Self::Beacon),
                1 => Some(Self::Data),
                2 => Some(Self::Ack),
                3 => Some(Self::Command),
                _ => None,
            }
        }
    }

    /// The supported IEEE 802.15.4 frame address modes
    #[derive(Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    enum FrameAddrMode {
        NotPresent,
        Short,
        Extended,
    }

    impl FrameAddrMode {
        /// Get the destination address mode from the FCF.
        ///
        /// If the mode is not supported, returns `None`.
        #[inline(always)]
        fn get_dst(fcf: u16) -> Option<Self> {
            match (fcf & MacHeader::FCF_FRAME_DST_ADDR_MODE_MASK)
                >> MacHeader::FCF_FRAME_DST_ADDR_MODE_SHIFT
            {
                0 => Some(Self::NotPresent),
                2 => Some(Self::Short),
                3 => Some(Self::Extended),
                _ => None,
            }
        }
    }
}
