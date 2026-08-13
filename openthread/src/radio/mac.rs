use core::fmt::Debug;
use core::mem::MaybeUninit;
use core::pin::pin;

use embassy_futures::select::{select, Either};

use embassy_time::Instant;

use crate::fmt::Bytes;
use crate::sys::OT_RADIO_FRAME_MAX_SIZE;
use crate::{
    Config, MacCapabilities, PsduMeta, Radio, RadioCaps, RadioError, RadioErrorKind, SrcMatchConfig,
};

pub(crate) use mac_utils::MacHeader;

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

/// The default depth of a [`MacRadio`]'s pending-RX queue: the frames it
/// accepts and ACKs while a transmission of its own is waiting for its ACK.
pub const DEFAULT_RX_QUEUE_SIZE: usize = 12;

/// The resources for a [`MacRadio`].
///
/// Sized once, by the user, and borrowed by the wrapper for its lifetime.
pub struct MacRadioResources<const RX_QUEUE_SIZE: usize = DEFAULT_RX_QUEUE_SIZE> {
    /// The buffer for the ACK PSDU, if the `MacRadio` is instructed to send or receive ACKs in software.
    ack_psdu_buf: MaybeUninit<[u8; OT_RADIO_FRAME_MAX_SIZE as _]>,
    /// Frames accepted (and ACKed) while `transmit` was waiting for its own ACK, parked here
    /// until subsequent `receive` calls deliver them.
    pending_rx: MaybeUninit<[PendingRxFrame; RX_QUEUE_SIZE]>,
    /// The source-address-match table, consulted for the Frame Pending bit
    /// of the software ACKs answering data polls (see [`SrcMatchConfig`]).
    src_match: SrcMatchConfig,
}

impl<const RX_QUEUE_SIZE: usize> MacRadioResources<RX_QUEUE_SIZE> {
    #[allow(clippy::declare_interior_mutable_const)]
    const INIT_FRAME: PendingRxFrame = PendingRxFrame::new();

    /// Create a new set of `MacRadio` resources.
    pub const fn new() -> Self {
        Self {
            ack_psdu_buf: MaybeUninit::uninit(),
            pending_rx: MaybeUninit::uninit(),
            src_match: SrcMatchConfig::new(),
        }
    }

    /// Initialize the resources, as they start their life as `MaybeUninit` so as to avoid mem-moves.
    ///
    /// Returns the borrowed pieces, with `RX_QUEUE_SIZE` erased into the queue's
    /// slice length - which is what keeps [`MacRadio`] free of a const parameter.
    fn init(&mut self) -> (&mut [u8], &mut [PendingRxFrame], &mut SrcMatchConfig) {
        let ack_psdu_buf = self.ack_psdu_buf.write([0; OT_RADIO_FRAME_MAX_SIZE as _]);
        let pending_rx = self.pending_rx.write([Self::INIT_FRAME; RX_QUEUE_SIZE]);

        (ack_psdu_buf, pending_rx, &mut self.src_match)
    }
}

impl<const RX_QUEUE_SIZE: usize> Default for MacRadioResources<RX_QUEUE_SIZE> {
    fn default() -> Self {
        Self::new()
    }
}

/// An enhanced (MAC) radio that can optionally send and receive ACKs for transmitted frames
/// as well as optionally do address filtering.
///
/// # When it is needed
///
/// The OpenThread stack requires the full MAC-offload set
/// ([`MacCapabilities::REQUIRED`]) from the radio it is handed, and panics
/// otherwise. A radio reporting less than that - a bare PHY, typically - has to
/// be wrapped in this type by the user, which emulates whatever the radio
/// itself does not do:
///
/// ```ignore
/// static MAC_RADIO_RESOURCES: StaticCell<MacRadioResources> = StaticCell::new();
/// let mac_radio_resources = MAC_RADIO_RESOURCES.init(MacRadioResources::new());
///
/// let radio = MacRadio::new(MyBarePhyRadio::new(...), MyTimer, mac_radio_resources);
///
/// ot.run(radio).await
/// ```
///
/// The software emulation has hard timing deadlines (ACKs must go out within
/// the inter-frame gap), so when it is in play, running the radio in a
/// higher-priority executor via [`crate::ProxyRadio`] / [`crate::PhyRadioRunner`]
/// is strongly advisable - the wrapping then happens around the PHY radio on
/// the runner's side.
pub struct MacRadio<'a, R, T> {
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
    /// The wrapped radio's MAC-offloading capabilities, discovered from its
    /// `init`. Any capability the radio lacks, this wrapper emulates in software;
    /// the `transmit`/`receive` paths branch on this at runtime (the wrapped
    /// radio's MAC caps are only known after `init`, so they cannot be a const).
    mac_caps: MacCapabilities,
    /// Whether the radio is in promiscuous mode.
    promiscuous: bool,
    /// A buffer for the MAC header of the received or transmitted frame.
    /// (For filtering and ACKs)
    mac_header: MacHeader,
    /// The buffer for the ACK PSDU, if the `MacRadio` is instructed
    /// to send or receive ACKs in software.
    ack_psdu_buf: &'a mut [u8],
    /// Frames accepted (and ACKed) while `transmit` was waiting for its own
    /// ACK, parked here until subsequent `receive` calls deliver them.
    pending_rx: PendingRx<'a>,
    /// The source-address-match table, consulted for the Frame Pending bit
    /// of the software ACKs answering data polls (see [`SrcMatchConfig`]).
    src_match: &'a mut SrcMatchConfig,
    /// The channel the radio was last commanded onto (by `set_receive` or by
    /// a `transmit`) - the software ACKs are sent on it, since a radio is
    /// only ever on one channel at a time.
    channel: u8,
    /// The transmit power to send the software ACKs with: the one of the last
    /// `transmit`, or the radio's own default until then.
    power: i8,
    /// The PAN ID to filter by, if the filter policy allows it.
    pan_id: u16,
    /// The short address to filter by, if the filter policy allows it.
    short_addr: u16,
    /// The alternate short address to *also* accept, or `BROADCAST_SHORT_ADDR`
    /// (`0xffff`) when there is none. A real destination address never equals
    /// the broadcast sentinel, so an unset alternate never matches — a no-op.
    alt_short_addr: u16,
    /// The extended address to filter by, if the filter policy allows it.
    ext_addr: u64,
}

impl<'a, R, T> MacRadio<'a, R, T>
where
    R: Radio,
    T: MacRadioTimer,
{
    /// The waiting timeout for a TX ACK to be received: OpenThread's own
    /// SubMac ACK timeout (`kAckTimeout`, 16ms).
    ///
    /// Besides matching the stack's expectations, keeping this short bounds
    /// a structural cost of this wrapper: a frame that crosses our
    /// transmission is ACKed and parked during the wait (see `transmit`),
    /// but reaches the stack only once the wait resolves - so on ACK loss,
    /// this timeout is the worst-case added latency for the crossing
    /// frame's processing (and thus for our reply to it). A real radio's
    /// independent RX path has no such serialization.
    const TX_ACK_WAIT_US: u64 = 16 * 1000;
    /// The waiting timeout for an RX ACK to be sent.
    // TODO: Should be 190us, but we need to be more precise
    // and not use `embassy-time` with the NRF...
    const RX_ACK_SEND_US: u64 = 10;

    /// Create a new enhanced MAC radio.
    ///
    /// Arguments:
    /// - `radio`: The radio to wrap.
    /// - `timer`: The timer implementation to use. Should be with a high precision of ideally < 10us
    /// - `resources`: The resources to borrow the software MAC's buffers from.
    pub fn new<const RX_QUEUE_SIZE: usize>(
        radio: R,
        timer: T,
        resources: &'a mut MacRadioResources<RX_QUEUE_SIZE>,
    ) -> Self {
        let (ack_psdu_buf, pending_rx, src_match) = resources.init();

        Self {
            radio,
            timer,
            // Filled from the wrapped radio's `init`; until then assume no MAC
            // offload (the wrapper emulates everything).
            mac_caps: MacCapabilities::empty(),
            mac_header: MacHeader::new(),
            ack_psdu_buf,
            pending_rx: PendingRx::new(pending_rx),
            src_match,
            channel: 11,
            power: RadioCaps::DEFAULT_TX_POWER,
            promiscuous: false,
            pan_id: MacHeader::BROADCAST_PAN_ID,
            short_addr: MacHeader::BROADCAST_SHORT_ADDR,
            alt_short_addr: MacHeader::BROADCAST_SHORT_ADDR,
            ext_addr: MacHeader::BROADCAST_EXT_ADDR,
        }
    }

    /// Screen an incoming frame: apply the software address filters the
    /// wrapped radio does not offload and - for an accepted frame that
    /// requests one - send the ACK. Returns whether the frame is for us and
    /// should be delivered to the stack.
    ///
    /// Shared by the `receive` path and by `transmit`'s ACK wait, so that a
    /// frame crossing our transmission is served all the same (see there).
    ///
    /// `psdu` must not alias `self`'s buffers (callers pass caller-owned or
    /// stack copies).
    async fn screen_incoming(&mut self, psdu: &[u8]) -> Result<bool, MacRadioError<R::Error>> {
        if self.mac_caps == MacCapabilities::all() {
            return Ok(true);
        }

        if self.mac_header.load(psdu).is_none() {
            trace!(
                "MacRadio, received frame with invalid MAC header, dropping: {}",
                Bytes(psdu)
            );
            return Ok(false);
        }

        if !self.mac_caps.contains(MacCapabilities::PROMISCUOUS) && !self.promiscuous {
            if !self.mac_caps.contains(MacCapabilities::FILTER_PAN_ID)
                && self.mac_header.pan_id != MacHeader::BROADCAST_PAN_ID
                && self.mac_header.pan_id != self.pan_id
            {
                trace!(
                    "MacRadio, filtering out frame: {}, PAN ID does not match",
                    Bytes(psdu)
                );
                return Ok(false);
            }

            if !self.mac_caps.contains(MacCapabilities::FILTER_SHORT_ADDR)
                && self.mac_header.dst_short_addr != MacHeader::BROADCAST_SHORT_ADDR
                && self.mac_header.dst_short_addr != self.short_addr
                && self.mac_header.dst_short_addr != self.alt_short_addr
            {
                trace!(
                    "MacRadio, filtering out frame: {}, short address does not match",
                    Bytes(psdu)
                );
                return Ok(false);
            }

            if !self.mac_caps.contains(MacCapabilities::FILTER_EXT_ADDR)
                && self.mac_header.dst_ext_addr != MacHeader::BROADCAST_EXT_ADDR
                && self.mac_header.dst_ext_addr != self.ext_addr
            {
                trace!(
                    "MacRadio, filtering out frame: {}, extended address does not match",
                    Bytes(psdu)
                );
                return Ok(false);
            }

            if !self.mac_caps.contains(MacCapabilities::RX_ACK) && self.mac_header.needs_ack() {
                let ack_at = self.timer.now() + Self::RX_ACK_SEND_US;

                // Ack MAC command frames with Frame Pending set: a
                // sleepy child's data poll is a command frame, and the
                // FP bit is what keeps the child awake for the queued
                // indirect frame. Conservative - like the C simulation
                // radio with source matching disabled: a spurious FP
                // only keeps a child listening briefly when nothing
                // is queued.
                let frame_pending = self.mac_header.is_command()
                    && self.src_match.ack_frame_pending(
                        self.mac_header.src_short_addr,
                        self.mac_header.src_ext_addr,
                    );

                let ack_len = self.mac_header.prep_ack(self.ack_psdu_buf, frame_pending);

                trace!(
                    "MacRadio, about to transmit ACK: {}",
                    Bytes(&self.ack_psdu_buf[..ack_len])
                );

                if self.timer.now() < ack_at {
                    self.timer.wait(ack_at).await;
                }

                self.radio
                    .transmit(
                        &self.ack_psdu_buf[..ack_len],
                        self.channel,
                        self.power,
                        // An ACK is sent in the inter-frame gap, without CCA:
                        // the medium is ours for the turnaround.
                        None,
                        None,
                    )
                    .await
                    .map_err(MacRadioError::TxAckFailed)?;
            }
        }

        Ok(true)
    }
}

impl<R, T> Radio for MacRadio<'_, R, T>
where
    R: Radio,
    T: MacRadioTimer,
{
    type Error = MacRadioError<R::Error>;

    async fn init(&mut self) -> Result<RadioCaps, Self::Error> {
        let caps = self.radio.init().await.map_err(Self::Error::Io)?;

        // Remember the wrapped radio's *actual* MAC offload so `transmit`/
        // `receive` know what to emulate. To the outside, a `MacRadio` provides
        // the full MAC-offload set — whatever the hardware doesn't do, this
        // wrapper does in software — while the PHY caps pass through unchanged.
        self.mac_caps = caps.mac;

        // Outwards: the full MAC-offload set - whatever the hardware doesn't
        // do, this wrapper does in software - with ONE exception: for an
        // inner radio that sends its own RX ACKs but has no source-match
        // table, `SRC_MATCH` cannot be claimed by anyone (the ACKs' Frame
        // Pending bits are decided below, out of software's reach).
        let mut mac = MacCapabilities::all();
        if caps.mac.contains(MacCapabilities::RX_ACK)
            && !caps.mac.contains(MacCapabilities::SRC_MATCH)
        {
            mac.remove(MacCapabilities::SRC_MATCH);
        }

        self.power = caps.default_tx_power;

        Ok(RadioCaps { mac, ..caps })
    }

    async fn set_receive(&mut self, channel: u8) -> Result<(), Self::Error> {
        // Remembered for the software ACKs (see `channel`); the inner radio
        // needs it too - if it queues frames of its own, it must be on the
        // right channel to fill that queue.
        self.channel = channel;

        self.radio
            .set_receive(channel)
            .await
            .map_err(Self::Error::Io)
    }

    async fn set_sleep(&mut self) -> Result<(), Self::Error> {
        // The parked frames stay parked - they were screened and ACKed while
        // awake, so they are legitimately received and still owed to the
        // caller.
        self.radio.set_sleep().await.map_err(Self::Error::Io)
    }

    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error> {
        self.radio
            .set_config(config)
            .await
            .map_err(Self::Error::Io)?;

        self.promiscuous = config.promiscuous;
        self.pan_id = config.pan_id.unwrap_or(MacHeader::BROADCAST_PAN_ID);
        self.short_addr = config.short_addr.unwrap_or(MacHeader::BROADCAST_SHORT_ADDR);
        self.alt_short_addr = config
            .alt_short_addr
            .unwrap_or(MacHeader::BROADCAST_SHORT_ADDR);
        self.ext_addr = config.ext_addr.unwrap_or(MacHeader::BROADCAST_EXT_ADDR);

        Ok(())
    }

    async fn set_src_match_config(&mut self, entries: &SrcMatchConfig) -> Result<(), Self::Error> {
        if self.mac_caps.contains(MacCapabilities::SRC_MATCH) {
            // The inner radio's own acking honors the table - hand it down.
            self.radio
                .set_src_match_config(entries)
                .await
                .map_err(Self::Error::Io)
        } else {
            // This wrapper's software ACKs consult the copy.
            *self.src_match = entries.clone();

            Ok(())
        }
    }

    async fn energy_scan(&mut self, channel: u8, duration_millis: u16) -> Result<i8, Self::Error> {
        // Energy scan involves no MAC-layer processing - pass through.
        self.radio
            .energy_scan(channel, duration_millis)
            .await
            .map_err(Self::Error::Io)
    }

    async fn transmit(
        &mut self,
        psdu: &[u8],
        channel: u8,
        power: i8,
        cca_threshold: Option<i8>,
        ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Self::Error> {
        trace!("MacRadio, about to transmit");

        // A transmit puts the radio on this channel, and the ACKs this
        // wrapper sends follow the same power as the traffic it emits.
        self.channel = channel;
        self.power = power;

        if self.mac_caps.contains(MacCapabilities::TX_ACK) {
            let result = self
                .radio
                .transmit(psdu, channel, power, cca_threshold, ack_psdu_buf)
                .await
                .map_err(Self::Error::Io);

            trace!("MacRadio, transmitted");

            result
        } else {
            self.radio
                .transmit(psdu, channel, power, cca_threshold, None)
                .await
                .map_err(Self::Error::Io)?;

            let sent_at = self.timer.now();

            self.mac_header.load(psdu).ok_or(MacRadioError::TxInvalid)?;

            if self.mac_header.needs_ack() {
                let psdu_seq = self.mac_header.seq;

                trace!("MacRadio, about to receive transmit ACK");

                // Wait for the matching ACK until the deadline, SERVING the
                // medium meanwhile: on a receive-everything PHY, other frames
                // routinely land in this window (a neighbor's broadcast, a
                // crossing transmission), and they must be screened, ACKed
                // and parked for `receive` just as if no wait were running -
                // which is what a real radio's independent RX path (and the C
                // simulation radio in TX wait) does. Merely dropping them
                // makes two nodes retransmitting to each other mutually deaf
                // - each sitting in its own ACK wait, ACKing nothing - until
                // their MAC retry budgets run out.
                let ack_meta = loop {
                    let result = {
                        let mut ack = pin!(self.radio.receive(self.ack_psdu_buf));
                        let mut timeout = pin!(self.timer.wait(sent_at + Self::TX_ACK_WAIT_US));

                        select(&mut ack, &mut timeout).await
                    };

                    let meta = match result {
                        Either::First(result) => result.map_err(Self::Error::RxAckFailed)?,
                        Either::Second(_) => {
                            trace!("MacRadio, transmit ACK timeout");

                            Err(Self::Error::RxAckTimeout)?
                        }
                    };

                    let psdu = &self.ack_psdu_buf[..meta.len];
                    if self
                        .mac_header
                        .load(psdu)
                        .is_some_and(|()| self.mac_header.ack_for(psdu_seq))
                    {
                        break meta;
                    }

                    // A crossing frame: screen it off a stack copy (the ACK
                    // buffer is about to be reused for both the ACK we may
                    // send and the wait's next read).
                    let mut crossing = [0; OT_RADIO_FRAME_MAX_SIZE as _];
                    crossing[..meta.len].copy_from_slice(&self.ack_psdu_buf[..meta.len]);

                    if self.screen_incoming(&crossing[..meta.len]).await?
                        && !self.pending_rx.push_back(meta, &crossing[..meta.len])
                    {
                        trace!(
                            "MacRadio, crossing-frame queue full, dropped: {}",
                            Bytes(&crossing[..meta.len])
                        );
                    }
                };

                let ack_psdu = &self.ack_psdu_buf[..ack_meta.len];

                if let Some(ack_psdu_buf) = ack_psdu_buf {
                    ack_psdu_buf[..ack_psdu.len()].copy_from_slice(ack_psdu);
                }

                trace!("MacRadio, transmitted with ACK");

                // Report the received ACK: the stack reads more than "it was
                // acked" out of it - notably the Frame Pending bit, which
                // tells a sleepy child whether to stay awake for a frame its
                // parent has queued.
                Ok(Some(ack_meta))
            } else {
                trace!("MacRadio, transmitted without ACK");

                Ok(None)
            }
        }
    }

    async fn receive(&mut self, psdu_buf: &mut [u8]) -> Result<PsduMeta, Self::Error> {
        // A frame screened - and already ACKed - during a transmission's ACK
        // wait is delivered first.
        if let Some(psdu_meta) = self.pending_rx.pop_front(psdu_buf) {
            trace!(
                "MacRadio, delivering frame parked during an ACK wait: {}",
                Bytes(&psdu_buf[..psdu_meta.len])
            );

            return Ok(psdu_meta);
        }

        loop {
            trace!("MacRadio, about to receive");

            let psdu_meta = self
                .radio
                .receive(psdu_buf)
                .await
                .map_err(Self::Error::Io)?;

            trace!(
                "MacRadio, received: {}, meta: {:?}",
                Bytes(&psdu_buf[..psdu_meta.len]),
                psdu_meta
            );

            if self.screen_incoming(&psdu_buf[..psdu_meta.len]).await? {
                trace!(
                    "MacRadio, received frame: {}",
                    Bytes(&psdu_buf[..psdu_meta.len])
                );

                break Ok(psdu_meta);
            }
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

/// A frame parked in the [`MacRadio`] pending-RX queue.
struct PendingRxFrame {
    /// The meta-data of the parked frame
    meta: PsduMeta,
    /// The PSDU of the parked frame, valid up to `meta.len`
    psdu: [u8; OT_RADIO_FRAME_MAX_SIZE as _],
}

impl PendingRxFrame {
    /// Create a new, empty parked frame.
    const fn new() -> Self {
        Self {
            meta: PsduMeta {
                len: 0,
                channel: 0,
                rssi: None,
            },
            psdu: [0; OT_RADIO_FRAME_MAX_SIZE as _],
        }
    }
}

/// The pending-RX queue: a ring buffer over the user-provided frame slots.
struct PendingRx<'a> {
    /// The frame slots, as borrowed from `MacRadioResources`
    frames: &'a mut [PendingRxFrame],
    /// The slot holding the oldest parked frame
    head: usize,
    /// The number of parked frames
    len: usize,
}

impl<'a> PendingRx<'a> {
    /// Create a new, empty queue over `frames`.
    const fn new(frames: &'a mut [PendingRxFrame]) -> Self {
        Self {
            frames,
            head: 0,
            len: 0,
        }
    }

    /// Park a frame.
    ///
    /// Returns `false` - and drops the frame, like a saturated real radio -
    /// if the queue is full.
    fn push_back(&mut self, meta: PsduMeta, psdu: &[u8]) -> bool {
        if self.len == self.frames.len() {
            return false;
        }

        let frame = &mut self.frames[(self.head + self.len) % self.frames.len()];

        frame.meta = meta;
        frame.psdu[..psdu.len()].copy_from_slice(psdu);

        self.len += 1;

        true
    }

    /// Take the oldest parked frame, if any, into `psdu_buf`.
    fn pop_front(&mut self, psdu_buf: &mut [u8]) -> Option<PsduMeta> {
        if self.len == 0 {
            return None;
        }

        let frame = &self.frames[self.head];
        let meta = frame.meta;

        psdu_buf[..meta.len].copy_from_slice(&frame.psdu[..meta.len]);

        self.head = (self.head + 1) % self.frames.len();
        self.len -= 1;

        Some(meta)
    }
}

/// A minimal set of utilities for parsing the IEEE 802.15.4 MAC header
/// for the purposes of MAC filtering and RX/TX ACK processing.
mod mac_utils {
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
        /// Source short address
        /// 0xffff if the Frame does not carry a short source address
        pub src_short_addr: u16,
        /// Source extended address
        /// 0xffffffffffffffff if the Frame does not carry an extended source
        /// address
        pub src_ext_addr: u64,
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
        const FCF_FRAME_SRC_ADDR_MODE_SHIFT: u16 = 14;
        const FCF_FRAME_SRC_ADDR_MODE_MASK: u16 = 0x03 << Self::FCF_FRAME_SRC_ADDR_MODE_SHIFT;

        /// Create a new empty MAC header.
        pub const fn new() -> Self {
            Self {
                fcf: 0,
                seq: 0,
                pan_id: 0,
                dst_short_addr: 0,
                dst_ext_addr: 0,
                src_short_addr: 0,
                src_ext_addr: 0,
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
                    self.dst_ext_addr = u64::from_le_bytes(unwrap!(psdu[5..13].try_into()));
                    self.dst_short_addr = Self::BROADCAST_SHORT_ADDR;
                }
            }

            let src_addr_mode = FrameAddrMode::get_src(self.fcf)?;

            // Offset just past the destination addressing fields.
            let mut offs = Self::ADDRS_OFFSET
                + match dst_addr_mode {
                    FrameAddrMode::NotPresent => 0,
                    FrameAddrMode::Short => 2 + 2,
                    FrameAddrMode::Extended => 2 + 8,
                };

            // The source PAN ID is elided when the PAN ID Compression bit is
            // set - the common case for intra-PAN traffic, data polls
            // included. (The elision rule modeled here is the 2003/2006 one;
            // the 2015 frame version's table-based rules are not - the
            // consumers of the source fields only look at 2006-era command
            // frames.)
            if !matches!(src_addr_mode, FrameAddrMode::NotPresent)
                && (self.fcf & Self::FCF_PAN_ID_COMPRESSION_MASK) == 0
            {
                offs += 2;
            }

            match src_addr_mode {
                FrameAddrMode::NotPresent => {
                    self.src_short_addr = Self::BROADCAST_SHORT_ADDR;
                    self.src_ext_addr = Self::BROADCAST_EXT_ADDR;
                }
                FrameAddrMode::Short => {
                    Self::ensure_len(psdu, offs + 2 + Self::CRC_LEN)?;

                    self.src_short_addr =
                        u16::from_le_bytes(unwrap!(psdu[offs..offs + 2].try_into()));
                    self.src_ext_addr = Self::BROADCAST_EXT_ADDR;
                }
                FrameAddrMode::Extended => {
                    Self::ensure_len(psdu, offs + 8 + Self::CRC_LEN)?;

                    self.src_ext_addr =
                        u64::from_le_bytes(unwrap!(psdu[offs..offs + 8].try_into()));
                    self.src_short_addr = Self::BROADCAST_SHORT_ADDR;
                }
            }

            Some(())
        }

        /// Return `true` if the frame needs an ACK.
        #[inline(always)]
        pub fn needs_ack(&self) -> bool {
            (self.fcf & Self::FCF_ACK_REQ_BIT) != 0
        }

        /// Return `true` if the frame is a MAC command frame.
        ///
        /// A sleepy child's Data Request (data poll) is one; its command ID
        /// sits in the (potentially encrypted) MAC payload, so the frame
        /// *type* is as precise as header-level parsing can get.
        #[inline(always)]
        pub fn is_command(&self) -> bool {
            matches!(FrameType::get(self.fcf), Some(FrameType::Command))
        }

        /// Prepare an ACK PSDU.
        /// Assumes that the parsed frame header indicates that ACK is necessary (`self.needs_ack` returns `true`)
        ///
        /// `frame_pending` sets the ACK's Frame Pending bit - the "stay awake,
        /// data follows" hint a parent gives a polling sleepy child.
        #[inline(always)]
        pub fn prep_ack(&self, ack_buf: &mut [u8], frame_pending: bool) -> usize {
            assert!(ack_buf.len() >= Self::ACK_PSDU_LEN);

            let ack_fcf = Self::FCF_FRAME_TYPE_ACK
                | (self.fcf & Self::FCF_FRAME_VERSION_MASK)
                | if frame_pending {
                    Self::FCF_PENDING_BIT
                } else {
                    0
                };

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

        fn get_src(fcf: u16) -> Option<Self> {
            match (fcf & MacHeader::FCF_FRAME_SRC_ADDR_MODE_MASK)
                >> MacHeader::FCF_FRAME_SRC_ADDR_MODE_SHIFT
            {
                0 => Some(Self::NotPresent),
                2 => Some(Self::Short),
                3 => Some(Self::Extended),
                _ => None,
            }
        }
    }
}
