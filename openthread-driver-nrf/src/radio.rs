//! [`NrfRadio`]: the `openthread_driver::Radio` implementation over the `nrf-802154` driver.

use nrf_802154::{Error, MacKey, PsduMeta, Radio, TxError, MAX_PSDU_SIZE};

/// The error of [`NrfRadio`]: an error of the `nrf-802154` driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NrfRadioError(pub Error);

impl From<Error> for NrfRadioError {
    fn from(error: Error) -> Self {
        Self(error)
    }
}

impl openthread_driver::RadioError for NrfRadioError {
    fn kind(&self) -> openthread_driver::RadioErrorKind {
        use openthread_driver::RadioErrorKind;

        match self.0 {
            Error::TransmitDataTooLarge | Error::ReceiveBufTooSmall => RadioErrorKind::Other,
            // Couldn't even hand the frame to the driver (radio busy after the
            // schedule retries). Treat like a channel-access failure so OpenThread
            // backs off and retransmits.
            Error::ScheduleTransmit => RadioErrorKind::ChannelAccessFailure,
            Error::Transmit(e) => match e {
                // CSMA-CA gave up: the channel was still busy after all backoffs.
                TxError::BusyChannel => RadioErrorKind::ChannelAccessFailure,
                // The frame went out but no valid ACK came back: OpenThread
                // applies its no-ACK retransmission policy, and counts these
                // towards the neighbor's link failures.
                TxError::NoAck | TxError::InvalidAck => RadioErrorKind::NoAck,
                // MPSL denied/ended our radio timeslot — no airtime to transmit.
                // Closest to a channel-access failure; let OpenThread retry.
                TxError::TimeslotEnded | TxError::TimeslotDenied => {
                    RadioErrorKind::ChannelAccessFailure
                }
                // Aborted by another op, out of ACK buffers, or an unknown driver
                // code: a generic transmit failure (→ `OT_ERROR_ABORT`).
                TxError::Aborted | TxError::NoMem | TxError::Unknown(_) => RadioErrorKind::Other,
                _ => RadioErrorKind::Other,
            },
            Error::EnterReceive | Error::Receive | Error::ScheduleReceive => RadioErrorKind::Other,
            Error::EnterSleep | Error::Security(_) => RadioErrorKind::Other,
            _ => RadioErrorKind::Other,
        }
    }
}

/// The IEEE 802.15.4 radio of the nRF chips for OpenThread: a wrapper around the
/// `nrf-802154` [`Radio`] that implements the [`openthread_driver::Radio`] trait
/// with config caching.
///
/// OpenThread pushes its standing configuration before radio operations, and
/// channel/power now arrive with each operation. Without caching, every call
/// would go directly to the Nordic 802.15.4 C driver, which can disrupt
/// in-progress radio operations. This wrapper caches the last applied values
/// and only forwards actual changes to the driver.
///
/// This matches the caching pattern of `EspRadio`
/// (`openthread-driver-esp`).
///
/// # Example
///
/// ```ignore
/// let radio = nrf_802154::Radio::new(/* ... */);
/// let ot_radio = openthread_driver_nrf::NrfRadio::new(radio);
/// // Pass ot_radio to OpenThread::run() or EnetRunner::run()
/// ```
pub struct NrfRadio<'d> {
    radio: Radio<'d>,
    config: openthread_driver::Config,
    power: i8,
    cca_threshold: Option<i8>,
    /// The CSMA-CA backoff limit last applied to the driver; `None` while it
    /// is still the driver's default.
    csma_max_backoffs: Option<u8>,
    /// Whether the receiver is currently commanded by a timed window
    /// (`set_receive_at`) rather than by `set_receive` / `set_sleep`.
    timed_rx: bool,
    /// The channel of the last timed window, which frames received in it
    /// arrived on.
    window_channel: u8,
    /// The last CSL schedule applied to the driver.
    csl: openthread_driver::CslConfig,
    /// The clock accuracy reported to a CSL parent, in PPM.
    csl_accuracy_ppm: u8,
    /// The timed-receive uncertainty reported to a CSL parent, in 10 µs units.
    csl_uncertainty: u8,
}

/// The driver clock, as an `openthread_driver::RadioClock`.
fn radio_now_us() -> u64 {
    unsafe { nrf_802154::raw::nrf_802154_time_get() }
}

impl<'d> NrfRadio<'d> {
    /// Create a new `NrfRadio` wrapper around the given radio.
    ///
    /// The initial config is applied to the driver immediately.
    pub fn new(mut radio: Radio<'d>) -> Self {
        let config = openthread_driver::Config::new();
        Self::apply_config(&mut radio, &config);
        let power = radio.tx_power();
        Self {
            radio,
            config,
            power,
            // The C driver's PIB default: Energy Detection at -75 dBm - which
            // is also OpenThread's own default threshold.
            cca_threshold: Some(-75),
            csma_max_backoffs: None,
            timed_rx: false,
            window_channel: 0,
            csl: openthread_driver::CslConfig::new(),
            csl_accuracy_ppm: Self::DEFAULT_CSL_ACCURACY_PPM,
            csl_uncertainty: Self::DEFAULT_CSL_UNCERTAINTY,
        }
    }

    /// The default clock accuracy reported to a CSL parent: a 32.768 kHz
    /// crystal (LFXO) driving the SL timer, ±20 PPM. With the internal RC
    /// oscillator (LFRC, ±250 PPM even when calibrated) set the real figure
    /// with [`with_csl_timing`](Self::with_csl_timing), or the parent's windows
    /// will be too narrow and frames will be missed.
    ///
    /// On nRF52/nRF53, a CSL *parent* needs the crystal regardless: the driver
    /// timestamps received frames with a 16 MHz timer, re-aligned with the LP
    /// clock only every ~17 s, and the RC oscillator (whose rate moves with
    /// each recalibration) drifts hundreds of µs away from it in between -
    /// timing the parent's transmissions to its children off their windows.
    /// The nRF54L timestamps off the LP clock itself.
    pub const DEFAULT_CSL_ACCURACY_PPM: u8 = 20;

    /// The default timed-receive uncertainty reported to a CSL parent, in units
    /// of 10 µs: the jitter of the driver's receive-window start (SL timer
    /// granularity plus the radio ramp-up), a conservative ±120 µs.
    pub const DEFAULT_CSL_UNCERTAINTY: u8 = 12;

    /// Set the CSL timing figures this radio reports to its parent
    /// (`RadioCaps::csl_accuracy_ppm` / `csl_uncertainty`); see the defaults
    /// for what they mean.
    #[must_use]
    pub const fn with_csl_timing(mut self, accuracy_ppm: u8, uncertainty: u8) -> Self {
        self.csl_accuracy_ppm = accuracy_ppm;
        self.csl_uncertainty = uncertainty;
        self
    }

    fn apply_config(radio: &mut Radio<'_>, config: &openthread_driver::Config) {
        // CCA is per-transmit: OpenThread's threshold arrives with each frame
        // and is applied there (see `transmit`).
        radio.set_promiscuous(config.promiscuous);
        radio.set_pan_id(config.pan_id);
        radio.set_short_addr(config.short_addr);
        // `alt_short_addr` is disregarded: the Nordic driver's hardware filter
        // matches a single short address (see the `Config::alt_short_addr`
        // docs - radios like this one are allowed to ignore it).
        radio.set_ext_addr(config.ext_addr);
        // The trait's polarity is "may the radio power down when idle";
        // the driver's is "keep the receiver on when idle".
        radio.set_rx_when_idle(!config.auto_sleep);
    }
}

impl openthread_driver::Radio for NrfRadio<'_> {
    type Error = NrfRadioError;

    async fn init(&mut self) -> Result<openthread_driver::RadioCaps, Self::Error> {
        // Fixed, statically-known capabilities of the Nordic SoC radio (no
        // hardware handshake needed, unlike a remote-RCP radio). The MAC that
        // `openthread_driver::Radio` asks for - auto-ACK, address filtering,
        // ACK handling and the source-match table (the ACKs' pending bit
        // consults the driver's pending-bit lists, see `set_src_match_config`)
        // - is all done by the Nordic driver/hardware.
        let mut caps = openthread_driver::RadioCaps::new();
        caps.phy = openthread_driver::Capabilities::ACK_TIMEOUT
            // Hardware CSMA-CA. Required in practice: OpenThread's software
            // CSMA-CA timing is too disrupted when the radio shares a busy
            // executor (e.g. embassy-net), so the attach exchange fails
            // without it.
            .union(openthread_driver::Capabilities::CSMA_BACKOFF)
            // The driver can keep the receiver on during idle periods (or
            // not - `Config::auto_sleep`, forwarded as `rx_when_idle` in
            // `apply_config`), so OpenThread hands it the standing policy
            // instead of issuing explicit sleep/receive commands around
            // every idle gap - a sleep/re-arm gap would drop frames that
            // arrive asynchronously (routed responses, or Parent Responses
            // when the executor is busy with e.g. embassy-net).
            .union(openthread_driver::Capabilities::AUTO_SLEEP)
            // The transmit power arrives per-transmit and is applied
            // before each frame (see `transmit`).
            .union(openthread_driver::Capabilities::TRANSMIT_FRAME_POWER)
            // Timed receive (`set_receive_at`), a microsecond radio clock, frame
            // timestamps and enhanced-ACK security with the CSL IE: the
            // driver has it all, so this node can be a CSL (Synchronized)
            // Sleepy End Device.
            .union(openthread_driver::Capabilities::RECEIVE_TIMING)
            // The driver finishes the frames it sends: frame counter and
            // CSL IE written at transmit time (its security and IE
            // writers), AES-CCM* with the keys from `set_mac_keys`.
            .union(openthread_driver::Capabilities::TRANSMIT_SEC)
            // Timed transmit (`PsduTxInfo::tx_at_us`), which a Thread FTD
            // uses to send into the receive windows of its CSL children.
            .union(openthread_driver::Capabilities::TRANSMIT_TIMING);
        // The Nordic OT platform's receive-sensitivity figure for this radio.
        caps.receive_sensitivity = -100;
        // Whatever the driver came up with (0 dBm unless overridden before
        // wrapping).
        caps.default_tx_power = self.radio.tx_power();
        // The C driver's PIB default ED threshold (see `apply_config`).
        caps.default_cca_threshold = -75;
        caps.clock = Some(openthread_driver::RadioClock(radio_now_us));
        caps.csl_accuracy_ppm = self.csl_accuracy_ppm;
        caps.csl_uncertainty = self.csl_uncertainty;

        Ok(caps)
    }

    async fn set_config(&mut self, config: &openthread_driver::Config) -> Result<(), Self::Error> {
        if self.config != *config {
            let to_auto_sleep = config.auto_sleep && !self.config.auto_sleep;

            self.config = config.clone();
            Self::apply_config(&mut self.radio, &self.config);

            // The driver only takes the new idle behavior after the operation in
            // progress: a receiver on because the node used to receive when idle
            // would stay on until the next one. So the switch to sleeping when
            // idle puts it to sleep right away (unless it is busy, or a receive
            // window is pending - those end in sleep by themselves now).
            if to_auto_sleep {
                self.radio.sleep_if_idle();
            }
        }

        Ok(())
    }

    async fn set_src_match_config(
        &mut self,
        config: &openthread_driver::SrcMatchConfig,
    ) -> Result<(), Self::Error> {
        // The table is small and changes rarely (children with pending
        // indirect frames), so rebuild it wholesale instead of diffing.
        self.radio.clear_pending();

        for &addr in &config.short_addrs {
            // A full driver list (`NRF_802154_PENDING_SHORT_ADDRESSES`) drops
            // the overflowing child to FP = 0. The `openthread` glue already
            // caps the table at its own capacity by answering `NO_BUFS`, which
            // makes OpenThread fall back to FP-on-every-ACK - so overflow here
            // means the driver list is configured smaller than that cap.
            self.radio.set_pending_short(addr, true);
        }

        for &addr in &config.ext_addrs {
            self.radio.set_pending_ext(addr, true);
        }

        // Disabled matching = pending bit set in every ACK, which is exactly
        // the `SrcMatchConfig::enabled == false` contract.
        self.radio.set_src_match_enabled(config.enabled);

        Ok(())
    }

    async fn set_receive(&mut self, channel: u8) -> Result<(), Self::Error> {
        self.timed_rx = false;

        if self.radio.channel() != channel {
            self.radio.set_channel(channel);
        }

        // Wake the receiver (the counterpart of `set_sleep`); reception
        // itself runs driver-side, into the IRQ-fed RX queue.
        if !self.radio.enter_receive() {
            return Err(NrfRadioError(Error::EnterReceive));
        }

        Ok(())
    }

    async fn set_receive_at(
        &mut self,
        channel: u8,
        start_us: u64,
        duration_us: u32,
    ) -> Result<(), Self::Error> {
        // The window carries its own channel, and the driver's channel - what
        // it receives on otherwise, and returns to after transmitting - stays:
        // retuning it here would abort a window still running.
        self.window_channel = channel;

        // The driver refuses a start that is not comfortably ahead of its
        // clock, and OpenThread's request may have aged on its way here. Trim
        // a window whose start has (nearly) passed rather than lose it - the
        // frame is still expected in its remaining part.
        const MIN_LEAD_US: u64 = 250;

        let now = self.radio.now_us();
        let end_us = start_us + duration_us as u64;
        let start_us = start_us.max(now + MIN_LEAD_US);

        if end_us <= start_us + MIN_LEAD_US {
            debug!(
                "Timed receive window missed: it ended {} us before it could be armed",
                (now + MIN_LEAD_US).saturating_sub(end_us)
            );

            return Err(NrfRadioError(Error::ScheduleReceive));
        }

        if !self
            .radio
            .receive_at(start_us, (end_us - start_us) as u32, channel)
        {
            return Err(NrfRadioError(Error::ScheduleReceive));
        }

        // Until the next `set_sleep` / `set_receive`, `next_event` only drains
        // the RX queue: the receiver is the window's to switch on and off.
        self.timed_rx = true;

        Ok(())
    }

    async fn set_sleep(&mut self) -> Result<(), Self::Error> {
        if core::mem::take(&mut self.timed_rx) || self.radio.has_pending_window() {
            // A timed window is pending or just ran: the driver sleeps by
            // itself between and after windows, and a frame may be arriving
            // right now - do not abort it. `sleep_if_idle` refusing (busy) is
            // then fine. OpenThread asks for sleep freely (after every
            // operation), so this is the common case for a CSL child.
            self.radio.sleep_if_idle();

            return Ok(());
        }

        // The receiver goes off, so frames sent to a sleeping node are
        // genuinely missed, as the radio contract requires; frames already
        // in the RX queue were received while awake and remain deliverable.
        if !self.radio.sleep() {
            return Err(NrfRadioError(Error::EnterSleep));
        }

        Ok(())
    }

    async fn set_csl(&mut self, csl: &openthread_driver::CslConfig) -> Result<(), Self::Error> {
        let peer_changed =
            self.csl.short_addr != csl.short_addr || self.csl.ext_addr != csl.ext_addr;

        // The parent learns this node's receive schedule from the CSL IE in
        // the enhanced ACKs it gets back, so the IE is injected for exactly
        // the current parent, under both of its addresses.
        if self.csl.enabled() && (!csl.enabled() || peer_changed) {
            self.radio
                .clear_csl_ie_peer(Some(self.csl.short_addr), Some(self.csl.ext_addr));
        }

        if !csl.enabled() {
            // No more windows to keep: whatever is scheduled is stale. A window
            // already running is not ended by the cancel - the radio stays in
            // the receive state - and OpenThread does not ask for sleep after
            // CSL either (a timed-receive radio sleeps by itself between its
            // windows), so put it to sleep here.
            let in_window = self.timed_rx;

            self.radio.receive_at_cancel();

            if in_window {
                self.radio.sleep_if_idle();
            }
        }

        if self.csl.period != csl.period {
            // OpenThread's period is in the same 10-symbol units; it caps it
            // at `u16::MAX` itself.
            self.radio
                .set_csl_period(csl.period.min(u16::MAX as u32) as u16);
        }

        if csl.enabled()
            && (!self.csl.enabled() || peer_changed)
            && !self
                .radio
                .set_csl_ie_peer(Some(csl.short_addr), Some(csl.ext_addr))
        {
            warn!("CSL IE not injected into enhanced ACKs: the driver's ACK data table is full");
        }

        if csl.enabled() && (self.csl.sample_time_us != csl.sample_time_us || !self.csl.enabled()) {
            // OpenThread hands out the low 32 bits of the radio clock; widen it
            // against the current time. Its sample time is the expected time
            // of the first symbol of the frame's MHR, which is exactly the
            // driver's definition of the anchor (the time of CSL phase zero).
            let now = self.radio.now_us();
            let offset = csl.sample_time_us.wrapping_sub(now as u32);
            let sample_time = if offset < 0x80000000 {
                now + offset as u64
            } else {
                now.saturating_sub((u32::MAX - offset + 1) as u64)
            };

            self.radio.set_csl_anchor_time(sample_time);
        }

        self.csl = *csl;

        Ok(())
    }

    async fn set_mac_keys(
        &mut self,
        keys: Option<&openthread_driver::MacKeys>,
    ) -> Result<(), Self::Error> {
        match keys {
            Some(keys) => self
                .radio
                .set_mac_keys(
                    keys.key_id_mode,
                    &[
                        MacKey {
                            key_id: keys.prev_key_id(),
                            key: keys.prev,
                        },
                        MacKey {
                            key_id: keys.key_id,
                            key: keys.curr,
                        },
                        MacKey {
                            key_id: keys.next_key_id(),
                            key: keys.next,
                        },
                    ],
                )
                .map_err(NrfRadioError),
            None => {
                self.radio.clear_mac_keys();

                Ok(())
            }
        }
    }

    async fn set_mac_frame_counter(
        &mut self,
        frame_counter: u32,
        if_larger: bool,
    ) -> Result<(), Self::Error> {
        self.radio.set_frame_counter(frame_counter, if_larger);

        Ok(())
    }

    async fn transmit(
        &mut self,
        psdu: &mut [u8],
        psdu_tx: &mut openthread_driver::PsduTxInfo,
        channel: u8,
        power: i8,
        cca_threshold: Option<i8>,
        ack_psdu_buf: &mut [u8],
    ) -> Result<Option<openthread_driver::PsduRxInfo>, Self::Error> {
        if psdu.len() > MAX_PSDU_SIZE + 2
        /* + FCS */
        {
            return Err(NrfRadioError(Error::TransmitDataTooLarge));
        }

        if ack_psdu_buf.len() < MAX_PSDU_SIZE + 2
        /* + FCS */
        {
            return Err(NrfRadioError(Error::ReceiveBufTooSmall));
        }

        // The frame goes out on its own channel; the driver's channel - which it
        // receives on, and returns to afterwards - stays. Retuning it would
        // abort a timed receive window (on a CSL channel) still running, and
        // leave a CSL parent's receiver on its child's channel.
        if self.power != power {
            self.power = power;
            self.radio.set_tx_power(power);
        }

        let len = psdu.len();
        let data = &mut psdu[..len - 2];
        let ack_len = ack_psdu_buf.len();
        let ack = Some(&mut ack_psdu_buf[..ack_len - 2]);

        // What the driver still has to do to the frame before it goes out
        // (`TRANSMIT_SEC`): assign the frame counter and fill the CSL IE unless
        // the header is final already (a retransmission), and secure it unless
        // it is secured already. The driver does that in its own buffer, and
        // the finished frame is copied back.
        let props = nrf_802154::FrameProps {
            is_secured: psdu_tx.security_processed,
            dynamic_data_is_set: psdu_tx.header_updated,
        };

        // CCA, when requested (a `Some` threshold), is Energy Detection at the
        // requested dBm threshold.
        if let Some(threshold) = cca_threshold {
            if self.cca_threshold != Some(threshold) {
                self.cca_threshold = Some(threshold);
                self.radio.set_cca(nrf_802154::Cca::ed_from_dbm(threshold));
            }
        }

        let cca = cca_threshold.is_some();

        // A timed frame, if it can still make its time: its SHR starts 5
        // octets (10 symbols) before the end of the SFD OpenThread times it
        // by, and the driver wants that comfortably ahead of its clock to fit
        // the CCA and the ramp-up in. A frame that is too late goes out right
        // away, as OpenThread's own timing would send it - the window it is
        // aimed at may still be open.
        const SHR_US: u64 = 10 * 16;
        const MIN_LEAD_US: u64 = 400;

        let tx_start_us = psdu_tx
            .tx_at_us
            .map(|tx_at_us| tx_at_us.saturating_sub(SHR_US))
            .filter(|start_us| *start_us >= self.radio.now_us() + MIN_LEAD_US);

        let result = if let Some(start_us) = tx_start_us {
            Radio::transmit_at_with(&mut self.radio, data, props, start_us, channel, cca, ack).await
        } else if cca
            && psdu_tx.tx_at_us.is_none()
            && psdu_tx
                .max_csma_backoffs
                .is_none_or(|backoffs| backoffs > 0)
        {
            // We advertise `Capabilities::CSMA_BACKOFF`, so OpenThread expects
            // the radio to perform CSMA-CA channel access itself, with as many
            // backoffs as it asks for.
            if let Some(backoffs) = psdu_tx.max_csma_backoffs {
                if self.csma_max_backoffs != Some(backoffs) {
                    self.csma_max_backoffs = Some(backoffs);
                    self.radio.set_csma_ca_max_backoffs(backoffs);
                }
            }

            Radio::transmit_csma_ca_with(&mut self.radio, data, props, channel, ack).await
        } else {
            // No CCA, or a single one and no backoff (a late timed frame, or
            // OpenThread asking for no backoffs).
            Radio::transmit_with(&mut self.radio, data, props, channel, cca, ack).await
        };

        // The driver hands back the frame as it finished it - frame counter,
        // key index and CSL IE set, secured - whether it got through or not:
        // OpenThread repeats the frame counter of an unacknowledged frame to a
        // sleepy child when it retransmits it, so a failure must report it too.
        if let Some(done) = self.radio.last_tx_frame_props() {
            psdu_tx.header_updated |= done.dynamic_data_is_set;
            psdu_tx.security_processed |= done.is_secured;
        }

        let meta = result?;

        Ok(meta.map(|meta| {
            write_crc(&meta, ack_psdu_buf);

            rx_info(&meta, channel)
        }))
    }

    async fn next_event(
        &mut self,
        psdu_buf: &mut [u8],
    ) -> Result<openthread_driver::RadioEvent, Self::Error> {
        if psdu_buf.len() < MAX_PSDU_SIZE + 2
        /* + FCS */
        {
            return Err(NrfRadioError(Error::ReceiveBufTooSmall));
        }

        let len = psdu_buf.len();
        let window = self.timed_rx;
        let meta = if window {
            // Inside a timed window (`set_receive_at`): just wait for the window's
            // frames, the receiver is the driver's to run - and to put to sleep
            // when the window ends.
            Radio::wait_window_frame(&mut self.radio, &mut psdu_buf[..len - 2]).await
        } else {
            // `set_receive` switched the receiver on; from there the driver
            // keeps it on (rx-on-when-idle), or lets it sleep once idle - which
            // switching it back on here, on every frame, would defeat.
            Radio::wait_frame(&mut self.radio, &mut psdu_buf[..len - 2]).await
        };

        write_crc(&meta, psdu_buf);

        let channel = if window {
            self.window_channel
        } else {
            self.radio.channel()
        };

        // OpenThread never commands `Sleep` after a receive window: the radio
        // sleeps once it is over. The driver, receiving when idle, would keep
        // receiving after the window's frame - for a CSL child until the next
        // window, a whole period later; and that frame can end a window that
        // already timed out while it was being received. So the receiver goes
        // off with the window's frame handed over (which only happens once its
        // ACK is out); OpenThread commands `Receive` when it expects more.
        if window {
            self.radio.sleep_if_idle();
        }

        Ok(openthread_driver::RadioEvent::Rx(rx_info(&meta, channel)))
    }
}

/// The `PsduRxInfo` of a frame received on `channel`, with metadata `meta`.
fn rx_info(meta: &PsduMeta, channel: u8) -> openthread_driver::PsduRxInfo {
    let mut info = openthread_driver::PsduRxInfo::new(meta.len as usize + 2, channel);
    info.rssi = Some(meta.power);
    info.lqi = meta.lqi;
    info.timestamp_us = meta.phr_time();
    info.ack_security = meta.ack_security.map(|sec| openthread_driver::AckSecurity {
        frame_counter: sec.frame_counter,
        key_id: sec.key_id,
    });
    info.acked_with_frame_pending = Some(meta.acked_frame_pending);

    info
}

/// Write the FCS of a frame with metadata `meta` after its PSDU in `buf`.
fn write_crc(meta: &PsduMeta, buf: &mut [u8]) {
    let len = meta.len as usize;
    buf[len..len + 2].copy_from_slice(meta.crc.to_le_bytes().as_slice());
}
