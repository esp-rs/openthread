//! `Radio` trait implementation for the `esp-hal` ESP IEEE 802.15.4 radio.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use esp_radio::ieee802154::Config as EspConfig;

use crate::fmt::Bytes;
use crate::{
    Capabilities, Config, MacCapabilities, PsduMeta, Radio, RadioCaps, RadioErrorKind,
    SrcMatchConfig,
};

pub use esp_radio::ieee802154::Ieee802154;

/// The `esp-hal` ESP IEEE 802.15.4 radio.
pub struct EspRadio<'a> {
    driver: Ieee802154<'a>,
    config: Config,
    channel: u8,
    power: i8,
    cca_threshold: i8,
    rx_queue_size: usize,
}

impl<'a> EspRadio<'a> {
    const DEFAULT_CONFIG: Config = Config::new();

    /// The transmit power the radio starts with, reported to the stack as
    /// this radio's default (`RadioCaps::default_tx_power`) and used until
    /// OpenThread sets another. Matches `esp-radio`'s own default.
    const DEFAULT_TX_POWER: i8 = 10;

    /// The CCA energy-detect threshold the radio starts with, in dBm - the
    /// ESP-IDF default (`CONFIG_IEEE802154_CCA_THRESHOLD`), which is also
    /// `esp-radio`'s.
    const DEFAULT_CCA_THRESHOLD: i8 = -60;

    /// The channel the radio starts on, until the stack commands another.
    const DEFAULT_CHANNEL: u8 = 11;

    /// Default esp-radio receive-queue depth (frames buffered before drops).
    ///
    /// esp-radio's own default of 10 is too small for OpenThread's RX bursts; 50
    /// is a safer baseline. Bursty Matter commissioning/SRP may need more — see
    /// [`EspRadio::with_rx_queue_size`].
    pub const DEFAULT_RX_QUEUE_SIZE: usize = 50;

    /// Create a new `EspRadio` instance.
    pub fn new(ieee802154: Ieee802154<'a>) -> Self {
        let mut this = Self {
            driver: ieee802154,
            config: Self::DEFAULT_CONFIG,
            channel: Self::DEFAULT_CHANNEL,
            power: Self::DEFAULT_TX_POWER,
            cca_threshold: Self::DEFAULT_CCA_THRESHOLD,
            rx_queue_size: Self::DEFAULT_RX_QUEUE_SIZE,
        };

        this.driver.set_rx_available_callback_fn(Self::rx_callback);
        this.driver.set_tx_done_callback_fn(Self::tx_done_callback);
        this.driver
            .set_tx_failed_callback_fn(Self::tx_failed_callback);

        this.update_driver_config();

        this
    }

    /// Set the esp-radio receive-queue depth: frames buffered before further
    /// incoming frames are dropped (logged as `Receive queue full`).
    ///
    /// Raising the default ([`Self::DEFAULT_RX_QUEUE_SIZE`], e.g. to 200) gives
    /// headroom when the OpenThread consumer stalls under bursty load (e.g.
    /// crypto during commissioning). Each queued frame is a heap buffer of
    /// ~130 bytes, allocated on demand and not freed until reset, so the depth
    /// caps the worst-case RX heap (e.g. 200 ≈ 26 KB).
    #[must_use]
    pub fn with_rx_queue_size(mut self, rx_queue_size: usize) -> Self {
        self.rx_queue_size = rx_queue_size;
        self.update_driver_config();
        self
    }

    fn update_driver_config(&mut self) {
        let config = &self.config;

        let esp_config = EspConfig {
            auto_ack_tx: true,
            auto_ack_rx: true,
            enhance_ack_tx: true,
            promiscuous: config.promiscuous,
            coordinator: false,
            rx_when_idle: !config.auto_sleep,
            txpower: self.power,
            channel: self.channel,
            cca_threshold: self.cca_threshold,
            cca_mode: esp_radio::ieee802154::CcaMode::Ed,
            pan_id: config.pan_id,
            short_addr: config.short_addr,
            // `config.alt_short_addr` (the second short address an FTD accepts
            // during a child-to-router role transition) is NOT programmed here.
            //
            // The ESP 802.15.4 MAC *does* have the hardware for it — up to 4
            // multi-PAN address-filter slots, so the alternate could live in slot
            // 1 alongside the primary in slot 0, keeping full hardware filtering
            // (and hence `MacCapabilities::all()`). But `esp-radio` (0.18) exposes
            // neither a second short address on `Config` nor its `set_multipan_*`
            // / `set_short_address(index, …)` primitives publicly (they are in a
            // private `raw` module). So there is no way to reach slot 1 from here.
            //
            // Because `EspRadio` advertises full HW MAC offload, the software
            // `MacRadio` filter is bypassed too — so on ESP the alternate RLOC16
            // is simply not honored, and frames to it during the ~8 s transition
            // window are dropped (recovered by higher-layer retransmission).
            //
            // TODO: implement once `esp-radio` gains a public multi-PAN /
            // second-short-address API (an upstream `esp-radio` addition, akin to
            // exposing the HW security registers) — then program slot 1 with the
            // alternate + the primary PAN ID and enable slots 0 and 1.
            ext_addr: config.ext_addr,
            rx_queue_size: self.rx_queue_size,
            ..Default::default()
        };

        self.driver.set_config(esp_config);
    }

    /// Apply the operation parameters that now arrive per-operation, pushing
    /// a driver config only when one of them actually changed.
    fn set_op_params(&mut self, channel: u8, power: i8, cca_threshold: i8) {
        if self.channel != channel || self.power != power || self.cca_threshold != cca_threshold {
            self.channel = channel;
            self.power = power;
            self.cca_threshold = cca_threshold;

            self.update_driver_config();
        }
    }

    fn rx_callback() {
        RX_SIGNAL.signal(());
    }

    fn tx_done_callback() {
        TX_SIGNAL.signal(true); // success
    }

    fn tx_failed_callback() {
        TX_SIGNAL.signal(false); // failure
    }
}

impl Radio for EspRadio<'_> {
    type Error = RadioErrorKind;

    async fn init(&mut self) -> Result<RadioCaps, Self::Error> {
        // Fixed, statically-known capabilities of the ESP 802.15.4 radio: it does
        // full MAC offload (auto-ACK, filtering) and CSMA/ACK-timeout in hardware.
        //
        // No `ENERGY_SCAN` (and no `Radio::energy_scan` impl): the ESP 802.15.4
        // hardware does have an energy detector, but `esp-radio`'s `Ieee802154`
        // driver does not expose it (as of 0.18). Until it does, energy scans on
        // this radio yield no measurements (see `Radio::energy_scan`).
        Ok(RadioCaps {
            phy: Capabilities::ACK_TIMEOUT.union(Capabilities::CSMA_BACKOFF),
            // .union(Capabilities::AUTO_SLEEP) TODO: Depends on coex being off in ESP-IDF
            //
            // TODO: Upstream `SRC_MATCH` support to `esp-radio`.
            mac: MacCapabilities::all().difference(MacCapabilities::SRC_MATCH),
            // TODO: Report the ESP 802.15.4 hardware's real figure.
            receive_sensitivity: RadioCaps::DEFAULT_RECEIVE_SENSITIVITY,
            default_tx_power: Self::DEFAULT_TX_POWER,
            default_cca_threshold: Self::DEFAULT_CCA_THRESHOLD,
        })
    }

    async fn set_src_match_config(&mut self, _config: &SrcMatchConfig) -> Result<(), Self::Error> {
        // No-op (but will be called) because the driver does not report `MacCapabilities::SRC_MATCH`
        Ok(())
    }

    async fn set_receive(&mut self, channel: u8) -> Result<(), Self::Error> {
        self.set_op_params(channel, self.power, self.cca_threshold);

        self.driver.start_receive();

        Ok(())
    }

    async fn set_sleep(&mut self) -> Result<(), Self::Error> {
        // TODO: Upstream in `esp-radio` a `stop_receive` counterpart to the existing `start_receive`
        // and call it here.
        //
        // Until that gap is closed: the receiver stays on while OpenThread believes this node is asleep,
        // so a sleepy end device neither saves the power it parked for, nor genuinely misses the traffic
        // the stack assumes it missed (see `docs/radio-contract.md`, C6) - frames keep accumulating in
        // the driver queue and are delivered late on the next `receive`.
        Ok(())
    }

    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error> {
        if self.config != *config {
            debug!("Setting radio config: {:?}", config);

            self.config = config.clone();
            self.update_driver_config();
        }

        Ok(())
    }

    async fn transmit(
        &mut self,
        psdu: &[u8],
        channel: u8,
        power: i8,
        cca_threshold: Option<i8>,
        ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Self::Error> {
        TX_SIGNAL.reset();

        // The threshold only matters when CCA is performed at all; keep the
        // current one otherwise, so a no-CCA frame does not churn the config.
        self.set_op_params(channel, power, cca_threshold.unwrap_or(self.cca_threshold));

        trace!("802.15.4: About to TX {} bytes ch{}", psdu.len(), channel);

        self.driver
            .transmit_raw(psdu, cca_threshold.is_some())
            .map_err(|_| RadioErrorKind::Other)?;

        let success = TX_SIGNAL.wait().await;

        if success {
            trace!("802.15.4: TX done");

            if let Some(ack_psdu_buf) = ack_psdu_buf {
                // After tx_done signal received, get the ACK frame:
                if let Some(ack_frame) = self.driver.get_ack_frame() {
                    if ack_frame.data.len() >= 1 {
                        // Must have at least 1 byte for PSDU
                        let ack_psdu_len =
                            (ack_frame.data.len() - 1).min((ack_frame.data[0] & 0x7f) as usize);

                        if ack_psdu_len <= ack_psdu_buf.len() {
                            ack_psdu_buf[..ack_psdu_len]
                                .copy_from_slice(&ack_frame.data[1..][..ack_psdu_len]);

                            trace!(
                                "802.15.4: ACK: {} on ch{}",
                                Bytes(&ack_psdu_buf[..ack_psdu_len]),
                                ack_frame.channel
                            );

                            // Only read RSSI if there is at least one byte after the PSDU.
                            let rssi = if ack_frame.data.len() > 1 + ack_psdu_len {
                                Some(ack_frame.data[1..][ack_psdu_len] as i8)
                            } else {
                                None
                            };

                            return Ok(Some(PsduMeta {
                                len: ack_psdu_len,
                                channel: ack_frame.channel,
                                rssi,
                                lqi: None,
                            }));
                        } else {
                            trace!(
                                "802.15.4: ACK frame too large for provided buffer: {} bytes",
                                ack_psdu_len
                            );
                        }
                    }
                }
            }

            Ok(None)
        } else {
            trace!("802.15.4: TX failed");

            // Report as a failure so OpenThread SubMac retries
            Err(RadioErrorKind::TxFailed)
        }
    }

    async fn receive(&mut self, psdu_buf: &mut [u8]) -> Result<PsduMeta, Self::Error> {
        RX_SIGNAL.reset();

        trace!("802.15.4: About to RX on ch{}", self.channel);

        // Idempotent: `set_receive` has normally armed the driver already, but
        // a `receive` following a transmit needs the RX path back on.
        self.driver.start_receive();

        let raw = loop {
            if let Some(frame) = self.driver.raw_received() {
                break frame;
            }

            RX_SIGNAL.wait().await;
        };

        if raw.data.len() < 1 {
            // Must have at least 1 byte for PSDU
            return Err(RadioErrorKind::Other);
        }

        let psdu_len = (raw.data.len() - 1).min((raw.data[0] & 0x7f) as usize);
        if psdu_len > psdu_buf.len() {
            // PSDU length is larger than the provided buffer
            trace!(
                "802.15.4: Received frame too large for provided buffer: {} bytes",
                psdu_len
            );
            return Err(RadioErrorKind::Other);
        }

        psdu_buf[..psdu_len].copy_from_slice(&raw.data[1..][..psdu_len]);

        // Only read RSSI if there is at least one byte after the PSDU.
        let rssi = if raw.data.len() > 1 + psdu_len {
            Some(raw.data[1..][psdu_len] as i8)
        } else {
            None
        };

        trace!(
            "802.15.4: RX {} bytes ch{} rssi={:?}",
            psdu_len,
            raw.channel,
            rssi
        );

        Ok(PsduMeta {
            len: psdu_len,
            channel: raw.channel,
            rssi,
            lqi: None,
        })
    }
}

// Esp chips have a single radio, so having statics for these is OK
static TX_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
