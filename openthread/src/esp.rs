//! `Radio` trait implementation for the `esp-hal` ESP IEEE 802.15.4 radio.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use esp_radio::ieee802154::Config as EspConfig;

use crate::fmt::Bytes;
use crate::{Capabilities, Cca, Config, MacCapabilities, PsduMeta, Radio, RadioErrorKind};

pub use esp_radio::ieee802154::Ieee802154;

/// The `esp-hal` ESP IEEE 802.15.4 radio.
pub struct EspRadio<'a> {
    driver: Ieee802154<'a>,
    config: Config,
}

impl<'a> EspRadio<'a> {
    const DEFAULT_CONFIG: Config = Config::new();

    /// Create a new `EspRadio` instance.
    pub fn new(ieee802154: Ieee802154<'a>) -> Self {
        let mut this = Self {
            driver: ieee802154,
            config: Self::DEFAULT_CONFIG,
        };

        this.driver.set_rx_available_callback_fn(Self::rx_callback);
        this.driver.set_tx_done_callback_fn(Self::tx_done_callback);
        this.driver
            .set_tx_failed_callback_fn(Self::tx_failed_callback);

        this.update_driver_config();

        this
    }

    fn update_driver_config(&mut self) {
        let config = &self.config;

        let esp_config = EspConfig {
            auto_ack_tx: true,
            auto_ack_rx: true,
            enhance_ack_tx: true,
            promiscuous: config.promiscuous,
            coordinator: false,
            rx_when_idle: config.rx_when_idle,
            txpower: config.power,
            channel: config.channel,
            cca_threshold: match config.cca {
                Cca::Carrier => 0,
                Cca::Ed { ed_threshold } => ed_threshold as _,
                Cca::CarrierAndEd { ed_threshold } => ed_threshold as _,
                Cca::CarrierOrEd { ed_threshold } => ed_threshold as _,
            },
            cca_mode: match config.cca {
                Cca::Carrier => esp_radio::ieee802154::CcaMode::Carrier,
                Cca::Ed { .. } => esp_radio::ieee802154::CcaMode::Ed,
                Cca::CarrierAndEd { .. } => esp_radio::ieee802154::CcaMode::CarrierAndEd,
                Cca::CarrierOrEd { .. } => esp_radio::ieee802154::CcaMode::CarrierOrEd,
            },
            pan_id: config.pan_id,
            short_addr: config.short_addr,
            ext_addr: config.ext_addr,
            // The default of 10 is too small for OpenThread,
            // which can have bursts of incoming frames, so we increase it to 50.
            // TODO: See if we can get by with a smaller number to save memory.
            rx_queue_size: 50,
            ..Default::default()
        };

        self.driver.set_config(esp_config);
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

    const CAPS: Capabilities = Capabilities::ACK_TIMEOUT
        .union(Capabilities::CSMA_BACKOFF)
        // .union(Capabilities::RX_ON_WHEN_IDLE) TODO: Depends on coex being off in ESP-IDF
        ;

    const MAC_CAPS: MacCapabilities = MacCapabilities::all();

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
        cca: bool,
        ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Self::Error> {
        TX_SIGNAL.reset();

        trace!(
            "802.15.4: About to TX {} bytes ch{}",
            psdu.len(),
            self.config.channel
        );

        self.driver
            .transmit_raw(psdu, cca)
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

        trace!("802.15.4: About to RX on ch{}", self.config.channel);

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
        })
    }
}

// Esp chips have a single radio, so having statics for these is OK
static TX_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
