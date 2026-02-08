//! `Radio` trait implementation for the `esp-hal` ESP IEEE 802.15.4 radio.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use esp_radio::ieee802154::{Config as EspConfig, Error};

use crate::{
    Capabilities, Cca, Config, MacCapabilities, PsduMeta, Radio, RadioError, RadioErrorKind,
};

pub use esp_radio::ieee802154::Ieee802154;

impl RadioError for Error {
    fn kind(&self) -> RadioErrorKind {
        // TODO
        RadioErrorKind::Other
    }
}

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
        this.driver.set_tx_done_callback_fn(Self::tx_callback);

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
            ..Default::default()
        };

        self.driver.set_config(esp_config);
    }

    fn rx_callback() {
        RX_SIGNAL.signal(());
    }

    fn tx_callback() {
        TX_SIGNAL.signal(());
    }
}

impl Radio for EspRadio<'_> {
    type Error = Error;

    fn caps(&mut self) -> Capabilities {
        Capabilities::RX_WHEN_IDLE
    }

    fn mac_caps(&mut self) -> MacCapabilities {
        MacCapabilities::all()
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
        _ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Self::Error> {
        TX_SIGNAL.reset();

        trace!(
            "802.15.4 TX: {} bytes ch{}",
            psdu.len(),
            self.config.channel
        );

        self.driver.transmit_raw(psdu)?;

        TX_SIGNAL.wait().await;

        debug!("802.15.4 TX done");

        Ok(None)
    }

    async fn receive(&mut self, psdu_buf: &mut [u8]) -> Result<PsduMeta, Self::Error> {
        RX_SIGNAL.reset();

        trace!(
            "ESP Radio, about to receive on channel {}",
            self.config.channel
        );

        self.driver.start_receive();

        let raw = loop {
            if let Some(frame) = self.driver.raw_received() {
                break frame;
            }

            RX_SIGNAL.wait().await;
        };

        let psdu_len = (raw.data.len() - 1).min((raw.data[0] & 0x7f) as usize);
        psdu_buf[..psdu_len].copy_from_slice(&raw.data[1..][..psdu_len]);

        let rssi = raw.data[1..][psdu_len] as i8;

        trace!(
            "802.15.4 RX: {} bytes ch{} rssi={}",
            psdu_len,
            raw.channel,
            rssi
        );

        Ok(PsduMeta {
            len: psdu_len,
            channel: raw.channel,
            rssi: Some(rssi),
        })
    }
}

// Esp chips have a single radio, so having statics for these is OK
static TX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
