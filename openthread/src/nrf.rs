//! `Radio` trait implementation for the `embassy-nrf` ESP IEEE 802.15.4 radio.

pub use embassy_nrf::radio::ieee802154::{Cca as RadioCca, Packet};

use crate::fmt::Bytes;
use crate::{
    Capabilities, Cca, Config, MacCapabilities, PsduMeta, Radio, RadioError, RadioErrorKind,
};

pub use embassy_nrf::radio::ieee802154::Radio as Ieee802154;
pub use embassy_nrf::radio::{Error, Instance as Ieee802154Peripheral};

impl RadioError for Error {
    fn kind(&self) -> RadioErrorKind {
        // TODO
        RadioErrorKind::Other
    }
}

/// The `embassy-nrf` ESP IEEE 802.15.4 radio.
pub struct NrfRadio<'a> {
    driver: Ieee802154<'a>,
    config: Config,
}

impl<'a> NrfRadio<'a> {
    const DEFAULT_CONFIG: Config = Config::new();

    /// Create a new `EspRadio` instance.
    pub fn new(radio: Ieee802154<'a>) -> Self {
        let mut this = Self {
            driver: radio,
            config: Self::DEFAULT_CONFIG,
        };

        this.update_driver_config();

        this
    }

    fn update_driver_config(&mut self) {
        let config = &self.config;

        self.driver.set_channel(config.channel);
        self.driver.set_cca(match config.cca {
            Cca::Carrier => RadioCca::CarrierSense,
            Cca::Ed { ed_threshold } => RadioCca::EnergyDetection { ed_threshold },
            Cca::CarrierAndEd { ed_threshold } => RadioCca::EnergyDetection { ed_threshold },
            Cca::CarrierOrEd { ed_threshold } => RadioCca::EnergyDetection { ed_threshold },
        });
        self.driver.set_transmission_power(config.power);
    }
}

impl Radio for NrfRadio<'_> {
    type Error = Error;

    fn caps(&mut self) -> Capabilities {
        Capabilities::RX_WHEN_IDLE
    }

    fn mac_caps(&mut self) -> MacCapabilities {
        // The NRF radio does not have any MAC offloading capabilities
        MacCapabilities::empty()
    }

    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error> {
        if self.config != *config {
            trace!("Setting radio config: {:?}", config);

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
        trace!("NRF Radio, about to transmit: {}", Bytes(psdu));

        let mut packet = Packet::new();
        // TODO: `embassy-nrf` driver wants the PSDU without the CRC,
        // however, OpenThread provides 2 bytes CRC
        packet.copy_from_slice(&psdu[..psdu.len() - 2]);

        self.driver.try_send(&mut packet).await?;

        trace!("NRF Radio, transmission done");

        Ok(None)
    }

    async fn receive(&mut self, psdu_buf: &mut [u8]) -> Result<PsduMeta, Self::Error> {
        trace!("NRF Radio, about to receive");

        let channel = self.config.channel;

        loop {
            let mut packet = Packet::new();

            let result = self.driver.receive(&mut packet).await;
            if matches!(&result, Err(Error::CrcFailed(_))) {
                trace!("CRC error");
                continue;
            } else {
                result?;
            }

            let len = packet.len() as _;
            psdu_buf[..len].copy_from_slice(&packet);

            trace!("NRF Radio, received: {}", Bytes(&psdu_buf[..len]));

            let lqi = packet.lqi();
            let rssi = lqi as _; // TODO: Convert LQI to RSSI

            break Ok(PsduMeta {
                // TODO: `embassy-nrf` driver provides the PSDU without the CRC,
                // however, OpenThread wants the PSDU len to include the CRC
                len: len + 2,
                channel,
                rssi: Some(rssi),
            });
        }
    }
}
