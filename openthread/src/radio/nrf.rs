//! `Radio` trait implementation for the `embassy-nrf` ESP IEEE 802.15.4 radio.

pub use embassy_nrf::radio::ieee802154::{Cca as RadioCca, Packet};

use crate::fmt::Bytes;
use crate::{Config, PsduMeta, Radio, RadioCaps, RadioError, RadioErrorKind, SrcMatchConfig};

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
    /// The channel the driver is currently on (commanded by `set_receive` or
    /// by a `transmit`); also stamped onto received frames' metadata.
    channel: u8,
}

impl<'a> NrfRadio<'a> {
    const DEFAULT_CONFIG: Config = Config::new();
    /// The channel the driver starts on, until the stack commands another.
    const DEFAULT_CHANNEL: u8 = 11;

    /// Create a new `EspRadio` instance.
    pub fn new(radio: Ieee802154<'a>) -> Self {
        let mut this = Self {
            driver: radio,
            config: Self::DEFAULT_CONFIG,
            channel: Self::DEFAULT_CHANNEL,
        };

        this.driver.set_channel(Self::DEFAULT_CHANNEL);
        this.driver
            .set_transmission_power(Self::clamp_tx_power(RadioCaps::DEFAULT_TX_POWER));

        this
    }

    /// Put the driver on `channel`, remembering it for the metadata of
    /// received frames.
    fn set_driver_channel(&mut self, channel: u8) {
        if self.channel != channel {
            self.channel = channel;
            self.driver.set_channel(channel);
        }
    }

    /// Snap a requested transmit power (in dBm) to a value the nRF radio's
    /// `set_transmission_power` accepts.
    ///
    /// `Config::power` is a cross-platform dBm value.
    /// The nRF radio however only supports a discrete set of dBm levels with
    /// a much lower ceiling (+8 dBm on the nRF52840), and `embassy-nrf`'s
    /// `set_transmission_power` *panics* on any value not in that set. So map the
    /// request to the highest supported level not exceeding it (clamping to the
    /// min/max of the supported range), which both avoids the panic and applies
    /// the closest power the radio can actually produce.
    fn clamp_tx_power(power: i8) -> i8 {
        // The dBm levels `embassy-nrf` accepts for the nRF52840, descending.
        // Other nRF variants support a subset (e.g. the nRF52811 / nRF5340
        // network core drop the higher positive levels), but these are the ones
        // this driver targets. Keep in sync with `embassy_nrf`'s
        // `Radio::set_transmission_power`.
        //
        // TODO: This table is nRF52840-specific. If/when this driver targets
        // other nRF variants (nRF52811, nRF5340 net core, ...), gate it by chip
        // `cfg` to match `embassy_nrf`'s own per-chip `match` arms (the higher
        // positive levels are unavailable on some, and the 5340 net core adds
        // extra negative levels).
        const SUPPORTED_DBM: [i8; 15] = [8, 7, 6, 5, 4, 3, 2, 0, -4, -8, -12, -16, -20, -30, -40];

        // Highest supported level <= requested power; if the request is below the
        // minimum, use the minimum.
        SUPPORTED_DBM
            .into_iter()
            .find(|&level| level <= power)
            .unwrap_or(SUPPORTED_DBM[SUPPORTED_DBM.len() - 1])
    }
}

impl Radio for NrfRadio<'_> {
    type Error = Error;

    async fn init(&mut self) -> Result<RadioCaps, Self::Error> {
        // The nRF radio has no MAC offloading capabilities of its own;
        // OpenThread / `MacRadio` handle everything in software. (This includes
        // address filtering, so `Config::alt_short_addr` — the alternate short
        // address an FTD accepts during a child-to-router transition — is honored
        // here for free: the `MacRadio` software filter accepts both the primary
        // and the alternate. Radios that offload short-address filtering to
        // hardware/firmware only honor the alternate if that layer supports a
        // second address; this one always does, in software.)
        //
        // No `ENERGY_SCAN` (and no `Radio::energy_scan` impl) either: the nRF
        // RADIO peripheral can sample channel energy (EDSAMPLE), but
        // `embassy-nrf`'s IEEE 802.15.4 driver does not expose it (as of 0.10;
        // energy detection is only used internally as a CCA mode). Until it
        // does, energy scans on this radio yield no measurements (see
        // `Radio::energy_scan`).
        // TODO: Report the hardware's real receive sensitivity.
        Ok(RadioCaps::default())
    }

    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error> {
        // Nothing to apply: everything this driver needs (channel, power, CCA)
        // now arrives with the operation, and the MAC-level policy in `Config`
        // (filtering, promiscuous mode) is emulated above by `MacRadio` - this
        // is a bare PHY.
        if self.config != *config {
            trace!("Setting radio config: {:?}", config);

            self.config = config.clone();
        }

        Ok(())
    }

    async fn set_src_match_config(&mut self, _config: &SrcMatchConfig) -> Result<(), Self::Error> {
        // No RX-ACK offload, so nothing here answers data polls: `MacRadio`
        // keeps the table and decides the Frame Pending bit in software.
        Ok(())
    }

    async fn set_receive(&mut self, channel: u8) -> Result<(), Self::Error> {
        // Only the channel: this driver has no free-running RX path, it
        // receives exactly while `receive` is being polled.
        self.set_driver_channel(channel);

        Ok(())
    }

    async fn set_sleep(&mut self) -> Result<(), Self::Error> {
        // Nothing to power down through `embassy-nrf`'s driver, and nothing
        // queues up while parked (see `set_receive`).
        Ok(())
    }

    async fn transmit(
        &mut self,
        psdu: &[u8],
        channel: u8,
        power: i8,
        cca_threshold: Option<i8>,
        _ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Self::Error> {
        trace!("NRF Radio, about to transmit: {}", Bytes(psdu));

        self.set_driver_channel(channel);
        self.driver
            .set_transmission_power(Self::clamp_tx_power(power));

        // CCA mode: carrier sense whenever the stack asks for CCA at all.
        //
        // TODO: honor the requested threshold. It arrives in dBm (the unit of
        // OpenThread's `otPlatRadioSetCcaEnergyDetectThreshold`), while
        // `embassy-nrf`'s `EnergyDetection { ed_threshold }` takes the raw
        // 0..0xFF ED-sample value of the nRF hardware - "not given in dBm, but
        // can be converted", per the nRF52840 Product Specification section
        // 6.20.12.4. Until that conversion is written down, carrier sense is
        // used, which needs no threshold (and is what this driver did before
        // the threshold was plumbed at all). `None` (transmit without CCA)
        // cannot be honored either: `try_send` is the driver's only transmit
        // and it always performs CCA.
        let _ = cca_threshold;
        self.driver.set_cca(RadioCca::CarrierSense);

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

        // `ED_RSSIOFFS`, offset for converting the radio's LQI energy reading to dBm.
        // nRF52/53/54 Product Specifications report -92 or -93.
        const ED_RSSI_OFFSET: i8 = -93;

        let channel = self.channel;

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

            let rssi = ED_RSSI_OFFSET.saturating_add_unsigned(packet.lqi());

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
