//! The IEEE 802.15.4 radio of the nRF chips for the `openthread` crate: [`NrfRadio`], an
//! implementation of the `Radio` trait of `openthread-driver`, over the `nrf-802154` driver.
//!
//! Select the chip with its feature (`nrf52840`, `nrf54l15-app-s`, ...). Without one, the crate
//! does not build for bare-metal targets, and is empty on the host.

#![no_std]

// On the host, the crate is empty without a chip, so that the workspace and the publish check build
#[cfg(all(target_os = "none", not(feature = "nrf-802154")))]
compile_error!(
    "openthread-driver-nrf: select the chip with one of the features nrf52805, nrf52810, \
     nrf52811, nrf52820, nrf52832, nrf52833, nrf52840, nrf5340-net, nrf54l05-app-s, \
     nrf54l10-app-s, nrf54l10-app-ns, nrf54l15-app-s, nrf54l15-app-ns, nrf54lm20-app-s"
);

#[cfg(feature = "nrf-802154")]
pub use radio::*;

mod fmt;
#[cfg(feature = "nrf-802154")]
mod radio;
