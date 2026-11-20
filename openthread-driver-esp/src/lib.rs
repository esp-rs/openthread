//! The IEEE 802.15.4 radio of the ESP32 chips (ESP32-C5, ESP32-C6, ESP32-H2 and ESP32-S31) for
//! the `openthread` crate, over `esp-radio`: [`EspRadio`], an `openthread_driver::Radio`.
//!
//! Select the chip with its feature (`esp32c6`, ...). Without one, the crate does not build for
//! bare-metal targets, and is empty on the host.

#![no_std]
#![allow(async_fn_in_trait)]

// On the host, the crate is empty without a chip, so that the workspace and the publish check build
#[cfg(all(
    target_os = "none",
    not(any(
        feature = "esp32c5",
        feature = "esp32c6",
        feature = "esp32h2",
        feature = "esp32s31"
    ))
))]
compile_error!(
    "openthread-driver-esp: select the chip with one of the features esp32c5, esp32c6, esp32h2, \
     esp32s31"
);

#[cfg(feature = "esp-radio")]
pub use radio::*;

mod fmt;
#[cfg(feature = "esp-radio")]
mod radio;
