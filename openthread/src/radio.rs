//! The IEEE 802.15.4 radio: the `Radio` trait and its types (from `openthread-driver`), and the
//! spinel (RCP) radio.

#![allow(clippy::unnecessary_cast)]

pub use openthread_driver::radio::*;

use crate::sys::{
    OT_RADIO_CAPS_ACK_TIMEOUT, OT_RADIO_CAPS_ALT_SHORT_ADDR, OT_RADIO_CAPS_CSMA_BACKOFF,
    OT_RADIO_CAPS_ENERGY_SCAN, OT_RADIO_CAPS_RECEIVE_TIMING, OT_RADIO_CAPS_RX_ON_WHEN_IDLE,
    OT_RADIO_CAPS_SLEEP_TO_TX, OT_RADIO_CAPS_TRANSMIT_FRAME_POWER, OT_RADIO_CAPS_TRANSMIT_RETRIES,
    OT_RADIO_CAPS_TRANSMIT_SEC, OT_RADIO_CAPS_TRANSMIT_TIMING, OT_RADIO_RSSI_INVALID,
};

// The [`Radio`] implementation of this crate, `spinel`: the radio lives on a
// *separate* chip (an OpenThread RCP) reached over a UART/SPI spinel link - an
// RCP-host deployment. (The feature is named `rcp` after that deployment role;
// the module is named `spinel` after the wire protocol it speaks.) Radios local
// to the MCU are in their own crates, such as `openthread-driver-esp`.
#[cfg(feature = "rcp")]
pub mod spinel;

// `openthread-driver` defines these with OpenThread's values
const _: () = {
    core::assert!(RSSI_INVALID as i32 == OT_RADIO_RSSI_INVALID as i32);

    let caps = [
        (Capabilities::ACK_TIMEOUT, OT_RADIO_CAPS_ACK_TIMEOUT),
        (Capabilities::ENERGY_SCAN, OT_RADIO_CAPS_ENERGY_SCAN),
        (
            Capabilities::TRANSMIT_RETRIES,
            OT_RADIO_CAPS_TRANSMIT_RETRIES,
        ),
        (Capabilities::CSMA_BACKOFF, OT_RADIO_CAPS_CSMA_BACKOFF),
        (Capabilities::SLEEP_TO_TX, OT_RADIO_CAPS_SLEEP_TO_TX),
        (Capabilities::TRANSMIT_SEC, OT_RADIO_CAPS_TRANSMIT_SEC),
        (Capabilities::TRANSMIT_TIMING, OT_RADIO_CAPS_TRANSMIT_TIMING),
        (Capabilities::RECEIVE_TIMING, OT_RADIO_CAPS_RECEIVE_TIMING),
        (Capabilities::AUTO_SLEEP, OT_RADIO_CAPS_RX_ON_WHEN_IDLE),
        (
            Capabilities::TRANSMIT_FRAME_POWER,
            OT_RADIO_CAPS_TRANSMIT_FRAME_POWER,
        ),
        (Capabilities::ALT_SHORT_ADDR, OT_RADIO_CAPS_ALT_SHORT_ADDR),
    ];

    let mut i = 0;
    while i < caps.len() {
        core::assert!(caps[i].0.bits() as u32 == caps[i].1 as u32);
        i += 1;
    }
};

/// Finish a frame (or an enhanced ACK) going out at `now` with OpenThread's
/// `otMacFrameProcessTxSfd`, but with its CSL phase rounded to the nearest
/// 160 us unit rather than down.
///
/// Rounded down, the phase has the parent transmit up to a whole unit early -
/// more than the CSL receive windows open ahead of the sample time in this
/// crate's builds (see `openthread-sys`). Rounded to the nearest unit, it is at
/// most half a unit off: early within the window, or late by less than the
/// synchronization header the radio detects in it.
#[cfg(feature = "_csl")]
pub(crate) fn process_tx_sfd(
    frame: &mut crate::sys::otRadioFrame,
    now: u64,
    ctx: &mut crate::sys::otRadioContext,
) -> crate::sys::otError {
    const HALF_PHASE_UNIT_US: u64 = 10 * 16 / 2;

    let err = unsafe {
        crate::sys::otMacFrameProcessTxSfd(frame, now.wrapping_sub(HALF_PHASE_UNIT_US), ctx)
    };

    // The helper also stamps the frame with the time it was handed.
    frame.mInfo.mTxInfo.mTimestamp = now;

    err
}
