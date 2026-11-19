# openthread-driver

The traits a platform implements for the [`openthread`](../openthread) crate:
- `Radio`: the IEEE 802.15.4 radio
- `Settings`: the storage of OpenThread's settings

Radio drivers and settings stores depend on this crate only, rather than on `openthread`. That way they do not build the OpenThread C library, and they do not need a new release for every release of `openthread`. The `openthread` crate re-exports everything in this crate.

The `Radio` trait describes a radio that does the time-critical part of the IEEE 802.15.4 MAC itself: it sends and receives the ACKs, filters the frames by PAN ID and address, and receives in the background into a queue. See the trait documentation, and [the radio contract](../docs/radio-contract.md), for what OpenThread expects from a radio.

Implementations:
- [`openthread-driver-esp`](../openthread-driver-esp): the radio of the ESP32-C5, ESP32-C6, ESP32-H2 and ESP32-S31
- [`openthread-driver-nrf`](../openthread-driver-nrf): the radio of the nRF52, nRF53 and nRF54, over the [`nrf-802154`](https://github.com/sysgrok/nrf-802154) driver
- `SpinelRadio` in `openthread`: a radio co-processor running OpenThread's RCP firmware
