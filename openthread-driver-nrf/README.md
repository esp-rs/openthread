# openthread-driver-nrf

The IEEE 802.15.4 radio of the nRF chips for the [`openthread`](../openthread) crate: `NrfRadio`, an implementation of the `Radio` trait of [`openthread-driver`](../openthread-driver), over the [`nrf-802154`](https://github.com/sysgrok/nrf-802154) driver.

The driver does the time-critical part of the IEEE 802.15.4 MAC in its interrupt handlers - the ACKs, the address filtering, the queue of received frames - as the `Radio` trait requires. It also supports what a CSL (Synchronized Sleepy) End Device and its parent need: timed receive and transmit, frame timestamps, and enhanced ACKs secured with the CSL IE.

## Usage

Select the chip with its feature (`nrf52840`, `nrf54l15-app-s`, ...), here and on `nrf-802154`:

```toml
nrf-802154 = { version = "0.2", features = ["nrf52840"] }
openthread-driver-nrf = { version = "0.1", features = ["nrf52840"] }
```

Then wrap the `nrf-802154` radio and hand it to `OpenThread`:

```rust,ignore
use openthread_driver_nrf::NrfRadio;

let mut radio = NrfRadio::new(nrf_802154::Radio::new(/* ... */));
let radio_caps = radio.init().await?;

// ... `OpenThread::new` with `radio_caps`, then:
ot.run(radio).await
```

The `defmt` and `log` features select how the crate logs.

The examples are in [`examples/nrf`](../examples/nrf).
