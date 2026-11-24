# openthread-driver-esp

The IEEE 802.15.4 radio of the ESP32 chips for the [`openthread`](../openthread) crate: `EspRadio`, an implementation of the `Radio` trait of [`openthread-driver`](../openthread-driver), over `esp-radio`.

Select the chip with one of the `esp32c5`, `esp32c6`, `esp32h2` or `esp32s31` features:

```toml
openthread-driver-esp = { version = "0.1", features = ["esp32c6"] }
```

Then hand the radio to `OpenThread`:

```rust,ignore
use esp_radio::ieee802154::Ieee802154;
use openthread_driver_esp::EspRadio;

let mut radio = EspRadio::new(Ieee802154::new(peripherals.IEEE802154));
let radio_caps = radio.init().await?;

// ... `OpenThread::new` with `radio_caps`, then:
ot.run(radio).await
```

The `log` and `defmt` features select how the driver logs.

Notes:
- `esp-radio` does not say why a transmission failed, so every failure is reported to OpenThread as a channel access failure, never as a missing ACK.
- `esp-radio` has no source match table, and cannot stop the receiver, so the source match configuration and `set_sleep` are ignored.
