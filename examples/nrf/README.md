# `openthread` nRF examples

These examples run the full OpenThread stack on the nRF52840 and nRF54L chips, with the radio of the `openthread-driver-nrf` crate (`NrfRadio`) over the [`nrf-802154`](https://github.com/sysgrok/nrf-802154) driver.

- `basic_udp`: a native OpenThread UDP socket
- `basic_enet`: the `embassy-net` integration
- `srp`: an SRP service registration
- `srp_ble_coex`: the same, with BLE running next to Thread (nRF52840 only)

## Build and run

The nRF52840 is the default chip:

```sh
cargo run --release --bin basic_udp
```

For the nRF54L15, select its feature and target (and the `probe-rs` chip in `.cargo/config.toml`):

```sh
cargo run --release --bin basic_udp --no-default-features --features nrf54l15 --target thumbv8m.main-none-eabihf
```
