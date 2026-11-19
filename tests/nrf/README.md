# openthread-tests-nrf

The [`cli_node`](src/bin/cli_node.rs) binary for the nRF52840 and nRF54L15 MCUs, with the `NrfRadio` of [`openthread-driver-nrf`](../../openthread-driver-nrf) over the [`nrf-802154`](https://github.com/sysgrok/nrf-802154) driver.

A separate project due to the need to use nRF-specific crates (`embassy-nrf` and others).

See the [`openthread-tests`](../README.md) umbrella project for more information on the E2E test harness.

## Building and flashing

The console is a UART by default: on the nRF52840-DK, the J-Link virtual COM port. Other options:
- `console-usb`: the board's own USB peripheral, for a board whose only serial port is USB (a XIAO nRF52840 plugged into the host, an nRF52840 dongle). nRF52840 only.
- `console-uart-xiao`: the UART on the D6/D7 pads of a XIAO nRF52840, for a XIAO wired to the UART bridge of a debug probe.

```sh
cd tests/nrf

cargo build --release
probe-rs download --chip nRF52840_xxAA target/thumbv7em-none-eabi/release/cli_node
probe-rs reset --chip nRF52840_xxAA

# or, with the .cargo/config.toml runner (flash + RTT):
cargo run --release
```

For the nRF54L15, select its feature and target (and the `probe-rs` chip in `.cargo/config.toml`):

```sh
cargo run --release --no-default-features --features nrf54l15 --target thumbv8m.main-none-eabihf
```

Until `nrf-802154` 0.2 is released, `Cargo.toml` takes it from git.

## Running the suites against it

The board is `mcu` in the port map; pairing it against a **known-good RCP
node** isolates a failure to the device:

```sh
cargo xtask itest \
  --hw-port /dev/serial/by-id/<this-board>=mcu \
  --hw-port /dev/serial/by-id/<ot-rcp-dongle>=rcp
```

Before a full run, check the board answers at all: open the console with any
terminal program and type `state`.
