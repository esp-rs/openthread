# openthread-tests-nrf

The [`cli_node`](src/bin/cli_node.rs) binary for the NRF52840 MCU, utilizing the `NrfRadio` radio driver.

A separate project due to the need to use nrf-specific crates (`embassy-nrf` and others).

See the [`openthread-tests`](../README.md) umbrella project for more information on the E2E test harness.

## Building and flashing

```sh
cd tests/nrf

cargo build --release --features console-usb
probe-rs download --chip nRF52840_xxAA target/thumbv7em-none-eabi/release/cli_node
probe-rs reset --chip nRF52840_xxAA

# or, with the .cargo/config.toml runner (flash + RTT):
cargo run --release --features console-usb
```

Boards with the Adafruit UF2 bootloader can be flashed probe-less instead:
build with `--features console-usb,uf2` (links [`memory-uf2.x`](memory-uf2.x)
so the application starts above the bootloader/SoftDevice), convert the ELF
to `.uf2` (`cargo objcopy -- -O ihex` + `uf2conv --family 0xADA52840`), and
copy it onto the bootloader's mass-storage drive. Check the flash origin
against your particular bootloader before trusting it.

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
