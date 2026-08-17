# openthread-tests-esp

The [`cli_node`](src/bin/cli_node.rs) binary for the 802.15.4-capable ESP32XX MCUs, utilizing the `EspRadio` radio driver.

A separate project due to the need to use esp32-specific crates (`esp-hal`, `esp-radio` and so on).

See the [`openthread-tests`](../README.md) umbrella project for more information on the E2E test harness.

## Building and flashing

```sh
cd tests/esp

# Flash + open a monitor (the .cargo/config.toml runner):
cargo run --release

# Or flash without a monitor - the shape the automated loop uses:
cargo build --release
espflash flash --port /dev/ttyACM0 \
  target/riscv32imac-unknown-none-elf/release/cli_node
```

For an ESP32-H2: `cargo build --release --no-default-features --features esp32h2`.

## Console and logs

The CLI console and the log output share the USB-Serial-JTAG channel, and the
harness *parses* the console - so test builds compile logs out (`ESP_LOG=off`
in `.cargo/config.toml`; the level is baked in at build time). Panics still
print there via `esp-backtrace`: a dying node's last words end up in the
failing test's log tail, which is where you want them.

For interactive debugging, rebuild with logs on and talk to the node by hand:

```sh
ESP_LOG=debug cargo run --release     # flash + monitor, logs interleaved
```

## Running the suites against it

The board is `mcu` in the port map; pair it against a **known-good
reference** so a failure points at the device - an `ot-rcp` dongle driven by
this crate's host node (`=rcp`) or by the upstream posix host (`=cposix`):

```sh
cargo xtask itest \
  --hw-port /dev/serial/by-id/<this-board>=mcu \
  --hw-port /dev/serial/by-id/<ot-rcp-dongle>=rcp
```

Before a full run, check the board answers at all: open the console with any
terminal program and type `state`.
