# The ESP32-C6/H2 firmware node (hardware-in-the-loop tier)

The `openthread` stack running as **firmware on an ESP32-C6** (or -H2), driven
by the unmodified upstream OpenThread test harness through the chip's
USB-Serial-JTAG console.

This node exercises `EspRadio` on the chip's IEEE 802.15.4 peripheral under
the real test suites. Unlike the nRF node there is no software MAC in the
picture — `esp-radio` offloads the whole MAC set (bar source matching) — so
the radio runs straight in `OpenThread::run`. It is also the `log`-backend
node (the nRF node uses `defmt`), so the two MCU flavors cover both logging
paths of the crate.

## Status

**Compile-verified only.** Never flashed or run under the harness. Expect the
first sessions to find things — that is the tier's purpose.

Known gap: **settings are RAM-backed**, so the CLI `reset` loses the dataset
(a chip reboot is factory-fresh — correct for `factoryreset`, lossy for
`reset`). Scenarios that reset a node and expect it to rejoin stay off this
tier until flash-backed settings land.

## Why this is the automation-friendly board

One USB port does everything: `espflash` resets the chip into download mode
over USB-Serial-JTAG (no buttons), flashes, resets into the app — and the same
port then carries the CLI console (`/dev/ttyACM*`). A flash → test → fix →
re-flash loop needs no human hands.

## Building and flashing

```sh
cd tests/esp

# Flash + open a monitor (the .cargo/config.toml runner):
cargo run --release

# Or flash without a monitor - the shape the automated loop uses:
cargo build --release
espflash flash --chip esp32c6 --port /dev/ttyACM0 \
  target/riscv32imac-unknown-none-elf/release/cli_node
```

For an ESP32-H2: `cargo build --release --no-default-features --features esp32h2`.

## Console and logs

The CLI console and the log output share the USB-Serial-JTAG channel, and the
harness *parses* the console — so test builds compile logs out (`ESP_LOG=off`
in `.cargo/config.toml`; the level is baked in at build time). Panics still
print there via `esp-backtrace`: a dying node's last words end up in the
failing test's log tail, which is where you want them.

For interactive debugging, rebuild with logs on and talk to the node by hand:

```sh
ESP_LOG=debug cargo run --release     # flash + monitor, logs interleaved
```

## Running the suites against it

The board is `mcu` in the port map; pair it against a **known-good reference**
so a failure points at the device. With a second dongle running stock `ot-rcp`,
the strongest rig drives that dongle with the *upstream posix host* (`cposix`):

```sh
# node 1 = this firmware, node 2 = upstream posix host + ot-rcp dongle
cargo xtask itest \
  --hw-port /dev/ttyACM2=mcu \
  --hw-port /dev/ttyACM1=cposix \
  Cert_5_1_01_RouterAttach
```

(`=rcp` instead of `=cposix` pairs it against this crate's own host node.)

Before a full run, check the board answers at all:

```sh
picocom -q /dev/ttyACM2      # then type `state` + enter
```
