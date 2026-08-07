# The nRF firmware node (hardware-in-the-loop tier)

The `openthread` stack running as **firmware on an nRF52840**, driven by the
unmodified upstream OpenThread test harness through the board's serial console.

## Status

**Compile-verified only.** This has never been flashed or run. Expect the first
real run to find things.

Two known gaps:

- **Settings are RAM-backed**, so the CLI `reset` loses the dataset. Scenarios
  that reset a node will fail until flash-backed settings land.
- **Nothing here has been flashed**, so every board note below is reasoned
  from datasheets rather than observed.

## Boards

The console is where the harness talks to the node, so the board decides how
this firmware has to be built.

**nRF52840-DK** — the default build. `UARTE0` on the DK's J-Link virtual COM
port (P0.06 TX, P0.08 RX, 115200), which enumerates on the host as
`/dev/ttyACM*`. Same for any board where you wire a USB-serial adapter to those
two pins.

**XIAO nRF52840 / nRF52840 dongle** — `--features console-usb,uf2`. Neither has
a UART-to-USB bridge: their serial port is the nRF's own USB peripheral, so the
console is USB CDC, and the board enumerates as `/dev/ttyACM*` by itself. Both
also carry a UF2 bootloader, hence `uf2` for the flash layout.

`defmt` logs go out over RTT in either case, so they never touch the console
the harness parses.

| Board | Features |
|---|---|
| nRF52840-DK (or a UART adapter on P0.06/P0.08) | *(none)* |
| XIAO nRF52840, nRF52840 dongle | `console-usb,uf2` |

## Building and flashing

### With a debug probe

```sh
cd tests/nrf

# Flash and run (the .cargo/config.toml runner is `probe-rs run`).
cargo run --release

# Or just build, then download the ELF.
cargo build --release
probe-rs download --chip nRF52840_xxAA target/thumbv7em-none-eabi/release/cli_ftd
```

The DK has its debugger on board. A XIAO does not: SWDIO/SWCLK come out on test
pads, so this route means soldering and an external probe.

### With the UF2 bootloader (no probe)

Boards that ship the Adafruit nRF52 bootloader — the XIAO nRF52840 among them —
take firmware over USB mass storage. **Double-tap the reset button**: the board
re-enumerates as a small drive (`XIAO-SENSE`, `NRF52BOOT` or similar), and
copying a `.uf2` onto it flashes and reboots.

Cargo produces an ELF, so convert it:

```sh
cargo build --release --features console-usb,uf2

# ELF -> hex (cargo-binutils, or arm-none-eabi-objcopy)
cargo objcopy --release -- -O ihex cli_ftd.hex

# hex -> uf2, with the nRF52840 family id
uf2conv cli_ftd.hex --family 0xADA52840 --output cli_ftd.uf2

cp cli_ftd.uf2 /media/$USER/XIAO-SENSE/
```

`uf2conv` is `pip install uf2utils` or the `uf2conv.py` script from Microsoft's
`uf2` repository; `cargo objcopy` is `cargo install cargo-binutils` plus
`rustup component add llvm-tools`.

The `uf2` feature is what keeps the firmware off the bootloader's toes: it
links against [`memory-uf2.x`](memory-uf2.x) (`FLASH ORIGIN = 0x00027000`,
868K) instead of owning all of flash. We never *use* the SoftDevice sitting
below that address — we only start above it and leave it intact, which costs
156K of flash and no RAM (a SoftDevice reserves RAM only once an application
enables it, and this one never does).

**Check the origin against your board.** `0x27000` is the Adafruit bootloader
with S140 7.x; older ones use `0x26000`, and a bootloader built without a
SoftDevice starts the application at `0x1000`. Flashing at the wrong origin is
how a bootloader gets overwritten.

## Running the suites against it

The board is `mcu` in the port map; give it a peer to talk to. Pairing the
device under test against a **known-good RCP node** is what isolates a failure
to the device, so the usual rig is one of each:

```sh
# node 1 = this firmware, node 2 = an ot-rcp dongle driven by the host DUT
cargo xtask itest \
  --hw-port /dev/ttyACM0=mcu \
  --hw-port /dev/ttyACM1=rcp
```

The harness spawns one binary for every node; `cli_ftd` looks at the port map
and hands the `mcu` nodes over to `serial_bridge`, which pipes the harness's
stdin/stdout to the board's console. Nothing above notices the difference.

Before a full run, check the board answers at all:

```sh
# The bridge resets the board and waits for its prompt; talking to it by hand
# is the quickest way to see whether the firmware is alive.
picocom -b 115200 /dev/ttyACM0     # then type `state`
```
