# openthread

Platform-agnostic, async Rust bindings for the [`OpenThread`](https://openthread.io/) library.

Tailored for Rust embedded baremetal.

The crate does not depend on any platform features and only needs an implementation of the `Radio` trait, which represents the IEEE 802.15.4 radio. The trait is in the [`openthread-driver`](../openthread-driver) crate, so that radio drivers do not need to depend on this crate.
The radio might be located on the same die, or the user might provide an implementation that communicates with the actual radio over UART, SPI, USB, etc.

These IEEE 802.15.4 radios are supported (others could just implement the `Radio` trait):
- The ESP32-C5, ESP32-C6, ESP32-H2 and ESP32-S31 radio: [`openthread-driver-esp`](../openthread-driver-esp)
- The nRF52, nRF53 and nRF54 radio: [`openthread-driver-nrf`](../openthread-driver-nrf), over the [`nrf-802154`](https://github.com/sysgrok/nrf-802154) driver
- A radio co-processor running OpenThread's RCP firmware, over the spinel protocol: `SpinelRadio`, in this crate (enable the `rcp` feature)

## Build

For certain MCUs / Rust targets, the OpenThread libraries are pre-compiled for convenience.
Current list (might be extended upon request):
- `riscv32imac-unknown-none-elf` (ESP32C6 and ESP32H2)
- `thumbv7em-none-eabi` (NRF52, with `nrf-802154`)
- `thumbv6m-none-eabi` No chip with IEEE802.15.4 radio on this target (to our knowledge), but can be used with `openthread` RCP (radio offloading)

**For these targets you only need `rustc`/`cargo` as usual!**

Small caveat: since `openthread` does a few calls into the C standard library (primarily `str*` functions), at link time, it is up to the user to poly-fill the `str*` syscalls - either with the MCU ROM functions, or by depending on [`tinyrlibc`](https://github.com/rust-embedded-community/tinyrlibc), or with both.

### Build for other targets / custom build

For targets where pre-compiled libs are not available (including for the Host itself) - or if you using the library with non-default features - a standard `build.rs` build is also supported.
For the on-the-fly OpenThread CMake build to work, you'll need to install and set in your `$PATH`:
- Recent Clang (for Espressif `xtensa`, [it must be the Espressif fork](https://crates.io/crates/espup), but for all other chips, the stock Clang would work)
- CMake and Ninja
- On macOS: the `llvm-tools` rustup component (`rustup component add llvm-tools`), or a non-Apple Clang such as Homebrew's LLVM first on `$PATH` - Apple's `ar` silently drops the ELF objects of the C++ library

## Features

- Thread 1.4 compatibility
- MTD (Minimal Thread Device) functionality
- FTD (Full Thread Device) functionality
- SED functionality
- CSL (Synchronized Sleepy End Device) functionality: an FTD can always parent CSL children; the CSL child side needs the `csl-receiver` feature
- Support for the Spinel protocol and 802.15.4 radio offload via Thread RCP
- CLI API
- Optional integration with `embassy-net`
- Radio drivers for the ESP32 chips (`openthread-driver-esp`) and the nRF chips (`openthread-driver-nrf`)

## Non-Goals

- Thread Border Router functionality

## Status

- Upstream E2E and expect-framework tests running as part of CI (with virtual time); HIL tests pass with `EspRadio` and `NrfRadio`
- The examples (native OpenThread UDP sockets; `embassy-net` integration; SRP) build and run on Espressif MCUs, and on the NRF52840 / NRF54L10/15
- `rs-matter` - the pure-Rust Matter stack - does run with Thread as the operational network, powered by this library
