# openthread

Platform-agnostic, async Rust bindings for the [`OpenThread`](https://openthread.io/) library.

Tailored for Rust embedded baremetal.

The crate does not depend on any platform features and only needs an implementation of a single trait - [`Radio`](openthread/src/radio.rs) - that represents the IEEE 802.15.4 PHY radio. 
The radio might be located on the same die, or the user might provide an implementation that communicates with the actual radio over UART, SPI, USB, etc.

Two IEEE 802.15.4 radios are supported in-crate (others could just implement the `Radio` trait):
- The ESP32C6 and ESP32H2 radio (enable the `esp-ieee802154` feature);
- The NRF radio (enable the `embasy-nrf` feature).

## Build

For certain MCUs / Rust targets, the OpenThread libraries are pre-compiled for convenience.
Current list (might be extended upon request):
- `riscv32imac-unknown-none-elf` (ESP32C6 and ESP32H2)
- `thumbv7em-none-eabi` (NRF52)
- `thumbv6m-none-eabi` No chip with IEEE802.15.4 radio on this target (to our knowledge), but can be used with `openthread` RCP (radio offloading)

**For these targets you only need `rustc`/`cargo` as usual!**

Small caveat: since `openthread` does a few calls into the C standard library (primarily `str*` functions), at link time, it is up to the user to poly-fill the `str*` syscalls - either with the MCU ROM functions, or by depending on [`tinyrlibc`](https://github.com/rust-embedded-community/tinyrlibc), or with both.

### Build for other targets / custom build

For targets where pre-compiled libs are not available (including for the Host itself) - or if you using the library with non-default features - a standard `build.rs` build is also supported.
For the on-the-fly OpenThread CMake build to work, you'll need to install and set in your `$PATH`:
- Recent Clang (for Espressif `xtensa`, [it must be the Espressif fork](https://crates.io/crates/espup), but for all other chips, the stock Clang would work)
- CMake and Ninja

## Features

- MTD (Minimal Thread Device) functionality
- FTD (Full Thread Device) functionality
- Optional integration with `embassy-net`
- Out of the box support for the IEEE 802.15.4 radio in Espressif's `esp-hal` and the NRF52 radio in `embassy-nrf`.

## Next

- Sleepy end-device
- Support for the Spinel protocol and 802.15.4 rasdio oflload via Thread RCP

## Non-Goals

- Thread Border Router functionality

## Status

- The examples (native OpenThread UDP sockets; `embassy-net` integration; SRP) build and run on Espressif MCUs, and on the NRF52840.
- `rs-matter` - the pure-Rust Matter stack)- does run with Thread as the operational network, powered by this library 
