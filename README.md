# openthread

[![CI](https://github.com/esp-rs/openthread/actions/workflows/ci.yml/badge.svg)](https://github.com/esp-rs/openthread/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/openthread.svg)](https://crates.io/crates/openthread)
[![Documentation](https://img.shields.io/badge/docs-esp--rs-brightgreen)](https://esp-rs.github.io/esp-rs/openthread/index.html)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&color=BEC5C9&logo=matrix)](https://matrix.to/#/#esp-rs:matrix.org)

This repository hosts these crates:
- [`openthread`](openthread): platform-agnostic, async Rust bindings for OpenThread
- [`openthread-sys`](openthread-sys): raw bindings for the OpenThread C library
- [`openthread-driver`](openthread-driver): the traits a platform implements for `openthread` - the IEEE 802.15.4 radio and the settings storage
- [`openthread-driver-esp`](openthread-driver-esp): the radio of the ESP32-C5, ESP32-C6, ESP32-H2 and ESP32-S31, over `esp-radio`
- [`openthread-driver-nrf`](openthread-driver-nrf): the radio of the nRF52, nRF53 and nRF54, over [`nrf-802154`](https://github.com/sysgrok/nrf-802154)

It also hosts:
- [examples](examples): for the ESP32 chips ([`esp`](examples/esp)), the nRF chips ([`nrf`](examples/nrf)), and a host that drives a radio co-processor over serial ([`std`](examples/std))
- [tests](tests): the integration tests, which run the upstream OpenThread test suites
- [xtask](xtask): the repository automation (`cargo xtask`)

For more information, consult the README of each crate.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
