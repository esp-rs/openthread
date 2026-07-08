# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased
* The default MbedTLS backend is now OpenThread's own **bundled** MbedTLS, not the external `mbedtls-rs-sys` crate. A default-features build reuses the committed prebuilt libraries and needs **no C toolchain** (clang/cmake/ninja).
  * To build OpenThread against the external `mbedtls-rs-sys` instead, enable the `mbedtls-rs-sys` feature. Do this when another crate in the graph already provides `mbedtls-rs-sys` (e.g. `rs-matter`), so a single MbedTLS serves both, or when you need the HW accel capabilities of `mbedtls-rs-sys`.
  * WARNING: do not combine a default (bundled-MbedTLS) OpenThread with a separate `mbedtls-rs-sys` in the same firmware — that links two MbedTLS copies. If your graph needs `mbedtls-rs-sys`, enable this feature so OpenThread reuses it.
* Remote Radio Support (Spinel-RCP) (#98)
* Streamline several features (#97):
  * Put dns-client as part of the default / matter feature

## [0.2.1] - 2026-06-25
* Initial release
