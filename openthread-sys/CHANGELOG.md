# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased
* Advertise **Thread 1.4** instead of Thread 1.1 (#103)
  * The stack now reports Thread version 1.4. Note that Thread 1.3+ is the floor Matter-over-Thread expects; for a plain node (no Border Router, no TREL) the on-air/radio contract is unchanged past 1.2, so this is effectively a version bump plus a few benign internal behaviors (e.g. a more thorough parent search at attach).
  * CSL is deliberately **not** compiled in: both `OPENTHREAD_CONFIG_MAC_CSL_TRANSMITTER_ENABLE` (which otherwise defaults on at >= 1.2) and `OT_CSL_RECEIVER` are forced off. This keeps the radio-platform contract identical to 1.1 — no `EnableCsl` / `ReceiveAt` / `GetCslAccuracy` callbacks are referenced — so every existing `Radio` driver keeps working unchanged. Low-power CSL (SSED) remains a future opt-in.
  * HW-validated over an RCP: attach, SRP registration, and a `ping-stress` datapath sweep against a live Border Router.
* NRF52: Fix a corruption issue where Clang compiled the C code with `short-enums = false`, while bindgen generated bindings with `short-enums = true` (#102)
* The default MbedTLS backend is now OpenThread's own **bundled** MbedTLS, not the external `mbedtls-rs-sys` crate. A default-features build reuses the committed prebuilt libraries and needs **no C toolchain** (clang/cmake/ninja). (#101)
  * To build OpenThread against the external `mbedtls-rs-sys` instead, enable the `mbedtls-rs-sys` feature. Do this when another crate in the graph already provides `mbedtls-rs-sys` (e.g. `rs-matter`), so a single MbedTLS serves both, or when you need the HW accel capabilities of `mbedtls-rs-sys`.
  * WARNING: do not combine a default (bundled-MbedTLS) OpenThread with a separate `mbedtls-rs-sys` in the same firmware — that links two MbedTLS copies. If your graph needs `mbedtls-rs-sys`, enable this feature so OpenThread reuses it.
* Remote Radio Support (Spinel-RCP) (#98)
* Streamline several features (#97):
  * Put dns-client as part of the default / matter feature

## [0.2.1] - 2026-06-25
* Initial release
