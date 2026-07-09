# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased
* Advertise **Thread 1.4** instead of Thread 1.1 (#103)
  * The stack now reports Thread version 1.4. Note that Thread 1.3+ is the floor Matter-over-Thread expects; for a plain node (no Border Router, no TREL) the on-air/radio contract is unchanged past 1.2, so this is effectively a version bump plus a few benign internal behaviors (e.g. a more thorough parent search at attach).
  * CSL is deliberately **not** compiled in: both `OPENTHREAD_CONFIG_MAC_CSL_TRANSMITTER_ENABLE` (which otherwise defaults on at >= 1.2) and `OT_CSL_RECEIVER` are forced off. This keeps the radio-platform contract identical to 1.1 — no `EnableCsl` / `ReceiveAt` / `GetCslAccuracy` callbacks are referenced — so every existing `Radio` driver keeps working unchanged. Low-power CSL (SSED) remains a future opt-in.
  * HW-validated over an RCP: attach, SRP registration, and a `ping-stress` datapath sweep against a live Border Router.
* Alternate short address support (Thread 1.4 FTD) (#103)
  * Implements `otPlatRadioSetAlternateShortAddress`, which OpenThread calls during a child-to-router role transition so the node keeps receiving frames addressed to its previous RLOC16 for a short window (~8s). Surfaced as `Config::alt_short_addr`.
  * Honored by the software `MacRadio` filter, so radios with no MAC offload (e.g. `NrfRadio`) accept both the primary and alternate short address for free. `SpinelRadio` programs the RCP's alternate-short-address property, but only when the RCP advertises the `ALT_SHORT_ADDR` capability (stock RCPs without it fall back to the primary only). `EspRadio` does not yet honor it: `esp-radio` exposes no public multi-PAN / second-short-address API, so the alternate is dropped by the hardware filter for now. In all unsupported cases the alternate is a reliability optimization — peers relearn the new RLOC16 within the window and higher layers retransmit.
* New APIs for calling into OpenThread directly (#103)
  * `OpenThread::with_instance`: an `unsafe` escape hatch that runs a closure with the raw `*mut otInstance`, inside an active state scope, for calling OpenThread C APIs this crate does not yet wrap.
  * `OpenThread::become_router` (`ftd`): request an immediate router upgrade (`otThreadBecomeRouter`) instead of waiting for OpenThread's jittered automatic one. New std example `become_router` demonstrates it (and deterministically exercises the alternate-short-address transition).
* NRF52: Fix a corruption issue where Clang compiled the C code with `short-enums = false`, while bindgen generated bindings with `short-enums = true` (#102)
* The default MbedTLS backend is now OpenThread's own **bundled** MbedTLS, not the external `mbedtls-rs-sys` crate. A default-features build reuses the committed prebuilt libraries and needs **no C toolchain** (clang/cmake/ninja). (#101)
  * To build OpenThread against the external `mbedtls-rs-sys` instead, enable the `mbedtls-rs-sys` feature. Do this when another crate in the graph already provides `mbedtls-rs-sys` (e.g. `rs-matter`), so a single MbedTLS serves both, or when you need the HW accel capabilities of `mbedtls-rs-sys`.
  * WARNING: do not combine a default (bundled-MbedTLS) OpenThread with a separate `mbedtls-rs-sys` in the same firmware — that links two MbedTLS copies. If your graph needs `mbedtls-rs-sys`, enable this feature so OpenThread reuses it.
* Address the Tier 2 API gaps:
  * `OpenThread::energy_scan`: per-channel max-RSSI survey, e.g. for picking the quietest channel before forming a network
    * `SpinelRadio` performs real scans on the RCP via the spinel `MAC_SCAN` properties (validated on an ESP32-C6 `ot-rcp`); works even when the RCP under-reports the `ENERGY_SCAN` capability
    * `otPlatRadioGetRssi` now reports "invalid RSSI" instead of a fake -128 dBm, and `otPlatRadioEnergyScan` routes to the radio instead of panicking
    * NOTE: the `esp`/`nrf` SoC radios yield no measurements for now — their HALs (`esp-radio` 0.18, `embassy-nrf` 0.10) do not expose the hardware energy detector yet
  * `OpenThread::create_new_network_dataset` (`ftd`): generate the Operational Dataset for a brand-new network with random security parameters
  * `OpenThread::join` (new `joiner` feature): Thread-native MeshCoP commissioning with a pre-shared joiner key (PSKd)
  * `OpenThread::ping` (`ping-sender` feature, now part of the default `matter` bundle): ICMPv6 Echo diagnostics with per-reply callback and final statistics
  * Bugfix: `OperationalDataset::store_raw` wrote the *pending* timestamp into the *active* timestamp field
  * New std (RCP-over-serial) examples: `ping` (attach + ping the Leader ALOC), `form` (energy scan + form a new network as Leader), `joiner` (Thread-native commissioning)
* Address all Tier 1 API gaps (#99):
  * `OpenThread::join_multicast` / `leave_multicast`: interface-level IPv6 multicast group subscription (needed e.g. for Matter group messaging); idempotent
  * Sleepy End Device tuning: `poll_period` / `set_poll_period`, `child_timeout` / `set_child_timeout`, and the child-supervision interval / check-timeout getters and setters
  * Counters and diagnostics: `mac_counters`, `mle_counters`, `ip_counters` (with resets), `uptime_millis`, and the `OpenThread::version` / `OpenThread::thread_version` associated functions
  * `detach_gracefully`: announce departure to the mesh and stop Thread — the proper "forget network" / decommissioning path
* Remote Radio Support (Spinel-RCP) (#98)
* Streamline several features (#97):
  * Put dns-client as part of the default / matter feature
  * Rename the srp feature to srp-client
  * Remove the udp feature
* API for all metrics necessary so as to implement the Thread Diagnostics Matter cluster (#96)
* Keep looping forever in receive, if rx_on_idle is active (#95)
* Implement an API for fetching buffer usage stats (#93)
* Add EspRadio::with_rx_queue_size to make the RX queue depth tunable (#91)

## [0.2.0] - 2026-06-25
* Initial release
