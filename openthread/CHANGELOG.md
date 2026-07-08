# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased
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
