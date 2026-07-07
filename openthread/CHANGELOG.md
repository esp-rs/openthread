# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased
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
