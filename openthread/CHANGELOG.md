# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased
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
