# `openthread` STD (host) examples

These examples run the full OpenThread stack on a **host** (Linux/macOS) and
drive a **remote** `ot-rcp` radio co-processor over a serial port using the
spinel protocol (the crate's `std` feature / `SpinelRadio` + `SerialPort`).

Unlike the `esp`/`nrf` examples — where the 802.15.4 radio is local to the MCU —
here the radio lives on a separate chip (e.g. an nRF52840 dongle or an ESP32-C6
flashed with stock `ot-rcp` firmware) connected over USB serial.

## Requirements

- A device running stock `ot-rcp` firmware, exposed as a serial port
  (e.g. `/dev/ttyUSB0`, `/dev/ttyACM0`, or `/dev/tty.usbmodem*` on macOS).
- The same 802.15.4 network settings (`THREAD_DATASET`) as your other Thread
  nodes / border router.

## Running

```sh
# Point RCP_SERIAL at your dongle (default: /dev/ttyACM0), THREAD_DATASET at your
# network's operational dataset (a hex TLV string; a default is baked in).
RCP_SERIAL=/dev/ttyACM0 \
THREAD_DATASET=<hex-tlv> \
  cargo run --features force-generate-bindings --bin basic_udp
```

Available bins:

- **`basic_udp`** — provisions an MTD node and echoes IPv6 UDP packets.
- **`srp`** — registers an SRP host + service, then serves UDP.
- **`dns`** — resolves a host name (AAAA) and browses `_matter._tcp` via DNS-SD.

`force-generate-bindings` is currently required (until the committed host-target
bindings match the active feature set).

## Notes

- The serial transport (`SerialPort`) is **Unix-only** for now; Windows serial
  needs a different (overlapped-I/O) path — a planned addition behind the same
  `std` feature.
- There is no `ProxyRadio` / high-priority radio executor split as in the
  embedded examples: serial I/O is not latency-critical, so `SpinelRadio` runs
  directly inside `OpenThread::run`.
