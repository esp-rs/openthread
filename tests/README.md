# openthread-tests

This crate provides `openthread`-derived binaries used as targets when executing the OpenThread upstream E2E test suites.

[`cli_node`](src/bin/cli_node.rs) is the most important one, which is the full `openthread` stack driven through
OpenThread's CLI API.

Executing the upstream E2E suites against `cli_node` allows us to validate the correctness of the **platform** (`otPlat*`) 
plumbing the `openthread` crate provides on top of the wrapped OpenThread C library. 
As in the alarms' callbacks, the tasklets execution, the drivers'  callbacks which are wired to the Radio trait
and **the concrete Radio implementation itself**.

## Tiers

| Tier | Radio | What it exercises |
| --- | --- | --- |
| sim, real time | [`SimRadio`](src/sim.rs): the upstream UDP-multicast medium | the whole platform against real Python-harness pacing |
| sim, virtual time | [`VtRadio`](src/vt.rs) + [`executor`](src/executor.rs): the simulator's lockstep protocol | the same, deterministically, in seconds - the CI tier |
| hardware, RCP | `SpinelRadio` over a serial co-processor | the spinel host path, real RF |
| hardware, MCU | this repo's firmware nodes ([esp](esp/), [nrf](nrf/)) reached via [`serial_bridge`](src/bin/serial_bridge.rs) | the crate's own radio drivers on real silicon, real timing |

## Running

```sh
cargo test                 # smoke: two cli_nodes form a network on the sim medium
cargo xtask itest          # curated thread-cert allowlist, real time
cargo xtask itest --virtual-time
cargo xtask itest --hw-port /dev/ttyACM0=mcu --hw-port /dev/ttyACM1=rcp
```

See `cargo xtask itest --help` for suites, tiers and flags.
