# openthread-tests

Host-side integration-test binaries for the [`openthread`](../openthread) crate.

The goal is to exercise this crate's *platform implementation* — the embassy-based
alarm, the tasklet pumping, the software MAC ([`MacRadio`](../openthread/src/radio.rs))
and the `Radio` contract — against real multi-node Thread scenarios, ultimately by
reusing the upstream OpenThread e2e test suites (`tests/scripts/thread-cert`,
`tests/scripts/expect` in the [OpenThread repo](../openthread-sys/openthread/tests)),
the same way those suites drive the upstream C simulation binaries.

## How

The upstream simulation platform models 802.15.4 "RF" as UDP multicast on the
loopback interface: one datagram per frame (a channel byte + the PSDU), group
`224.0.0.116`, RX bound to `(group, PORT_BASE)` with `SO_REUSEPORT`, TX bound to
`(127.0.0.1, PORT_BASE + node id)` so the source port identifies the sender.

[`sim_radio::SimRadio`](src/sim_radio.rs) implements this crate's `Radio` trait over
that exact wire protocol. A node built on it participates in the same simulated
radio medium as upstream `ot-cli-ftd` / `ot-rcp` simulation binaries, and its frames
(with a real FCS) are visible to the upstream harness sniffer.

## Binaries

- [`sim_node`](src/bin/sim_node.rs) — a full `openthread` stack on `SimRadio`
  (wrapped in `MacRadio`, i.e. ACKs/filtering in software, like on PHY-only real
  radios). Spawned as `sim_node <node id>`, mirroring `ot-cli-ftd <node id>`;
  reports `role: <role>` lines on stdout.

## Tests

```
cargo test
```

- [`formation`](tests/formation.rs) — two `sim_node` processes must form a Thread
  network on localhost (Leader + attached Child), proving the whole radio/alarm/
  tasklet path end-to-end.

Each test run picks its own `PORT_BASE`, so parallel runs use disjoint media.

## Roadmap

1. ~~`SimRadio` + multi-process network-formation smoke test~~ (this crate)
2. A `cli` feature in `openthread-sys`/`openthread` linking the upstream C CLI
   (`otCliInit`/`otCliInputLine` over stdio), turning `sim_node` into a drop-in
   DUT for the upstream harness (`OT_CLI_PATH=… tests/scripts/thread-cert/…`,
   expect suite via `$OT_SIMULATION_APPS`), including mixed Rust/C-node topologies.
3. Real-time-mode subset of the upstream suites in CI (an `xtask itest` runner).
4. Virtual-time support (a custom embassy-time driver speaking the simulator's
   event protocol) to unlock the full `thread-cert` suite as upstream runs it.
