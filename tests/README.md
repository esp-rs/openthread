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
  driven via the crate API, reports `role: <role>` lines on stdout.
- [`cli_ftd`](src/bin/cli_ftd.rs) — the same stack driven exclusively through
  OpenThread's **C CLI** (the `cli` feature of `openthread`/`openthread-sys`):
  stdin lines to the interpreter, its output to stdout. This is the DUT shape
  the upstream harness spawns over a pty (`OT_CLI_PATH`), verified to hold up
  under pexpect (echo, `Done` terminators, `state` polling).

## Tests

```
cargo test
```

- [`formation`](tests/formation.rs) — two `sim_node` processes must form a Thread
  network on localhost (Leader + attached Child), proving the whole radio/alarm/
  tasklet path end-to-end.
- [`cli`](tests/cli.rs) — a `cli_ftd` node is driven harness-style
  (`dataset set active` / `ifconfig up` / `thread start` / `state` polling) to
  Leader, and an API-driven `sim_node` attaches to it — the mixed
  CLI-node/API-node topology.

Each test file picks its own `PORT_BASE` range, so parallel runs use disjoint
media.

## Upstream suites (`cargo xtask itest`)

The repository `xtask` runs *unmodified* upstream OpenThread e2e suites against
`cli_ftd`:

```
cargo xtask itest                      # curated thread-cert allowlist, real time
cargo xtask itest Cert_5_1_01_RouterAttach
cargo xtask itest --suite expect       # expect suite (needs the `expect` binary)
```

`thread-cert` scenarios (incl. Thread certification test plan derivatives) run
with `OT_CLI_PATH` pointing at `cli_ftd`, in real-time mode, with the harness's
own multicast sniffer verifying MLE exchanges on the wire. Python deps are
provisioned automatically into `.build/itest/venv` from the suite's pinned
requirements. See the allowlists in [itest.rs](../xtask/src/itest.rs) for
what runs and why some tests stay excluded.

## Roadmap

1. ~~`SimRadio` + multi-process network-formation smoke test~~
2. ~~A `cli` feature in `openthread-sys`/`openthread` linking the upstream C CLI,
   plus the `cli_ftd` drop-in DUT binary~~
3. ~~Real-time-mode `thread-cert` subset via `cargo xtask itest`~~ — expand the
   allowlist over time; run in CI; verify the `expect` suite where the binary
   is available; mixed Rust/C-node topologies.
4. Virtual-time support (a custom embassy-time driver speaking the simulator's
   event protocol) to unlock the full `thread-cert` suite as upstream runs it
   (fast, deterministic, no real-time races).
5. Persistent (file-backed) `Settings` + real reset semantics, unlocking the
   reboot/reset test groups.
