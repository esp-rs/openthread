//! `itest`: run upstream OpenThread e2e suites against the Rust-platform
//! simulation DUT.
//!
//! The DUT is `openthread-tests`' `cli_ftd`: the full `openthread` stack on
//! the Rust platform (embassy alarm, tasklet pumping, software MAC, the
//! UDP-multicast `SimRadio`), driven through OpenThread's C CLI - the exact
//! process shape the upstream harness spawns for its own `ot-cli-ftd`
//! simulation binary. Two upstream suites can be pointed at it, both taken
//! verbatim from the OpenThread submodule (`openthread-sys/openthread`):
//!
//! - `cert`: the Python `tests/scripts/thread-cert` scenarios, in real-time
//!   mode (`VIRTUAL_TIME=0`) - node processes spawned via `OT_CLI_PATH`,
//!   frames observed by the harness's own multicast sniffer. Python deps
//!   (pexpect, pycryptodome) are provisioned into a venv under `.build/`.
//! - `expect`: the Tcl `tests/scripts/expect` CLI tests - node processes
//!   spawned as `$OT_SIMULATION_APPS/cli/ot-cli-ftd`, which a shim directory
//!   of symlinks points at the DUT. Needs the system `expect` binary.
//!
//! Both suites run against curated allowlists (see [`CERT_TESTS`] /
//! [`EXPECT_TESTS`]), where every entry is verified green against the DUT.
//! The cert allowlists cover the entire upstream `Cert_*` corpus; the expect
//! allowlist covers the tests runnable with a CLI-FTD-only DUT (the rest of
//! that corpus needs posix/RCP node flavors or `diag` commands).
//!
//! # The hardware tier
//!
//! With `--hw-port` the simulated medium is dropped for real RF: each node
//! drives its own 802.15.4 co-processor over a serial link (the DUT's
//! `SpinelRadio`, see `openthread_tests::hw_radio`). Nothing else changes -
//! the harness still spawns `$OT_CLI_PATH <node id>` and still talks to it
//! over a pty - so the same unmodified upstream scenarios run, which is the
//! whole point: it is the only tier where the spinel radio, the serial
//! transport and real over-the-air timing are exercised at all.
//!
//! It is manually invoked and never part of CI: it needs one dongle per node
//! physically attached to the machine. Consequences of real RF:
//!
//! - real time only (virtual time has no meaning when the radio keeps its
//!   own clock);
//! - one test at a time, and no medium isolation between tests - there is a
//!   single air, so `PORT_OFFSET` cannot separate a straggler node of a
//!   previous run from this one;
//! - node count is capped by the number of attached dongles, so
//!   [`HW_TESTS`] carries each test's node count and the oversized ones are
//!   reported as skipped rather than dropped silently.
//!
//! A second hardware tier is the natural follow-on: node = firmware on an
//! MCU, driven over its serial console. It reuses everything here - the port
//! map, the node-count gating, the allowlist - and only swaps what
//! `OT_CLI_PATH` points at, from the host DUT to a bridge binary that pipes
//! stdin/stdout to a serial port. That one exercises the on-MCU drivers
//! (`NrfRadio`/`EspRadio`), `MacRadio`'s real ACK deadlines and
//! `ProxyRadio`'s executor split, none of which this tier reaches.

use std::fs;
use std::io::{BufRead, BufReader};
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};

use log::info;

/// The `thread-cert` tests run by default: verified green against the DUT in
/// real-time mode. The rest of the corpus runs in virtual time only (see
/// [`CERT_TESTS_VT_EXTRA`]) - mostly for wall-clock reasons, since real time
/// serves every scripted delay at 1x.
const CERT_TESTS: &[&str] = &[
    // Two-node leader/router attach, with full packet verification by the
    // harness sniffer (MLE parsing incl. decryption).
    "Cert_5_1_01_RouterAttach",
    // Child address registration + timeout, incl. a sleepy child (exercises
    // the indirect-messaging path: FP-in-ack, `mAckedWithFramePending`).
    "Cert_5_1_02_ChildAddressTimeout",
    // REED parent selection by connectivity (5 nodes).
    "Cert_5_1_09_REEDAttachConnectivity",
    // REED attach (leader/REED/MED topology).
    "Cert_5_2_01_REEDAttach",
    // Link-local unicast + multicast ping exchanges between two routers.
    "Cert_5_3_01_LinkLocal",
    // Realm-local pings across a topology incl. an SED.
    "Cert_5_3_02_RealmLocal",
    // EID-to-RLOC address queries across 5 nodes. Slow: the script sleeps
    // out a 700s router-id expiry (`simulator.go(700)`), which real-time
    // mode serves at 1x - see `test_timeout`.
    "Cert_5_3_03_AddressQuery",
];

/// Additional `thread-cert` tests run only in virtual time: verified green
/// there (each entry survived a full-corpus discovery sweep plus repeated
/// confirmation batches), while their real-time pacing is either unverified
/// or known-marginal. Real-time promotion is per-test, by demonstrated
/// stability at 1x.
///
/// Together with [`CERT_TESTS`], this covers the ENTIRE upstream `Cert_*`
/// corpus (102 scenarios) - nothing is excluded.
const CERT_TESTS_VT_EXTRA: &[&str] = &[
    // A SED-originated ping races the SED poll latency against the ping
    // deadline - deterministic under virtual time, marginal at 1x.
    "Cert_5_3_04_AddressMapCache",
    // MLE attach, router lifecycle and topology formation.
    "Cert_5_1_03_RouterAddressReallocation",
    "Cert_5_1_04_RouterAddressReallocation",
    "Cert_5_1_05_RouterAddressTimeout",
    "Cert_5_1_06_RemoveRouterId",
    "Cert_5_1_07_MaxChildCount",
    "Cert_5_1_08_RouterAttachConnectivity",
    "Cert_5_1_10_RouterAttachLinkQuality",
    "Cert_5_1_11_REEDAttachLinkQuality",
    "Cert_5_1_12_NewRouterNeighborSync",
    "Cert_5_2_03_LeaderReject2Hops",
    "Cert_5_2_04_REEDUpgrade",
    "Cert_5_2_05_AddressQuery",
    "Cert_5_2_06_RouterDowngrade",
    "Cert_5_2_07_REEDSynchronization",
    // Network layer: routing, address queries, duplicate detection.
    "Cert_5_3_05_RoutingLinkQuality",
    "Cert_5_3_06_RouterIdMask",
    "Cert_5_3_07_DuplicateAddress",
    "Cert_5_3_08_ChildAddressSet",
    "Cert_5_3_09_AddressQuery",
    "Cert_5_3_10_AddressQuery",
    "Cert_5_3_11_AddressQueryTimeoutIntervals",
    // Reboot / split-merge / child-reset / persistent-dataset scenarios:
    // settings survive the CLI `reset` in a per-node file (the DUT's
    // `FileSettings`), so a reset node rejoins with its dataset intact.
    "Cert_5_1_13_RouterReset",
    "Cert_5_5_01_LeaderReboot",
    "Cert_5_5_02_LeaderReboot",
    "Cert_5_5_03_SplitMergeChildren",
    "Cert_5_5_04_SplitMergeRouters",
    "Cert_5_5_05_SplitMergeREED",
    "Cert_5_5_07_SplitMergeThreeWay",
    "Cert_6_5_01_ChildResetReattach",
    "Cert_6_5_02_ChildResetReattach",
    "Cert_6_5_03_ChildResetSynchronize",
    "Cert_9_2_08_PersistentDatasets",
    // Network data registration/propagation (the `border-router` DUT feature).
    "Cert_5_6_01_NetworkDataRegisterBeforeAttachLeader",
    "Cert_5_6_02_NetworkDataRegisterBeforeAttachRouter",
    "Cert_5_6_03_NetworkDataRegisterAfterAttachLeader",
    "Cert_5_6_04_NetworkDataRegisterAfterAttachRouter",
    "Cert_5_6_05_NetworkDataRegisterAfterAttachRouter",
    "Cert_5_6_06_NetworkDataExpiration",
    "Cert_5_6_07_NetworkDataRequestREED",
    "Cert_5_6_09_NetworkDataForwarding",
    // TMF network diagnostics (the `netdiag-client` DUT feature).
    "Cert_5_7_01_CoapDiagCommands",
    "Cert_5_7_02_CoapDiagCommands",
    "Cert_5_7_03_CoapDiagCommands",
    // thrKeySequenceCounter rotation + security policy TLV.
    "Cert_5_8_02_KeyIncrement",
    "Cert_5_8_03_KeyIncrementRollOver",
    "Cert_5_8_04_SecurityPolicyTLV",
    // The MED/SED (`Cert_6_*`) mirror of the attach / network-layer / key
    // groups: exercises the sleepy-child paths (indirect messaging, polling,
    // the radio sleep contract) end to end.
    "Cert_6_1_01_RouterAttach",
    "Cert_6_1_02_REEDAttach",
    "Cert_6_1_03_RouterAttachConnectivity",
    "Cert_6_1_04_REEDAttachConnectivity",
    "Cert_6_1_05_REEDAttachConnectivity",
    "Cert_6_1_06_REEDAttachLinkQuality",
    "Cert_6_1_07_RouterAttachLinkQuality",
    "Cert_6_2_01_NewPartition",
    "Cert_6_2_02_NewPartition",
    "Cert_6_3_01_OrphanReattach",
    "Cert_6_3_02_NetworkDataUpdate",
    "Cert_6_4_01_LinkLocal",
    "Cert_6_4_02_RealmLocal",
    "Cert_6_6_01_KeyIncrement",
    "Cert_6_6_02_KeyIncrementRollOver",
    // Border-router network data scenarios (the `border-router` DUT feature).
    "Cert_7_1_01_BorderRouterAsLeader",
    "Cert_7_1_02_BorderRouterAsRouter",
    "Cert_7_1_03_BorderRouterAsLeader",
    "Cert_7_1_04_BorderRouterAsRouter",
    "Cert_7_1_05_BorderRouterAsRouter",
    "Cert_7_1_06_BorderRouterAsLeader",
    "Cert_7_1_07_BorderRouterAsLeader",
    "Cert_7_1_08_BorderRouterAsFED",
    // MeshCoP commissioning (the `commissioner` + `joiner` DUT features:
    // J-PAKE over DTLS; the packet-verifying tests additionally read the
    // `[THCI]` certification dumps off the node's console - see the DUT's
    // cert-log tee).
    "Cert_8_1_01_Commissioning",
    "Cert_8_1_02_Commissioning",
    "Cert_8_1_06_Commissioning",
    "Cert_8_2_01_JoinerRouter",
    "Cert_8_2_02_JoinerRouter",
    "Cert_8_2_05_JoinerRouter",
    "Cert_8_3_01_CommissionerPetition",
    // MeshCoP active/pending operational datasets (MGMT_*_SET dissemination,
    // delay timers, announce, energy scan / PAN-id query).
    "Cert_9_2_01_MGMTCommissionerGet",
    "Cert_9_2_02_MGMTCommissionerSet",
    "Cert_9_2_03_ActiveDatasetGet",
    "Cert_9_2_04_ActiveDataset",
    "Cert_9_2_05_ActiveDataset",
    "Cert_9_2_06_DatasetDissemination",
    "Cert_9_2_07_DelayTimer",
    "Cert_9_2_09_PendingPartition",
    "Cert_9_2_10_PendingPartition",
    "Cert_9_2_11_NetworkKey",
    "Cert_9_2_12_Announce",
    "Cert_9_2_13_EnergyScan",
    "Cert_9_2_14_PanIdQuery",
    "Cert_9_2_15_PendingPartition",
    "Cert_9_2_16_ActivePendingPartition",
    "Cert_9_2_17_Orphan",
    "Cert_9_2_18_RollBackActiveTimestamp",
    "Cert_9_2_19_PendingDatasetGet",
];

/// The `expect` tests run by default: verified green against the DUT. The
/// rest of the corpus needs node flavors the DUT shim directory deliberately
/// does not provide (posix hosts, RCPs, MTD builds) or `diag` commands.
const EXPECT_TESTS: &[&str] = &[
    "cli-dataset",
    "cli-networkname",
    "cli-extaddr",
    "cli-counters",
    "cli-ping",
];

/// The `test_*.py` functional scripts (same directory and runner as the
/// `Cert_*` scenarios) run in virtual time: verified green against the DUT.
///
/// The excluded remainder of that pool:
/// - `test_srp_register_500_services`: the DUT stops answering after eight
///   `srp client service add` commands land while registration traffic is
///   in flight - a wedge needing dedicated debugging (not heap: reproduced
///   at the 65528 maximum).
/// - `test_pbbr_aloc`: needs the Backbone Router node flavor (node.py spawns
///   a literal `./ot-cli-ftd` from the cwd for it, and the DUT would need
///   `OT_BACKBONE_ROUTER`).
const FUNC_TESTS_VT: &[&str] = &[
    "test_anycast",
    "test_anycast_locator",
    "test_diag",
    "test_ipv6_fragmentation",
    "test_br_upgrade_router_role",
    "test_child_supervision",
    "test_coap",
    "test_coap_block",
    "test_coap_observe",
    "test_coaps",
    "test_common",
    "test_crypto",
    "test_dataset_updater",
    "test_detach",
    "test_dns_client_config_auto_start",
    "test_dnssd",
    "test_dnssd_name_with_special_chars",
    "test_history_tracker",
    "test_inform_previous_parent_on_reattach",
    "test_ipv6",
    "test_ipv6_source_selection",
    "test_key_rotation_and_key_guard_time",
    "test_leader_reboot_multiple_link_request",
    "test_lowpan",
    "test_mac802154",
    "test_mac_scan",
    "test_mle",
    "test_mle_msg_key_seq_jump",
    "test_netdata_publisher",
    "test_network_data",
    "test_network_layer",
    "test_on_mesh_prefix",
    "test_ping",
    "test_ping_lla_src",
    "test_radio_filter",
    "test_reed_address_solicit_rejected",
    "test_reset",
    "test_router_downgrade_on_sec_policy_change",
    "test_router_multicast_link_request",
    "test_router_reattach",
    "test_router_reboot_multiple_link_request",
    "test_router_upgrade",
    "test_route_table",
    "test_service",
    "test_set_mliid",
    "test_srp_auto_host_address",
    "test_srp_auto_start_mode",
    "test_srp_client_change_lease",
    "test_srp_client_remove_host",
    "test_srp_client_save_server_info",
    "test_srp_lease",
    "test_srp_many_services_mtu_check",
    "test_srp_name_conflicts",
    "test_srp_register_services_diff_lease",
    "test_srp_register_single_service",
    "test_srp_server_anycast_mode",
    "test_srp_server_reboot_port",
    "test_srp_sub_type",
    "test_srp_ttl",
    "test_zero_len_external_route",
];

/// The tests the hardware tier runs by default, with the node count each
/// needs (i.e. the number of dongles it takes to run it).
///
/// Unlike every other allowlist in this file, these entries are **not**
/// verified green - there is no hardware in CI to verify them against. They
/// are a deliberately small bring-up set, chosen to isolate failures to the
/// radio rather than the scenario: the two `Cert_*` entries are already
/// verified at 1x pacing on the simulated medium (they are in
/// [`CERT_TESTS`]), so a failure here points at the co-processor link, not
/// at the stack above it. Grow the list as entries go green on real
/// hardware - the same per-test promotion rule the other allowlists use.
const HW_TESTS: &[(&str, usize)] = &[
    // Leader/router attach: the smallest scenario that proves two real
    // radios can find and join each other.
    ("Cert_5_1_01_RouterAttach", 2),
    // Link-local unicast + multicast ping exchanges - real ACKs, real
    // retries, real timing.
    ("Cert_5_3_01_LinkLocal", 2),
    // An active scan, i.e. the co-processor's own channel-scan path.
    ("test_mac_scan", 2),
    // REED -> router promotion; more MLE traffic over the same two radios.
    ("test_router_upgrade", 2),
    // Multi-hop ping across a three-node line (needs a third dongle).
    ("test_ping", 3),
    // Route table convergence over three real links.
    ("test_route_table", 3),
];

/// Wall-clock budget for a test; exceeding it kills and fails the test.
///
/// Sized for real-time mode, where every `simulator.go(N)` in a script is a
/// literal N-second sleep - a test's budget is roughly its summed waits plus
/// setup/teardown slack. The default covers the corpus's common shape;
/// scripts that sleep out long protocol timeouts get their own entry.
fn test_timeout(test: &str) -> Duration {
    match test {
        // Sleeps out a 700s router-id expiry.
        "Cert_5_3_03_AddressQuery" => Duration::from_secs(1200),
        _ => Duration::from_secs(600),
    }
}

/// Arguments of the `itest` xtask subcommand.
#[derive(clap::Args, Debug)]
pub struct ItestArgs {
    /// The upstream suite to run the tests from.
    #[arg(long, value_enum, default_value_t = Suite::Cert)]
    suite: Suite,

    /// Run the `cert` suite in virtual time: the upstream Python simulator
    /// coordinates a lockstep event protocol instead of real-time waits, so
    /// scripted delays pass instantly and runs are deterministic. The DUT
    /// switches modes via the inherited `VIRTUAL_TIME` env var.
    #[arg(long)]
    virtual_time: bool,

    /// Run against real radios instead of the simulated medium: a serial
    /// device per node, given in node-id order (`--hw-port /dev/ttyACM0
    /// --hw-port /dev/ttyACM1 ...`), each an 802.15.4 co-processor the DUT
    /// drives over spinel. A device may carry its own link speed
    /// (`--hw-port /dev/ttyUSB0@460800`), which a mixed rig needs. Falls back
    /// to the `OT_HW_PORTS` environment variable (comma-separated). Real time
    /// only, and never CI - see the module docs.
    #[arg(long, value_name = "DEVICE[@BAUD]")]
    hw_port: Vec<String>,

    /// The link speed for `--hw-port` devices that do not carry one of their
    /// own (default: 115200, as in the host examples; ESP32xx RCPs want
    /// 460800). Ignored by devices exposing a USB CDC serial port.
    #[arg(long)]
    hw_baud: Option<u32>,

    /// Skip (re)building the DUT binaries.
    #[arg(long)]
    skip_build: bool,

    /// Override the per-test wall-clock timeout, in seconds (default: a
    /// per-test table sized for real-time pacing - see [`test_timeout`]).
    /// Useful for discovery sweeps over non-allowlisted tests, where a
    /// deadlocked test should fail fast.
    #[arg(long)]
    timeout: Option<u64>,

    /// Test names (file name, extension optional); defaults to the suite's
    /// curated allowlist.
    tests: Vec<String>,
}

#[derive(clap::ValueEnum, Debug, Clone, Copy, PartialEq, Eq)]
enum Suite {
    /// Python `tests/scripts/thread-cert` scenarios, real-time mode.
    Cert,
    /// Tcl `tests/scripts/expect` CLI tests.
    Expect,
}

/// Outcome of one test run.
enum Outcome {
    Passed,
    /// The upstream skip convention (exit code 77).
    Skipped,
    Failed(String),
}

pub fn run(workspace: &Path, args: &ItestArgs) -> Result<()> {
    let ot_root = workspace.join("openthread-sys").join("openthread");
    let build_dir = workspace.join(".build").join("itest");
    fs::create_dir_all(&build_dir).with_context(|| format!("creating {}", build_dir.display()))?;

    let hw_ports = hw_ports(args)?;

    if !hw_ports.is_empty() {
        if args.virtual_time {
            bail!(
                "--virtual-time and --hw-port are mutually exclusive: a real radio \
                 keeps its own clock, so the simulator's lockstep protocol cannot \
                 drive it"
            );
        }

        info!(
            "Hardware tier: {} node(s) on {}",
            hw_ports.len(),
            hw_ports.join(", "),
        );
    }

    let cli_ftd = build_dut(workspace, args.skip_build, !hw_ports.is_empty())?;

    let runner = Runner {
        ot_root,
        build_dir: build_dir.clone(),
        cli_ftd,
        hw: Hw {
            ports: hw_ports.clone(),
            baud: args.hw_baud,
        },
        virtual_time: args.virtual_time,
        timeout_secs: args.timeout,
    };

    let mut skipped_oversized = Vec::new();

    let tests: Vec<String> = if args.tests.is_empty() {
        if !hw_ports.is_empty() {
            // Only what the attached dongles can actually host; the rest is
            // reported, not silently dropped.
            let (runnable, oversized): (Vec<_>, Vec<_>) = HW_TESTS
                .iter()
                .partition(|(_, nodes)| *nodes <= hw_ports.len());

            skipped_oversized = oversized
                .iter()
                .map(|(test, nodes)| format!("{test} (needs {nodes})"))
                .collect();

            runnable.iter().map(|(test, _)| test.to_string()).collect()
        } else {
            let defaults = match args.suite {
                Suite::Cert => CERT_TESTS,
                Suite::Expect => EXPECT_TESTS,
            };
            let extra = match args.suite {
                Suite::Cert if args.virtual_time => CERT_TESTS_VT_EXTRA,
                _ => &[][..],
            };
            let func = match args.suite {
                Suite::Cert if args.virtual_time => FUNC_TESTS_VT,
                _ => &[][..],
            };
            defaults
                .iter()
                .chain(extra)
                .chain(func)
                .map(|t| t.to_string())
                .collect()
        }
    } else {
        args.tests
            .iter()
            .map(|t| {
                t.trim_end_matches(".py")
                    .trim_end_matches(".exp")
                    .to_string()
            })
            .collect()
    };

    let mut results = Vec::new();

    for (index, test) in tests.iter().enumerate() {
        info!("Running {test} ({}/{})", index + 1, tests.len());

        let outcome = match args.suite {
            Suite::Cert => runner.run_cert_test(test, index)?,
            Suite::Expect => runner.run_expect_test(test)?,
        };

        match &outcome {
            Outcome::Passed => info!("{test}: PASSED"),
            Outcome::Skipped => info!("{test}: SKIPPED"),
            Outcome::Failed(reason) => {
                info!("{test}: FAILED ({reason})");
                runner.hw_hint(test);
            }
        }

        results.push((test.clone(), outcome));
    }

    let failed: Vec<&str> = results
        .iter()
        .filter_map(|(test, outcome)| matches!(outcome, Outcome::Failed(_)).then_some(&**test))
        .collect();

    if !skipped_oversized.is_empty() {
        info!(
            "Not run - more nodes than attached radios ({}): {}",
            hw_ports.len(),
            skipped_oversized.join(", "),
        );
    }

    info!(
        "Summary: {} passed, {} skipped, {} failed (of {})",
        results
            .iter()
            .filter(|(_, o)| matches!(o, Outcome::Passed))
            .count(),
        results
            .iter()
            .filter(|(_, o)| matches!(o, Outcome::Skipped))
            .count(),
        failed.len(),
        results.len(),
    );

    if !failed.is_empty() {
        bail!("failed tests: {}", failed.join(", "));
    }

    Ok(())
}

/// The radio devices of the hardware tier, in node-id order; empty when this
/// is an ordinary simulated run.
///
/// `--hw-port` wins over `OT_HW_PORTS`, which exists so a shell can export
/// the rig once and leave the command lines alone.
fn hw_ports(args: &ItestArgs) -> Result<Vec<String>> {
    let ports: Vec<String> = if !args.hw_port.is_empty() {
        args.hw_port.clone()
    } else {
        std::env::var("OT_HW_PORTS")
            .unwrap_or_default()
            .split(',')
            .map(str::trim)
            .filter(|port| !port.is_empty())
            .map(str::to_string)
            .collect()
    };

    // A missing device is worth catching here rather than as an opaque node
    // failure ten seconds into a test. The optional `@baud` suffix is the
    // node's business (see `openthread_tests::hw_radio`), not part of the path.
    for port in &ports {
        let device = port.rsplit_once('@').map_or(&**port, |(device, _)| device);

        if !Path::new(device).exists() {
            bail!("no such radio device: {device}");
        }
    }

    Ok(ports)
}

/// The hardware tier's configuration, as handed to the node processes.
struct Hw {
    /// The serial devices, in node-id order (empty = simulated run).
    ports: Vec<String>,
    /// The link speed, if overridden.
    baud: Option<u32>,
}

impl Hw {
    /// Whether this is a hardware run at all.
    fn active(&self) -> bool {
        !self.ports.is_empty()
    }

    /// Apply the port map to a node-spawning command. A no-op for a
    /// simulated run, so callers need no branch of their own.
    fn apply(&self, command: &mut Command) {
        if self.ports.is_empty() {
            return;
        }

        command.env("OT_HW_PORTS", self.ports.join(","));

        if let Some(baud) = self.baud {
            command.env("OT_HW_BAUD", baud.to_string());
        }
    }
}

/// What the node logs say when the co-processor did not answer its startup
/// handshake - see [`Runner::hw_hint`].
const RADIO_INIT_FAILED: &str = "Radio init failed";

/// Everything a test run needs, constant for the whole invocation: where the
/// upstream suites live, where our artifacts go, which DUT binary to spawn,
/// and how the nodes' radios are configured.
struct Runner {
    /// The OpenThread submodule root the suites are taken from.
    ot_root: PathBuf,
    /// Where logs, the venv and per-test run dirs go.
    build_dir: PathBuf,
    /// The DUT the harness spawns per node.
    cli_ftd: PathBuf,
    /// The radio configuration (simulated, or the hardware tier's port map).
    hw: Hw,
    /// Virtual-time mode, for the `cert` suite.
    virtual_time: bool,
    /// Per-test wall-clock budget override.
    timeout_secs: Option<u64>,
}

/// Build the DUT binaries (the `openthread-tests` crate is intentionally
/// outside the workspace, like `examples`) and return the `cli_ftd` path.
///
/// `hw` additionally enables the RCP-over-serial radio; the node binary
/// refuses to start with a port map it cannot read, so a stale non-`hw`
/// build cannot turn a hardware run into a simulated one behind our back.
fn build_dut(workspace: &Path, skip_build: bool, hw: bool) -> Result<PathBuf> {
    let tests_crate = workspace.join("tests");

    if !skip_build {
        info!("Building the DUT binaries (openthread-tests)");

        let cargo = std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
        let mut command = Command::new(cargo);
        command.arg("build").arg("--bins");

        if hw {
            command.arg("--features").arg("hw");
        }

        let status = command
            .current_dir(&tests_crate)
            .status()
            .context("spawning `cargo build` for openthread-tests")?;
        if !status.success() {
            bail!("building the DUT binaries failed");
        }
    }

    let cli_ftd = tests_crate
        .join("target")
        .join("debug")
        .join("cli_ftd")
        .canonicalize()
        .context("locating the `cli_ftd` DUT binary (build it first or drop --skip-build)")?;

    Ok(cli_ftd)
}

/// Provision (once) and return the python of the harness venv, with the
/// suite's own pinned requirements installed (pexpect for node ptys,
/// pycryptodome for the sniffer's MLE decryption, pyshark for the
/// unconditional `pktverify` imports - version-pinned, its module layout
/// moves between releases).
fn ensure_venv(build_dir: &Path, thread_cert: &Path) -> Result<PathBuf> {
    let venv = build_dir.join("venv");
    let python = venv.join("bin").join("python");

    if !python.exists() {
        info!("Creating the harness python venv");

        let status = Command::new("python3")
            .arg("-m")
            .arg("venv")
            .arg(&venv)
            .status()
            .context("spawning `python3 -m venv` (is python3-venv installed?)")?;
        if !status.success() {
            bail!("creating the python venv failed");
        }
    }

    let marker = venv.join(".deps-ok");
    if !marker.exists() {
        info!("Installing the harness python deps (thread-cert requirements.txt)");

        let status = Command::new(venv.join("bin").join("pip"))
            .arg("install")
            .arg("--quiet")
            .arg("--requirement")
            .arg(thread_cert.join("requirements.txt"))
            .status()
            .context("spawning venv pip")?;
        if !status.success() {
            bail!("installing the python deps failed");
        }

        fs::write(&marker, "")?;
    }

    Ok(python)
}

impl Runner {
    fn run_cert_test(&self, test: &str, index: usize) -> Result<Outcome> {
        let Self {
            ot_root,
            build_dir,
            cli_ftd,
            hw,
            virtual_time,
            timeout_secs,
        } = self;
        let (virtual_time, timeout_secs) = (*virtual_time, *timeout_secs);

        let thread_cert = ot_root.join("tests").join("scripts").join("thread-cert");
        let python = ensure_venv(build_dir, &thread_cert)?;

        let script = thread_cert.join(format!("{test}.py"));
        if !script.is_file() {
            bail!("no such thread-cert test: {}", script.display());
        }

        // A fresh cwd per run: the harness drops logs and pcaps into it.
        let run_dir = build_dir.join("run").join(test);
        if run_dir.exists() {
            fs::remove_dir_all(&run_dir)?;
        }
        fs::create_dir_all(&run_dir)?;

        let mut command = Command::new(&python);
        command
            .arg(&script)
            .current_dir(&run_dir)
            .env("PYTHONPATH", &thread_cert)
            // The DUT: node.py spawns `$OT_CLI_PATH <node id>` under a pexpect pty.
            .env("OT_CLI_PATH", cli_ftd)
            // Real or virtual time - for the harness AND, via inheritance, the
            // spawned DUT nodes (which switch their clock/radio accordingly).
            .env("VIRTUAL_TIME", if virtual_time { "1" } else { "0" })
            // Matches the wrapped OpenThread's `OT_THREAD_VERSION` - and keeps
            // node.py off its 1.1-compatibility binary paths.
            .env("THREAD_VERSION", "1.4")
            // Distinct radio medium per test, so a straggler node of a previous
            // test cannot inject frames into this one.
            .env("PORT_OFFSET", (index % 10).to_string())
            // Per-node DUT log files in the run dir (`node.<id>`); the level
            // comes from `RUST_LOG`, so `RUST_LOG=openthread=debug cargo xtask
            // itest <test>` captures a failing node's stack-side view.
            .env("CLI_FTD_LOG", run_dir.join("node"))
            // Per-node persisted settings land in the run dir (fresh per run).
            .env("CLI_FTD_SETTINGS_DIR", run_dir.join("settings"));

        // Real radios, if this is a hardware run (`PORT_OFFSET` above then means
        // nothing - there is only one air).
        hw.apply(&mut command);

        run_logged(command, &run_dir.join("output.log"), test, timeout_secs)
    }

    fn run_expect_test(&self, test: &str) -> Result<Outcome> {
        let Self {
            ot_root,
            build_dir,
            cli_ftd,
            hw,
            timeout_secs,
            ..
        } = self;
        let timeout_secs = *timeout_secs;

        if !binary_exists("expect") {
            bail!(
                "the `expect` binary is required for the expect suite \
             (e.g. `sudo apt-get install expect`)"
            );
        }

        let script = ot_root
            .join("tests")
            .join("scripts")
            .join("expect")
            .join(format!("{test}.exp"));
        if !script.is_file() {
            bail!("no such expect test: {}", script.display());
        }

        // `$OT_SIMULATION_APPS/cli/ot-cli-ftd` is how the suite spawns nodes;
        // point it at the DUT via a shim directory. No `ot-cli-mtd`/`ncp/ot-rcp`
        // links: tests needing those flavors must stay off the allowlist.
        let apps = build_dir.join("simulation-apps");
        let cli_dir = apps.join("cli");
        fs::create_dir_all(&cli_dir)?;

        let link = cli_dir.join("ot-cli-ftd");
        if fs::symlink_metadata(&link).is_ok() {
            fs::remove_file(&link)?;
        }
        std::os::unix::fs::symlink(cli_ftd, &link)
            .with_context(|| format!("symlinking {}", link.display()))?;

        let run_dir = build_dir.join("run").join(test);
        if run_dir.exists() {
            fs::remove_dir_all(&run_dir)?;
        }
        fs::create_dir_all(&run_dir)?;

        let mut command = Command::new("expect");
        command
            .arg("-f")
            .arg(&script)
            // The scripts `source tests/scripts/expect/_common.exp` relative to
            // the cwd, so they must run from the OpenThread repo root. They write
            // nothing there (our log goes to `run_dir` via an absolute path;
            // gcov prefixes only materialize for coverage builds).
            .current_dir(ot_root)
            .env("OT_SIMULATION_APPS", &apps)
            // The DUT nodes run with the repo root as cwd (see above); point
            // their persisted settings at the run dir instead.
            .env("CLI_FTD_SETTINGS_DIR", run_dir.join("settings"));

        hw.apply(&mut command);

        run_logged(command, &run_dir.join("output.log"), test, timeout_secs)
    }

    /// On a failed hardware run, say so when the cause was the radio link
    /// rather than the scenario.
    ///
    /// The stack deliberately does not treat a failed radio handshake as
    /// fatal - it warns, advertises no capabilities and lets the radio
    /// recover later - which is right for a device in the field but turns a
    /// wrong port into a confusing cascade here (the node limps on radio-less
    /// until the harness gives up on it). So dig the real cause out of the
    /// node logs and put it in front of the reader.
    fn hw_hint(&self, test: &str) {
        if !self.hw.active() {
            return;
        }

        let run_dir = self.build_dir.join("run").join(test);

        let dead: Vec<String> = nodes(&run_dir)
            .into_iter()
            .filter(|node| {
                fs::read_to_string(run_dir.join(node))
                    .is_ok_and(|log| log.contains(RADIO_INIT_FAILED))
            })
            .collect();

        if dead.is_empty() {
            // The radios are up, so whatever went wrong is above them - and
            // the per-node logs are where it will be visible (the harness
            // output only shows its own side of a node that went away).
            for node in nodes(&run_dir) {
                echo_log_tail(&run_dir.join(&node), 20);
            }

            return;
        }

        info!(
            "{test}: the radio never came up on {} - check that each --hw-port is an \
             802.15.4 co-processor running `ot-rcp`, and that --hw-baud matches its \
             link speed (default 115200, but 460800 for ESP32xx RCPs)",
            dead.join(", "),
        );
    }
}

/// The per-node log file names in a run directory (`node.1`, `node.2`, ...).
fn nodes(run_dir: &Path) -> Vec<String> {
    let Ok(entries) = fs::read_dir(run_dir) else {
        return Vec::new();
    };

    let mut nodes: Vec<String> = entries
        .flatten()
        .map(|entry| entry.file_name().to_string_lossy().into_owned())
        .filter(|name| name.starts_with("node."))
        .collect();

    nodes.sort();
    nodes
}

/// Run a test command with its output captured to `log_path`, a wall-clock
/// timeout, and the upstream exit-77-means-skip convention. On failure, the
/// log tail is echoed for immediate diagnosis.
fn run_logged(
    mut command: Command,
    log_path: &Path,
    test: &str,
    timeout_secs: Option<u64>,
) -> Result<Outcome> {
    let log =
        fs::File::create(log_path).with_context(|| format!("creating {}", log_path.display()))?;

    let mut child = command
        .stdin(Stdio::null())
        .stdout(log.try_clone()?)
        .stderr(log)
        // Own process group: the harness spawns one process per simulated
        // node, and killing just the harness on timeout would orphan them -
        // still bound to their radio-medium ports, poisoning later runs.
        .process_group(0)
        .spawn()
        .with_context(|| format!("spawning {test}"))?;

    let timeout = timeout_secs.map_or_else(|| test_timeout(test), Duration::from_secs);
    let deadline = Instant::now() + timeout;

    let status = loop {
        if let Some(status) = child.try_wait()? {
            break Some(status);
        }
        if Instant::now() >= deadline {
            child.kill()?;
            child.wait()?;
            break None;
        }
        std::thread::sleep(Duration::from_millis(200));
    };

    // Sweep the whole process group (the group id is the child's pid),
    // whatever the outcome - node processes must never outlive their test.
    //
    // Via `killpg(2)` directly, NEVER by spawning `kill -9 -<pgid>`: procps'
    // `kill` parses a bare negative argument as an option cluster and keeps
    // only its first digit, so `kill -9 -1015086` actually executes
    // `kill(-1, SIGKILL)` - "signal every process this user may signal",
    // which wipes out the whole desktop session (editor server, language
    // servers, the agent driving the suite...). Verified with strace:
    //   kill -0 -1015086     ->  kill(-1, 0)          (!!)
    //   kill -0 -99999       ->  kill(-9, 0)
    //   kill -0 -- -1015086  ->  kill(-1015086, 0)    (correct)
    // The damage is pid-dependent - it bites whenever the pid starts with a
    // '1', i.e. for most pids once the system's pid counter passes 1000000 -
    // which is what made it look like a sporadic environment problem.
    //
    // SAFETY: plain libc call; the pid is our own child's, and an
    // already-reaped group simply yields `ESRCH`, which is ignored.
    unsafe {
        libc::killpg(child.id() as libc::pid_t, libc::SIGKILL);
    }

    let outcome = match status {
        None => Outcome::Failed(format!("timed out after {}s", timeout.as_secs())),
        Some(status) if status.success() => Outcome::Passed,
        Some(status) if status.code() == Some(77) => Outcome::Skipped,
        Some(status) => Outcome::Failed(format!("exit status {status}")),
    };

    if let Outcome::Failed(_) = &outcome {
        echo_log_tail(log_path, 40);
    }

    Ok(outcome)
}

/// Print the last `lines` lines of a log file (best-effort).
fn echo_log_tail(log_path: &Path, lines: usize) {
    let Ok(file) = fs::File::open(log_path) else {
        return;
    };

    let all: Vec<String> = BufReader::new(file).lines().map_while(Result::ok).collect();

    eprintln!("---- tail of {} ----", log_path.display());
    for line in all.iter().skip(all.len().saturating_sub(lines)) {
        eprintln!("{line}");
    }
    eprintln!("---- end ----");
}

fn binary_exists(name: &str) -> bool {
    match Command::new(name)
        .arg("-v")
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
    {
        Ok(mut child) => {
            let _ = child.wait();
            true
        }
        Err(err) => err.kind() != std::io::ErrorKind::NotFound,
    }
}
