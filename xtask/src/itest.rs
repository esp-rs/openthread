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
//! [`EXPECT_TESTS`]): the upstream corpus assumes capabilities the DUT does
//! not have yet (virtual time, persistent settings across `reset`, MTD-only
//! builds, posix/RCP nodes, diag commands), so tests are enabled as they are
//! verified, with the reason for exclusion documented next to the list.

use std::fs;
use std::io::{BufRead, BufReader};
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};

use log::info;

/// The `thread-cert` tests run by default: verified green against the DUT in
/// real-time mode. Notes on the corpus:
/// - Tests calling `node.reset()`/`factory_reset()` mid-run (e.g. the
///   `Cert_5_5_*` reboot group) need settings to survive a reset, which the
///   RAM-settings DUT cannot offer yet.
/// - Tests with large topologies (16 routers) or long attach cascades are
///   excluded for wall-clock reasons only - real time runs at 1x.
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
    // NOT enabled - the known-marginal near-misses, for the record:
    // - Cert_5_3_04_AddressMapCache: an SED-originated ping races the SED
    //   poll latency against the ping deadline; deterministic under virtual
    //   time, marginal in real time. Revisit with virtual-time support.
];

/// The `expect` tests run by default.
///
/// PROVISIONAL: the list compiles from inspection (FTD-only, no posix/RCP
/// nodes, no diag), but has not run against the DUT yet - the `expect`
/// binary is not present on the development host. Verify and prune when
/// enabling in CI (`apt-get install expect`).
const EXPECT_TESTS: &[&str] = &[
    "cli-dataset",
    "cli-networkname",
    "cli-extaddr",
    "cli-counters",
    "cli-ping",
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

    /// Skip (re)building the DUT binaries.
    #[arg(long)]
    skip_build: bool,

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
    fs::create_dir_all(&build_dir)
        .with_context(|| format!("creating {}", build_dir.display()))?;

    let cli_ftd = build_dut(workspace, args.skip_build)?;

    let tests: Vec<String> = if args.tests.is_empty() {
        let defaults = match args.suite {
            Suite::Cert => CERT_TESTS,
            Suite::Expect => EXPECT_TESTS,
        };
        defaults.iter().map(|t| t.to_string()).collect()
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
            Suite::Cert => run_cert_test(&ot_root, &build_dir, &cli_ftd, test, index)?,
            Suite::Expect => run_expect_test(&ot_root, &build_dir, &cli_ftd, test)?,
        };

        match &outcome {
            Outcome::Passed => info!("{test}: PASSED"),
            Outcome::Skipped => info!("{test}: SKIPPED"),
            Outcome::Failed(reason) => info!("{test}: FAILED ({reason})"),
        }

        results.push((test.clone(), outcome));
    }

    let failed: Vec<&str> = results
        .iter()
        .filter_map(|(test, outcome)| matches!(outcome, Outcome::Failed(_)).then_some(&**test))
        .collect();

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

/// Build the DUT binaries (the `openthread-tests` crate is intentionally
/// outside the workspace, like `examples`) and return the `cli_ftd` path.
fn build_dut(workspace: &Path, skip_build: bool) -> Result<PathBuf> {
    let tests_crate = workspace.join("tests");

    if !skip_build {
        info!("Building the DUT binaries (openthread-tests)");

        let cargo = std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
        let status = Command::new(cargo)
            .arg("build")
            .arg("--bins")
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

fn run_cert_test(
    ot_root: &Path,
    build_dir: &Path,
    cli_ftd: &Path,
    test: &str,
    index: usize,
) -> Result<Outcome> {
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
        // Real time; the DUT has no virtual-time event support (yet).
        .env("VIRTUAL_TIME", "0")
        // Matches the wrapped OpenThread's `OT_THREAD_VERSION` - and keeps
        // node.py off its 1.1-compatibility binary paths.
        .env("THREAD_VERSION", "1.4")
        // Distinct radio medium per test, so a straggler node of a previous
        // test cannot inject frames into this one.
        .env("PORT_OFFSET", (index % 10).to_string())
        // Per-node DUT log files in the run dir (`node.<id>`); the level
        // comes from `RUST_LOG`, so `RUST_LOG=openthread=debug cargo xtask
        // itest <test>` captures a failing node's stack-side view.
        .env("CLI_FTD_LOG", run_dir.join("node"));

    run_logged(command, &run_dir.join("output.log"), test)
}

fn run_expect_test(
    ot_root: &Path,
    build_dir: &Path,
    cli_ftd: &Path,
    test: &str,
) -> Result<Outcome> {
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
        .current_dir(&run_dir)
        .env("OT_SIMULATION_APPS", &apps);

    run_logged(command, &run_dir.join("output.log"), test)
}

/// Run a test command with its output captured to `log_path`, a wall-clock
/// timeout, and the upstream exit-77-means-skip convention. On failure, the
/// log tail is echoed for immediate diagnosis.
fn run_logged(mut command: Command, log_path: &Path, test: &str) -> Result<Outcome> {
    let log = fs::File::create(log_path)
        .with_context(|| format!("creating {}", log_path.display()))?;

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

    let timeout = test_timeout(test);
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
    let _ = Command::new("kill")
        .arg("-9")
        .arg(format!("-{}", child.id()))
        .stderr(Stdio::null())
        .status();

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
