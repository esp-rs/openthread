//! A CLI simulation node: the Rust-platform counterpart of the upstream
//! `ot-cli-ftd <node id>` simulation binary.
//!
//! The full `openthread` stack runs on this crate's platform (embassy alarm,
//! tasklet pumping, software MAC) with the UDP-multicast [`SimRadio`] as its
//! 802.15.4 "RF" - and is driven exclusively through OpenThread's C CLI:
//! stdin lines go to the interpreter, its output goes to stdout. That is the
//! DUT shape the upstream test harness spawns (`OT_CLI_PATH`, a pty via
//! pexpect), making this binary its drop-in node.
//!
//! Invocation: `cli_node [-L<addr>] <node id>` - the upstream simulation
//! binaries' shape (`-L` selects the local interface address; the expect
//! suite always passes it). The port base of the simulated radio medium
//! comes from `PORT_BASE`/`PORT_OFFSET` (harness convention). Exits on stdin
//! EOF, like when the harness tears the pty down.
//!
//! The CLI `reset`/`factoryreset` commands are honored by re-executing the
//! process: the platform cannot reset the C stack in place (see the crate's
//! `otPlatReset`), while a re-exec keeps the pty/stdio fds - so the
//! harness's session survives - and starts a genuinely fresh stack. Settings
//! persist in a per-node file (see [`openthread_tests::settings`]) under
//! `$CLI_NODE_SETTINGS_DIR` (default: `tmp/` beneath the cwd, mirroring the
//! upstream simulation platform's flash files) - so `reset` keeps the
//! dataset, while `factoryreset` deletes the file before re-executing.

use std::io::{BufRead, IsTerminal, Write};
use std::net::Ipv4Addr;
use std::os::unix::process::CommandExt;

use embassy_executor::Spawner;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

use log::info;

use openthread::{EmbassyTimeTimer, MacRadio, MacRadioResources, OpenThread, OtResources};

use openthread_tests::executor::{self, Mode};
use openthread_tests::settings::FileSettings;
use openthread_tests::sim::SimRadio;
use openthread_tests::vt::{VtLink, VtRadio};

use rand::rngs::StdRng;
use rand::SeedableRng;

use static_cell::StaticCell;

// Linked for its `utoa`/`strtoul` C symbols, which OpenThread's C references.
use tinyrlibc as _;

/// Input lines on their way from the stdin reader thread to the embassy
/// executor (the CLI must run on the executor thread, where the OpenThread
/// singleton lives).
static INPUT: Channel<CriticalSectionRawMutex, String, 8> = Channel::new();

fn main() {
    let args = NodeArgs::parse();

    // The harness spawns ONE `OT_CLI_PATH` binary for every node, while a rig
    // may well be mixed - so this binary is also the dispatcher that looks at
    // the maps and picks the flavor each node needs:
    //
    // - an MCU node runs the stack as firmware, so hand the harness's pipes
    //   straight to `serial_bridge`, which pumps them to the board's console;
    // - with `OT_C_CLI_PATH` set (the xtask's `--peers c`), every node but
    //   the DUT execs the *upstream* C simulation binary - the DUT-vs-golden-
    //   devices shape, where any interop failure is the DUT's by definition.
    if let Some(node) = openthread_tests::hw::node_for(args.node_id) {
        match node.kind {
            openthread_tests::hw::NodeKind::Mcu => exec_serial_bridge(),
            openthread_tests::hw::NodeKind::CPosix => exec_posix_host(&node),
            openthread_tests::hw::NodeKind::Rcp => (),
        }
    }

    if let Ok(c_cli) = std::env::var("OT_C_CLI_PATH") {
        let dut: u16 = std::env::var("OT_DUT_NODE")
            .ok()
            .and_then(|node| node.parse().ok())
            .unwrap_or(1);

        if args.node_id != dut {
            exec_replacement(c_cli.as_ref());
        }
    }

    // Logs MUST NOT go where the CLI conversation runs: under thread-cert's
    // `PopenSpawn` even stderr is merged into the stream the harness parses,
    // so a stray log line can derail its line matching. With
    // `CLI_NODE_LOG=<path>` set, logs go to `<path>.<node id>` (one file per
    // node; level via `RUST_LOG` as usual); without it, when driven by a
    // harness (stdin is not a tty), logs are discarded outright. Only an
    // interactive (tty) session logs to stderr.
    let mut builder = env_logger::builder();
    builder
        .filter_level(log::LevelFilter::Warn)
        .parse_default_env();

    if let Ok(path) = std::env::var("CLI_NODE_LOG") {
        let file = std::fs::File::create(format!("{path}.{}", args.node_id))
            .expect("create CLI_NODE_LOG file");
        builder.target(env_logger::Target::Pipe(Box::new(file)));
    } else if !std::io::stdin().is_terminal() {
        builder.target(env_logger::Target::Pipe(Box::new(std::io::sink())));
    }

    // One exception to "logs never touch the CLI conversation": OpenThread's certification dumps
    // (the `[THCI]` MeshCoP blocks of reference-device builds, marked by the crate with the `[OpenThread-OUT]` prefix).
    // The cert harness reads exactly these from the node's console - upstream reference builds print them there too -
    // so tee them onto stdout, stripped back to their native form, while everything else follows the logger configuration above.
    struct TeeOtLogger {
        inner: env_logger::Logger,
    }

    impl log::Log for TeeOtLogger {
        fn enabled(&self, metadata: &log::Metadata) -> bool {
            metadata.level() <= log::Level::Info || self.inner.enabled(metadata)
        }

        fn log(&self, record: &log::Record) {
            let msg = record.args().to_string();

            if let Some(cert) = msg.strip_prefix("[OpenThread-OUT] ") {
                cli_output(format!("{cert}\r\n").as_bytes());
            }

            self.inner.log(record);
        }

        fn flush(&self) {
            self.inner.flush();
        }
    }

    let inner = builder.build();
    let max_level = inner.filter().max(log::LevelFilter::Info);

    log::set_boxed_logger(Box::new(TeeOtLogger { inner })).expect("install logger");
    log::set_max_level(max_level);

    // Route panics through the logger too. A node's stderr goes into the
    // harness's pexpect buffer, which it never writes anywhere - so by default
    // a panicking node just vanishes ("EOF" on the harness side) with no trace
    // of why. The per-node log file is the artifact that survives, so put the
    // panic there as well as on stderr.
    {
        let default_hook = std::panic::take_hook();

        std::panic::set_hook(Box::new(move |info| {
            log::error!("node panicked: {info}");
            default_hook(info);
        }));
    }

    std::thread::spawn(read_stdin);

    // Virtual-time mode when the harness says so (the env var is set for the
    // whole test run; simulation nodes inherit it). The event link doubles as
    // the executor's clock source and the radio's frame transport.
    let virtual_time = std::env::var("VIRTUAL_TIME").as_deref() == Ok("1");

    let (mode, radio_link) = if virtual_time {
        let link = VtLink::new(args.node_id).expect("bind simulator event link");
        (Mode::Virtual(link.clone()), Some(link))
    } else {
        (Mode::RealTime, None)
    };

    executor::run(mode, move |spawner| {
        spawner.spawn(main_task(spawner, args, radio_link).unwrap())
    });
}

/// The upstream simulation binaries' command line: `[-L<addr>] <node id>`.
#[derive(Clone, Copy)]
struct NodeArgs {
    node_id: u16,
    local: Ipv4Addr,
}

impl NodeArgs {
    fn parse() -> Self {
        let mut node_id = None;
        let mut local = Ipv4Addr::LOCALHOST;

        let mut args = std::env::args().skip(1);
        while let Some(arg) = args.next() {
            if let Some(addr) = arg.strip_prefix("-L") {
                let addr = if addr.is_empty() {
                    args.next().unwrap_or_default()
                } else {
                    addr.to_string()
                };
                local = addr.parse().expect("-L: not an IPv4 address");
            } else if arg.starts_with('-') {
                // Tolerate harness-passed options this node does not model,
                // so a harness update doesn't silently kill every node.
                eprintln!("cli_node: ignoring unsupported option `{arg}`");
            } else {
                node_id = Some(arg.parse().expect("node id: not a number"));
            }
        }

        Self {
            node_id: node_id.expect("usage: cli_node [-L<addr>] <node id>"),
            local,
        }
    }
}

/// Replace this process with `serial_bridge`, keeping the harness's pipes and
/// our command line (the bridge reads the same node id and port map).
fn exec_serial_bridge() -> ! {
    let bridge = std::env::current_exe()
        .ok()
        .and_then(|exe| Some(exe.parent()?.join("serial_bridge")))
        .expect("locating `serial_bridge` next to this binary");

    exec_replacement(&bridge)
}

/// Replace this process with the *upstream* posix host (`ot-cli`) driving
/// this node's co-processor: a golden reference node on real RF.
///
/// Unlike every other node flavor, the posix host takes a radio URL rather
/// than a node id, so the argv is rebuilt rather than passed through. The
/// binary comes from `OT_POSIX_CLI_PATH` (the xtask builds and exports it);
/// failing loudly beats quietly running the wrong implementation.
fn exec_posix_host(node: &openthread_tests::hw::Node) -> ! {
    let posix_cli = std::env::var("OT_POSIX_CLI_PATH").expect(
        "OT_POSIX_CLI_PATH is not set, but this node's board is `cposix` \
         (run through `cargo xtask itest`, which builds and exports it)",
    );

    let url = format!(
        "spinel+hdlc+uart://{}?uart-baudrate={}",
        node.device, node.baud
    );

    let err = std::process::Command::new(&posix_cli).arg(url).exec();

    panic!("exec {posix_cli}: {err}");
}

/// Replace this process with `binary`, keeping the harness's pipes and our
/// command line (every node flavor takes the same `[-L<addr>] <node id>`).
fn exec_replacement(binary: &std::path::Path) -> ! {
    let err = std::process::Command::new(binary)
        .args(std::env::args_os().skip(1))
        .exec();

    panic!("exec {}: {err}", binary.display());
}

/// Re-execute this process with its original command line: the `reset` /
/// `factoryreset` implementation (fresh stack, same pty/stdio fds).
///
/// The marker env var is how the next incarnation knows it is a reset
/// rather than a fresh spawn - the same role as the upstream simulation
/// platform's pseudo-reset flag: persisted settings survive only a reset
/// (a plain spawn starts factory-new, see `main`).
fn reexec() -> ! {
    let mut args = std::env::args_os();
    let argv0 = args.next().unwrap();

    let err = std::process::Command::new(argv0)
        .args(args)
        .env("CLI_NODE_PSEUDO_RESET", "1")
        .exec();

    panic!("re-exec for reset failed: {err}");
}

/// Pump stdin lines into `INPUT`; on EOF, exit the process (the harness closed
/// our terminal - upstream simulation binaries exit the same way).
fn read_stdin() {
    let stdin = std::io::stdin();

    for line in stdin.lock().lines() {
        let Ok(line) = line else {
            break;
        };

        let mut line = Some(line);
        while let Err(embassy_sync::channel::TrySendError::Full(rejected)) =
            INPUT.try_send(line.take().unwrap())
        {
            // Queue full: the executor is still draining earlier commands.
            line = Some(rejected);
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    }

    std::process::exit(0);
}

/// The CLI output sink: raw bytes to stdout, flushed per chunk (the harness
/// matches on partial lines, e.g. the `> ` prompt).
fn cli_output(output: &[u8]) {
    let mut stdout = std::io::stdout().lock();
    stdout.write_all(output).unwrap();
    stdout.flush().unwrap();
}

#[embassy_executor::task]
async fn main_task(spawner: Spawner, args: NodeArgs, radio_link: Option<VtLink>) {
    let node_id = args.node_id;

    info!("CLI simulation node {node_id} starting");

    static RNG: StaticCell<StdRng> = StaticCell::new();
    let rng = RNG.init(StdRng::from_os_rng());

    // Deterministic, node-unique EUI64 (the node id in the last two bytes).
    let mut ieee_eui64 = [0x18, 0xb4, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00];
    ieee_eui64[6..].copy_from_slice(&node_id.to_be_bytes());

    static OT_RESOURCES: StaticCell<OtResources> = StaticCell::new();
    static OT_SETTINGS: StaticCell<FileSettings> = StaticCell::new();

    // Per-node settings file: `reset` re-execs and the fresh process loads it
    // back; `factoryreset` deletes it below before re-executing.
    let settings_dir = std::path::PathBuf::from(
        std::env::var("CLI_NODE_SETTINGS_DIR").unwrap_or_else(|_| "tmp".into()),
    );
    std::fs::create_dir_all(&settings_dir).expect("create settings dir");
    let settings_path = settings_dir.join(format!("ot-settings-{node_id}.bin"));

    // A fresh spawn starts factory-new: a settings file of an earlier node
    // with the same id (e.g. a previous parametrized sub-test of the same
    // harness script) must not leak into this one. Settings survive only a
    // `reset` re-exec, which marks itself via the env var (the same
    // semantics as the upstream simulation platform's pseudo-reset).
    if std::env::var_os("CLI_NODE_PSEUDO_RESET").is_none() {
        let _ = std::fs::remove_file(&settings_path);
    }

    let ot_resources = OT_RESOURCES.init(OtResources::new());
    let ot_settings = OT_SETTINGS.init(FileSettings::new(settings_path.clone()));

    let ot = OpenThread::new(ieee_eui64, rng, ot_settings, ot_resources).unwrap();

    // A hardware run takes precedence over both simulated media: it must never
    // quietly degrade into a simulated one (see `openthread_tests::hw`).
    // An MCU node has already been handed off to `serial_bridge` in `main`, so
    // anything left here is an RCP node.
    let hw_node = openthread_tests::hw::node_for(node_id);

    #[cfg(not(feature = "hw"))]
    assert!(
        hw_node.is_none(),
        "this node's board is an RCP, but the DUT was built without the `hw` \
         feature; rebuild with `--features hw`"
    );

    match (hw_node, radio_link) {
        #[cfg(feature = "hw")]
        (Some(node), _) => {
            // A real co-processor owns the RF and the whole MAC, so - unlike
            // the simulation radios below - no `MacRadio` goes on top.
            let radio = openthread_tests::hw::radio(&node.device, node.baud);
            spawner.spawn(run_ot_hw(ot.clone(), radio).unwrap());
        }
        #[cfg(not(feature = "hw"))]
        (Some(_), _) => unreachable!("guarded by the assertion above"),
        // These simulation radios are PHY-only, so the runner tasks below wrap
        // them in a `MacRadio` - which emulates every MAC duty their reported
        // capabilities lack, i.e. all of them.
        (None, Some(link)) => {
            let radio = VtRadio::new(link);
            spawner.spawn(run_ot_vt(ot.clone(), radio).unwrap());
        }
        (None, None) => {
            let radio = SimRadio::new_with(
                node_id,
                openthread_tests::sim::port_base_from_env(),
                args.local,
            )
            .expect("create simulation radio");
            spawner.spawn(run_ot_rt(ot.clone(), radio).unwrap());
        }
    }

    ot.cli_init(cli_output);

    // The harness expects its command lines echoed back. On a pty (the
    // expect suite, THCI) the kernel's terminal echo provides that; on plain
    // pipes (thread-cert's PopenSpawn) the DUT must echo itself - which is
    // what the upstream CLI app's console layer does too.
    let echo = !std::io::stdin().is_terminal();

    loop {
        let line = INPUT.receive().await;

        if echo {
            cli_output(format!("{line}\r\n").as_bytes());
        }

        // A reset is a process re-exec (see the module docs); earlier
        // commands have all been processed at this point, matching the
        // sequential semantics of the real CLI. `factoryreset` additionally
        // wipes the persisted settings (the C stack never sees either
        // command, so its own settings-wipe path is not in play here).
        if matches!(line.trim(), "reset" | "factoryreset") {
            if line.trim() == "factoryreset" {
                let _ = std::fs::remove_file(&settings_path);
            }

            reexec();
        }

        // `exit` terminates the node - the upstream simulation binaries'
        // behavior, which the harness teardown relies on (it sends `exit`
        // and waits for EOF).
        if line.trim() == "exit" {
            std::process::exit(0);
        }

        if let Err(err) = ot.cli_input_line(&line) {
            // Over-long line; report like the CLI itself reports failures.
            cli_output(format!("Error {}: input line too long\r\n", err.into_inner()).as_bytes());
        }
    }
}

#[embassy_executor::task]
async fn run_ot_rt(ot: OpenThread<'static>, radio: SimRadio) -> ! {
    static MAC_RADIO_RESOURCES_RT: StaticCell<MacRadioResources> = StaticCell::new();
    let mac_radio_resources = MAC_RADIO_RESOURCES_RT.init(MacRadioResources::new());

    ot.run(MacRadio::new(radio, EmbassyTimeTimer, mac_radio_resources))
        .await
}

/// The hardware tier's runner: the co-processor reports a complete MAC
/// offload, so the radio goes to `OpenThread::run` as-is.
#[cfg(feature = "hw")]
#[embassy_executor::task]
async fn run_ot_hw(ot: OpenThread<'static>, radio: openthread_tests::hw::HwRadio) -> ! {
    ot.run(radio).await
}

#[embassy_executor::task]
async fn run_ot_vt(ot: OpenThread<'static>, radio: VtRadio) -> ! {
    static MAC_RADIO_RESOURCES_VT: StaticCell<MacRadioResources> = StaticCell::new();
    let mac_radio_resources = MAC_RADIO_RESOURCES_VT.init(MacRadioResources::new());

    ot.run(MacRadio::new(radio, EmbassyTimeTimer, mac_radio_resources))
        .await
}
