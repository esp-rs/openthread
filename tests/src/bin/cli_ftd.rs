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
//! Invocation: `cli_ftd [-L<addr>] <node id>` - the upstream simulation
//! binaries' shape (`-L` selects the local interface address; the expect
//! suite always passes it). The port base of the simulated radio medium
//! comes from `PORT_BASE`/`PORT_OFFSET` (harness convention). Exits on stdin
//! EOF, like when the harness tears the pty down.
//!
//! The CLI `reset`/`factoryreset` commands are honored by re-executing the
//! process: the platform cannot reset the C stack in place (see the crate's
//! `otPlatReset`), while a re-exec keeps the pty/stdio fds - so the
//! harness's session survives - and starts a genuinely fresh stack. Settings
//! live in RAM, so until a file-backed `Settings` exists, `reset` behaves
//! like `factoryreset` (no dataset survives).

use std::io::{BufRead, IsTerminal, Write};
use std::net::Ipv4Addr;
use std::os::unix::process::CommandExt;

use embassy_executor::Spawner;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

use log::info;

use openthread::{EmbassyTimeTimer, MacRadio, OpenThread, OtResources, SimpleRamSettings};

use openthread_tests::executor::{self, Mode};
use openthread_tests::sim_radio::SimRadio;
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

    // Logs MUST NOT go where the CLI conversation runs: under thread-cert's
    // `PopenSpawn` even stderr is merged into the stream the harness parses,
    // so a stray log line can derail its line matching. With
    // `CLI_FTD_LOG=<path>` set, logs go to `<path>.<node id>` (one file per
    // node; level via `RUST_LOG` as usual); without it, when driven by a
    // harness (stdin is not a tty), logs are discarded outright. Only an
    // interactive (tty) session logs to stderr.
    let mut builder = env_logger::builder();
    builder
        .filter_level(log::LevelFilter::Warn)
        .parse_default_env();

    if let Ok(path) = std::env::var("CLI_FTD_LOG") {
        let file = std::fs::File::create(format!("{path}.{}", args.node_id))
            .expect("create CLI_FTD_LOG file");
        builder.target(env_logger::Target::Pipe(Box::new(file)));
    } else if !std::io::stdin().is_terminal() {
        builder.target(env_logger::Target::Pipe(Box::new(std::io::sink())));
    }

    builder.init();

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
                eprintln!("cli_ftd: ignoring unsupported option `{arg}`");
            } else {
                node_id = Some(arg.parse().expect("node id: not a number"));
            }
        }

        Self {
            node_id: node_id.expect("usage: cli_ftd [-L<addr>] <node id>"),
            local,
        }
    }
}

/// Re-execute this process with its original command line: the `reset` /
/// `factoryreset` implementation (fresh stack, same pty/stdio fds).
fn reexec() -> ! {
    let mut args = std::env::args_os();
    let argv0 = args.next().unwrap();

    let err = std::process::Command::new(argv0).args(args).exec();
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
    static OT_SETTINGS_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    static OT_SETTINGS: StaticCell<SimpleRamSettings> = StaticCell::new();

    let ot_resources = OT_RESOURCES.init(OtResources::new());
    let ot_settings_buf = OT_SETTINGS_BUF.init([0; 1024]);
    let ot_settings = OT_SETTINGS.init(SimpleRamSettings::new(ot_settings_buf));

    let ot = OpenThread::new(ieee_eui64, rng, ot_settings, ot_resources).unwrap();

    match radio_link {
        Some(link) => {
            let radio = MacRadio::new(VtRadio::new(link), EmbassyTimeTimer);
            spawner.spawn(run_ot_vt(ot.clone(), radio).unwrap());
        }
        None => {
            let radio = MacRadio::new(
                SimRadio::new_with(
                    node_id,
                    openthread_tests::sim_radio::port_base_from_env(),
                    args.local,
                )
                .expect("create simulation radio"),
                EmbassyTimeTimer,
            );
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
        // sequential semantics of the real CLI.
        if matches!(line.trim(), "reset" | "factoryreset") {
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
async fn run_ot_rt(ot: OpenThread<'static>, radio: MacRadio<SimRadio, EmbassyTimeTimer>) -> ! {
    ot.run(radio).await
}

#[embassy_executor::task]
async fn run_ot_vt(ot: OpenThread<'static>, radio: MacRadio<VtRadio, EmbassyTimeTimer>) -> ! {
    ot.run(radio).await
}
