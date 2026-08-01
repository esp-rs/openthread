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
//! Invocation: `cli_ftd <node id>`; the port base of the simulated radio
//! medium comes from `PORT_BASE`/`PORT_OFFSET` (harness convention). Exits
//! on stdin EOF, like when the harness tears the pty down.

use std::io::{BufRead, Write};

use embassy_executor::{Executor, Spawner};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

use log::info;

use openthread::{EmbassyTimeTimer, MacRadio, OpenThread, OtResources, SimpleRamSettings};

use openthread_tests::sim_radio::SimRadio;

use rand::rngs::StdRng;
use rand::SeedableRng;

use static_cell::StaticCell;

// Linked for its `utoa`/`strtoul` C symbols, which OpenThread's C references.
use tinyrlibc as _;

/// Input lines on their way from the stdin reader thread to the embassy
/// executor (the CLI must run on the executor thread, where the OpenThread
/// singleton lives).
static INPUT: Channel<CriticalSectionRawMutex, String, 8> = Channel::new();

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .parse_default_env()
        .init();

    let node_id = std::env::args()
        .nth(1)
        .and_then(|arg| arg.parse::<u16>().ok())
        .expect("usage: cli_ftd <node id>");

    std::thread::spawn(read_stdin);

    let executor = EXECUTOR.init(Executor::new());
    executor.run(move |spawner| spawner.spawn(main_task(spawner, node_id).unwrap()));
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
async fn main_task(spawner: Spawner, node_id: u16) {
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

    let radio = MacRadio::new(
        SimRadio::new(node_id).expect("create simulation radio"),
        EmbassyTimeTimer,
    );

    spawner.spawn(run_ot(ot.clone(), radio).unwrap());

    ot.cli_init(cli_output);

    loop {
        let line = INPUT.receive().await;

        if let Err(err) = ot.cli_input_line(&line) {
            // Over-long line; report like the CLI itself reports failures.
            cli_output(format!("Error {}: input line too long\r\n", err.into_inner()).as_bytes());
        }
    }
}

#[embassy_executor::task]
async fn run_ot(ot: OpenThread<'static>, radio: MacRadio<SimRadio, EmbassyTimeTimer>) -> ! {
    ot.run(radio).await
}
