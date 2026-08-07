//! Bring-up probe for the hardware tier: talk to each RCP directly and say
//! what came back.
//!
//! The e2e suites are a bad first contact with a new rig - a radio that never
//! answers surfaces as a harness timeout several layers up, with the actual
//! cause buried in a per-node log. This binary skips all of that: it opens
//! each serial device, runs the radio's own `init` (the spinel
//! reset/capability handshake, nothing else) and prints the result. No
//! OpenThread instance, no network, no test.
//!
//! ```sh
//! cargo run --features hw --bin hw_probe -- /dev/ttyACM0 /dev/ttyUSB0@460800
//! ```
//!
//! Ports use the same `<device>[@<baud>]` syntax as `OT_HW_PORTS`, and with
//! no arguments the probe reads that variable - so exactly what a test run
//! would use can be probed first. A port whose given rate yields nothing is
//! retried at the other rate in common use, since a silent link is far more
//! often a baud mismatch than a dead board.
//!
//! Unlike a node - which owns exactly one radio for the life of the process,
//! and takes its buffers from statics - the probe walks several ports and
//! several rates, so it allocates (and deliberately leaks) a fresh set per
//! attempt.

use std::process::exit;

use embassy_futures::select::{select, Either};

use embassy_time::{Duration, Timer};

use openthread::spinel::{
    SerialPort, SpinelRadio, SpinelRadioResources, UartSpinelTransport, UartTransportResources,
};
use openthread::{Radio, RadioCaps};

use openthread_tests::executor::{self, Mode};
use openthread_tests::hw_radio;

// Linked for its `utoa`/`strtoul` C symbols, which OpenThread's C references.
use tinyrlibc as _;

/// The rates worth trying: stock firmware elsewhere, and ESP32xx RCPs.
const COMMON_BAUDS: [u32; 2] = [115_200, 460_800];

/// How long to give the co-processor to answer its startup handshake before
/// calling the link silent. Generous - the handshake is a couple of frames.
const HANDSHAKE_TIMEOUT: Duration = Duration::from_secs(5);

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .parse_default_env()
        .init();

    executor::run(Mode::RealTime, |spawner| {
        spawner.spawn(probe_all().unwrap())
    });
}

#[embassy_executor::task]
async fn probe_all() -> ! {
    let ports = ports();

    if ports.is_empty() {
        eprintln!(
            "usage: hw_probe <device>[@<baud>]...   (or set {})",
            hw_radio::PORTS_VAR,
        );
        exit(2);
    }

    let mut failed = 0;

    for (index, port) in ports.iter().enumerate() {
        // Node ids start at 1, so the position doubles as the node this port
        // would serve in a test run - which is how the suites' failures name
        // it ("the radio never came up on node.1").
        println!("node {} - {port}", index + 1);

        if !probe(port).await {
            failed += 1;
        }
    }

    exit(i32::from(failed != 0));
}

/// Probe one `<device>[@<baud>]` entry, reporting what happened.
///
/// Returns whether the radio came up.
async fn probe(port: &str) -> bool {
    let (device, given) = match port.rsplit_once('@') {
        Some((device, baud)) => match baud.parse::<u32>() {
            Ok(baud) => (device, Some(baud)),
            Err(_) => {
                println!("  `{baud}` is not a baud rate");
                return false;
            }
        },
        None => (port, None),
    };

    // The given rate first, then whatever else is in common use.
    let first = given.unwrap_or(hw_radio::DEFAULT_BAUD);
    let bauds: Vec<u32> = core::iter::once(first)
        .chain(COMMON_BAUDS.into_iter().filter(|baud| *baud != first))
        .collect();

    for baud in bauds {
        println!("  {baud} baud:");

        match try_init(device, baud).await {
            Ok(caps) => {
                println!("    OK");
                report(&caps);

                if given.is_some_and(|given| given != baud) {
                    println!("    NOTE: not the rate you gave - use `{device}@{baud}`");
                }

                return true;
            }
            Err(err) => println!("    {err}"),
        }
    }

    println!("    -> no answer at any rate. Check that {device} is an 802.15.4");
    println!("       co-processor running `ot-rcp`, that it is not held open by");
    println!("       another process, and that you can read/write it (dialout).");

    false
}

/// Open `device` and run the radio's startup handshake.
async fn try_init(device: &str, baud: u32) -> Result<RadioCaps, String> {
    let serial = SerialPort::open(device, baud).map_err(|err| format!("cannot open: {err}"))?;

    // Leaked on purpose: one set per attempt, and the process is short-lived.
    // (Typed, so the resources' default queue depth applies - the same one a
    // node gets.)
    let uart_resources: &'static mut UartTransportResources =
        Box::leak(Box::new(UartTransportResources::new()));
    let radio_resources: &'static mut SpinelRadioResources =
        Box::leak(Box::new(SpinelRadioResources::new()));

    let mut radio = SpinelRadio::new(
        UartSpinelTransport::new(serial, uart_resources),
        radio_resources,
    );

    // A co-processor that is not there (or not speaking at this rate) must
    // not wedge the probe.
    match select(radio.init(), Timer::after(HANDSHAKE_TIMEOUT)).await {
        Either::First(Ok(caps)) => Ok(caps),
        Either::First(Err(err)) => Err(format!("the co-processor rejected the handshake: {err:?}")),
        Either::Second(_) => Err("silent - no answer within 5s".to_string()),
    }
}

/// Print what the co-processor said it can do.
fn report(caps: &RadioCaps) {
    println!("    phy caps:  {:?}", caps.phy);
    println!("    mac caps:  {:?}", caps.mac);
    println!("    rx sens:   {} dBm", caps.receive_sensitivity);
    println!("    tx power:  {} dBm", caps.default_tx_power);
    println!("    cca thr:   {} dBm", caps.default_cca_threshold);
}

/// The ports to probe: the command line, else the port map a test run uses.
fn ports() -> Vec<String> {
    let args: Vec<String> = std::env::args().skip(1).collect();

    if !args.is_empty() {
        return args;
    }

    std::env::var(hw_radio::PORTS_VAR)
        .unwrap_or_default()
        .split(',')
        .map(str::trim)
        .filter(|port| !port.is_empty())
        .map(str::to_string)
        .collect()
}
