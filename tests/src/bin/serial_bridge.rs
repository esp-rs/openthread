//! The HIL tier's node: a device running the `openthread` stack as *firmware*,
//! reached over its serial console.
//!
//! This binary is what `OT_CLI_PATH` points at for that tier. It is not a node
//! itself - it owns no stack and no radio. It maps the node id the harness
//! spawns it with to a serial device, and then gets out of the way, piping
//! stdin to the device and the device's output back to stdout. The harness
//! cannot tell the difference between this and a node that runs in-process,
//! which is the whole trick: the same unmodified upstream scenarios drive
//! firmware on real silicon.
//!
//! ```text
//!   harness --spawns--> serial_bridge <node id>
//!                          |  stdin/stdout
//!                          +--serial--> [MCU: openthread + CLI + NrfRadio/EspRadio]
//!                                                    ~)) real RF
//! ```
//!
//! # Why this tier exists
//!
//! The RCP tier (see [`openthread_tests::hw_radio`]) puts real RF under the
//! stack, but the radio it drives is a co-processor running someone else's
//! firmware, and the stack still runs on a host. This tier is the only one
//! where the crate's own radio drivers (`NrfRadio`, `EspRadio`, or any
//! user-provided `Radio`), the `MacRadio` software MAC with its real ACK
//! deadlines, and the `ProxyRadio` executor split all run where they are meant
//! to - on the MCU, against a real clock.
//!
//! # Port map
//!
//! The same `OT_HW_PORTS` map as the RCP tier, with the same
//! `<device>[@<baud>]` syntax, indexed by node id. A rig may mix the two tiers
//! freely - the `xtask` decides which binary to spawn per node - and mixing is
//! usually what you want: pairing the device under test against a known-good
//! RCP node isolates a failure to the device.
//!
//! # Fresh state
//!
//! The suites assume a factory-fresh node per test. A host node gets that for
//! free (a new process); firmware does not, since the device has been running
//! since the last test. So on startup the bridge issues the reset command in
//! `OT_HW_RESET_CMD` (default `factoryreset`, the CLI's own wipe-and-restart)
//! and waits for the prompt to come back. Set it empty to skip - useful when
//! attaching to a device mid-run to watch it.

use std::io::{ErrorKind, Read, Write};
use std::os::fd::OwnedFd;
use std::process::exit;
use std::time::{Duration, Instant};

use nix::fcntl::{open, OFlag};
use nix::sys::stat::Mode;
use nix::sys::termios::{
    cfmakeraw, cfsetispeed, cfsetospeed, tcgetattr, tcsetattr, BaudRate, SetArg,
};

/// The environment variable overriding the startup reset command.
const RESET_VAR: &str = "OT_HW_RESET_CMD";

/// What to send at startup to get a factory-fresh node.
const DEFAULT_RESET_CMD: &str = "factoryreset";

/// How long to wait for the device's prompt after the startup reset before
/// giving up and going transparent anyway (the harness will report the real
/// problem better than we can).
const PROMPT_TIMEOUT: Duration = Duration::from_secs(10);

fn main() {
    let node_id = node_id();

    let node = openthread_tests::hw_radio::node_for(node_id).unwrap_or_else(|| {
        eprintln!(
            "serial_bridge: {} is not set; there is no board to bridge to",
            openthread_tests::hw_radio::PORTS_VAR,
        );
        exit(2);
    });

    let device = node.device;

    let tty = open_raw(&device, node.baud).unwrap_or_else(|err| {
        eprintln!("serial_bridge: cannot open {device}: {err}");
        exit(2);
    });

    let mut to_device = std::fs::File::from(
        tty.try_clone()
            .unwrap_or_else(|err| fatal(&format!("cannot dup {device}: {err}"))),
    );
    let mut from_device = std::fs::File::from(tty);

    reset_device(&mut to_device, &mut from_device);

    // Device -> stdout on its own thread; the main thread pumps the other way.
    // Either direction ending ends the process: the harness closing our stdin
    // is how it tears a node down, and a device that stops talking is a
    // failure the harness must see as EOF.
    std::thread::spawn(move || {
        let mut stdout = std::io::stdout();
        let mut buf = [0; 512];

        loop {
            match from_device.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => {
                    if stdout.write_all(&buf[..n]).is_err() || stdout.flush().is_err() {
                        break;
                    }
                }
                Err(err) if err.kind() == ErrorKind::Interrupted => (),
                Err(_) => break,
            }
        }

        exit(0);
    });

    let mut stdin = std::io::stdin();
    let mut buf = [0; 512];

    loop {
        match stdin.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => {
                if to_device.write_all(&buf[..n]).is_err() || to_device.flush().is_err() {
                    break;
                }
            }
            Err(err) if err.kind() == ErrorKind::Interrupted => (),
            Err(_) => break,
        }
    }
}

/// Put the device back into a factory-fresh state, and wait until its CLI is
/// answering again.
///
/// Also serves as the startup handshake in the no-reset case: the harness
/// expects a prompt within a tenth of a second of spawning us, and a device
/// that has merely been sitting idle will not volunteer one - so something has
/// to prod it.
fn reset_device(to_device: &mut std::fs::File, from_device: &mut std::fs::File) {
    let command = std::env::var(RESET_VAR).unwrap_or_else(|_| DEFAULT_RESET_CMD.to_string());

    if to_device
        .write_all(format!("{command}\r\n").as_bytes())
        .and_then(|()| to_device.flush())
        .is_err()
    {
        return;
    }

    // Read until the prompt reappears, discarding the reset's own banner - the
    // harness has not started listening yet, and a boot banner in its stream
    // would derail its line matching.
    let deadline = Instant::now() + PROMPT_TIMEOUT;
    let mut seen = Vec::new();
    let mut buf = [0; 256];

    while Instant::now() < deadline {
        match from_device.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => {
                seen.extend_from_slice(&buf[..n]);

                if seen.ends_with(b"> ") {
                    return;
                }

                // Bound the memory a chatty boot can cost us.
                if seen.len() > 8192 {
                    seen.drain(..seen.len() - 2);
                }
            }
            Err(err) if err.kind() == ErrorKind::Interrupted => (),
            Err(_) => break,
        }
    }
}

/// Open `device` as a raw, non-canonical tty at `baud` - the console the
/// firmware's CLI speaks on.
fn open_raw(device: &str, baud: u32) -> Result<OwnedFd, nix::Error> {
    let fd = open(device, OFlag::O_RDWR | OFlag::O_NOCTTY, Mode::empty())?;

    let mut termios = tcgetattr(&fd)?;
    cfmakeraw(&mut termios);

    // A USB CDC console ignores the rate entirely; it only matters behind a
    // real UART bridge, so an unrepresentable rate is not worth failing over.
    if let Some(baud) = baud_rate(baud) {
        cfsetispeed(&mut termios, baud)?;
        cfsetospeed(&mut termios, baud)?;
    } else {
        eprintln!("serial_bridge: {baud} is not a standard baud rate; leaving the tty's own");
    }

    tcsetattr(&fd, SetArg::TCSANOW, &termios)?;

    Ok(fd)
}

/// The `termios` constant for a rate, if it is one of the standard ones.
fn baud_rate(baud: u32) -> Option<BaudRate> {
    Some(match baud {
        9600 => BaudRate::B9600,
        19200 => BaudRate::B19200,
        38400 => BaudRate::B38400,
        57600 => BaudRate::B57600,
        115_200 => BaudRate::B115200,
        230_400 => BaudRate::B230400,
        460_800 => BaudRate::B460800,
        921_600 => BaudRate::B921600,
        _ => return None,
    })
}

/// The upstream simulation binaries' command line: `[-L<addr>] <node id>`.
/// Only the node id matters here - the rest is accepted and ignored, as the
/// harness passes it unconditionally.
fn node_id() -> u16 {
    let mut node_id = None;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg.starts_with("-L") {
            if arg == "-L" {
                args.next();
            }
        } else if !arg.starts_with('-') {
            node_id = arg.parse().ok();
        }
    }

    node_id.unwrap_or_else(|| fatal("usage: serial_bridge [-L<addr>] <node id>"))
}

fn fatal(message: &str) -> ! {
    eprintln!("serial_bridge: {message}");
    exit(2);
}
