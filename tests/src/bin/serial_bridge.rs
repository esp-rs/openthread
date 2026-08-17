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
//! The RCP tier (see [`openthread_tests::hw`]) puts real RF under the
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
//!
//! # Devices that come and go
//!
//! On a board whose console is its own USB peripheral (a XIAO nRF52840, say),
//! a chip reset takes the console down with it: the device drops off the bus
//! and re-enumerates a moment later. The harness neither knows nor cares - it
//! sent `reset` and keeps talking to the same bridge - so the bridge absorbs
//! the gap: when the device side errors, it reopens the device path (same
//! `by-id` symlink, fresh tty) and carries on. Whatever the harness writes
//! meanwhile waits in the bridge's stdin pipe rather than being lost, because
//! the bridge only reads stdin while it has a device to forward to. Only a
//! device that stays gone past [`RECONNECT_TIMEOUT`] ends the bridge - and
//! with it, visibly, the test.

use std::io::{ErrorKind, Read, Write};
use std::os::fd::{AsFd, OwnedFd};
use std::process::exit;
use std::time::{Duration, Instant};

use nix::fcntl::{open, OFlag};
use nix::poll::{poll, PollFd, PollFlags, PollTimeout};
use nix::sys::stat::Mode;
use nix::sys::termios::{
    cfmakeraw, cfsetispeed, cfsetospeed, tcgetattr, tcsetattr, BaudRate, SetArg,
};

/// The environment variable overriding the startup reset command.
const RESET_VAR: &str = "OT_HW_RESET_CMD";

/// If set, a directory where the bridge tees the raw device traffic, one
/// timestamped line per chunk, both directions - the ground truth for "when
/// did the device actually say/hear that" when a harness timeout needs to be
/// attributed to the device or to the host side.
const CAPTURE_VAR: &str = "OT_BRIDGE_CAPTURE";

/// What to send at startup to get a factory-fresh node.
const DEFAULT_RESET_CMD: &str = "factoryreset";

/// How long to wait for the device's prompt after the startup reset before
/// giving up and going transparent anyway (the harness will report the real
/// problem better than we can).
const PROMPT_TIMEOUT: Duration = Duration::from_secs(10);

/// How long a vanished device gets to re-enumerate before the bridge gives
/// up and exits. Covers a chip reset plus USB re-enumeration with a wide
/// margin; a device still gone after this is not rebooting, it is dead.
const RECONNECT_TIMEOUT: Duration = Duration::from_secs(15);

/// The device end of the bridge: the console tty, plus what it takes to get
/// it back when a chip reset drops it off the bus.
struct Device {
    fd: OwnedFd,
    path: String,
    baud: u32,
    capture: Option<(std::fs::File, Instant)>,
    /// A reconnect happened since the last [`Device::take_reconnected`] -
    /// which is how `reset_device` learns that the reset it is waiting out
    /// has already run its course (reboot, re-enumeration, scrub).
    reconnected: bool,
}

impl Device {
    fn open(path: &str, baud: u32) -> Result<Self, nix::Error> {
        Ok(Self {
            fd: open_raw(path, baud)?,
            path: path.to_string(),
            baud,
            capture: None,
            reconnected: false,
        })
    }

    /// Append a `[+seconds] <dir> <printable-chunk>` line to the capture.
    fn capture(&mut self, dir: char, chunk: &[u8]) {
        if let Some((file, start)) = self.capture.as_mut() {
            let _ = writeln!(
                file,
                "[{:10.3}] {dir} {}",
                start.elapsed().as_secs_f64(),
                String::from_utf8_lossy(chunk).escape_debug(),
            );
        }
    }

    /// Re-open the device path after the device dropped off the bus,
    /// retrying until it re-enumerates or [`RECONNECT_TIMEOUT`] runs out.
    fn reconnect(&mut self) {
        self.capture('!', b"(device lost; reconnecting)");

        let deadline = Instant::now() + RECONNECT_TIMEOUT;

        loop {
            if Instant::now() >= deadline {
                fatal(&format!("{} did not come back", self.path));
            }

            std::thread::sleep(Duration::from_millis(200));

            if let Ok(fd) = open_raw(&self.path, self.baud) {
                self.fd = fd;
                self.capture('!', b"(reconnected)");
                break;
            }
        }

        self.scrub();
        self.reconnected = true;
    }

    /// Whether a reconnect happened since the last call.
    fn take_reconnected(&mut self) -> bool {
        core::mem::take(&mut self.reconnected)
    }

    /// Clean up after a re-enumeration, on both sides of the wire.
    ///
    /// A freshly enumerated tty starts out with the default line discipline -
    /// echo included - and whoever opens it first (this bridge racing its own
    /// `tcsetattr`, or any system service taking a look) reflects the boot
    /// banner back at the device, where it lodges in the CLI's line buffer,
    /// spawns stray `Error 35` lines as its embedded terminators trickle
    /// through, and garbles the next real command. The reflections keep
    /// arriving for a while, so one flush is not enough - scrub until the
    /// console is *provably* clean: the wire has gone quiet, and a flushed
    /// bare newline provokes no output (an empty line yields none; leftover
    /// junk yields an error and a prompt, and another round).
    fn scrub(&mut self) {
        for _ in 0..3 {
            self.discard_until_quiet(Duration::from_millis(500), Duration::from_secs(3));

            let _ = nix::unistd::write(self.fd.as_fd(), b"\n");

            if !self.discard_until_quiet(Duration::from_millis(400), Duration::from_secs(1)) {
                return;
            }
        }
    }

    /// Read and discard device output until `quiet` of continuous silence
    /// (bounded by `cap`); whether anything arrived at all.
    fn discard_until_quiet(&mut self, quiet: Duration, cap: Duration) -> bool {
        let deadline = Instant::now() + cap;
        let mut buf = [0; 256];
        let mut any = false;

        while Instant::now() < deadline {
            if !wait(&self.fd, PollFlags::POLLIN, Some(quiet)) {
                break;
            }

            let Ok(n) = nix::unistd::read(&self.fd, &mut buf) else {
                break;
            };
            if n == 0 {
                break;
            }

            self.capture('~', &buf[..n]);
            any = true;
        }

        any
    }

    /// Read what the device has, into `buf`: `Some(n)` bytes, or `None` if
    /// nothing is pending. A device error means the device dropped off the
    /// bus: reconnects (or dies) instead of returning.
    fn read(&mut self, buf: &mut [u8]) -> Option<usize> {
        if !wait(&self.fd, PollFlags::POLLIN, Some(Duration::ZERO)) {
            return None;
        }

        match nix::unistd::read(&self.fd, buf) {
            Ok(0) | Err(_) => {
                self.reconnect();
                None
            }
            Ok(n) => {
                self.capture('<', &buf[..n]);
                Some(n)
            }
        }
    }

    /// Write all of `chunk` to the device, reconnecting (and re-sending the
    /// remainder) if the device drops mid-write.
    fn write(&mut self, chunk: &[u8]) {
        self.capture('>', chunk);

        let mut pos = 0;
        while pos < chunk.len() {
            match nix::unistd::write(self.fd.as_fd(), &chunk[pos..]) {
                Ok(n) => pos += n,
                Err(nix::Error::EINTR) => (),
                Err(nix::Error::EAGAIN) => {
                    wait(&self.fd, PollFlags::POLLOUT, Some(Duration::from_secs(1)));
                }
                Err(_) => self.reconnect(),
            }
        }
    }
}

fn main() {
    let node_id = node_id();

    let node = openthread_tests::hw::node_for(node_id).unwrap_or_else(|| {
        eprintln!(
            "serial_bridge: {} is not set; there is no board to bridge to",
            openthread_tests::hw::PORTS_VAR,
        );
        exit(2);
    });

    let mut device = Device::open(&node.device, node.baud).unwrap_or_else(|err| {
        eprintln!("serial_bridge: cannot open {}: {err}", node.device);
        exit(2);
    });

    if let Ok(dir) = std::env::var(CAPTURE_VAR) {
        let path = format!("{dir}/bridge-{node_id}.log");
        match std::fs::File::create(&path) {
            Ok(file) => device.capture = Some((file, Instant::now())),
            Err(err) => eprintln!("serial_bridge: cannot create {path}: {err}"),
        }
    }

    reset_device(&mut device);

    pump(device)
}

/// The bridge's steady state: stdin to the device, the device to stdout,
/// until the harness hangs up or the device stays gone.
///
/// Single-threaded around `poll`, because the interesting events - harness
/// EOF, device loss - need one place to decide what happens next. While the
/// device is being reconnected nothing reads stdin, so harness commands sent
/// across a reboot queue in the pipe instead of vanishing.
fn pump(mut device: Device) -> ! {
    let stdin = std::io::stdin();
    let mut stdout = std::io::stdout();
    let mut buf = [0; 512];
    // Assembled from the stdin stream to spot the `exit` command - the only
    // line the bridge answers itself. The upstream simulation binaries
    // terminate on it (the harness sends it at teardown and waits for EOF);
    // firmware has no process to exit, so the BRIDGE is what must go away.
    // The board keeps running - the next run's startup reset renews it.
    let mut line: Vec<u8> = Vec::new();

    loop {
        // Device output first: it is the latency-sensitive direction (the
        // harness is timing its reads), and draining it keeps the poll below
        // honest.
        while let Some(n) = device.read(&mut buf) {
            if stdout.write_all(&buf[..n]).is_err() || stdout.flush().is_err() {
                // The harness hung up on us mid-stream.
                exit(0);
            }
        }

        // Nothing pending on the device: sleep until either side stirs.
        let stdin_ready = {
            let stdin_fd = stdin.as_fd();
            let device_fd = device.fd.as_fd();
            let mut fds = [
                PollFd::new(stdin_fd, PollFlags::POLLIN),
                PollFd::new(device_fd, PollFlags::POLLIN),
            ];

            let _ = poll(&mut fds, PollTimeout::from(1000u16));

            fds[0]
                .revents()
                .map(|r| r.intersects(PollFlags::POLLIN | PollFlags::POLLHUP))
                .unwrap_or(false)
        };

        if stdin_ready {
            let n = match stdin.lock().read(&mut buf) {
                Ok(0) => exit(0),
                Ok(n) => n,
                Err(err) if err.kind() == ErrorKind::Interrupted => continue,
                Err(_) => exit(0),
            };

            device.write(&buf[..n]);

            for byte in &buf[..n] {
                if *byte == b'\n' {
                    if line.trim_ascii() == b"exit" {
                        exit(0);
                    }
                    line.clear();
                } else {
                    line.push(*byte);
                }
            }
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
fn reset_device(device: &mut Device) {
    let command = std::env::var(RESET_VAR).unwrap_or_else(|_| DEFAULT_RESET_CMD.to_string());

    device.write(format!("{command}\r\n").as_bytes());

    // Read until the prompt reappears, discarding the reset's own banner - the
    // harness has not started listening yet, and a boot banner in its stream
    // would derail its line matching. A device whose console is USB drops off
    // the bus instead: `Device::read` comes back from that with the reboot
    // absorbed and the console scrubbed, which IS the prompt state.
    let deadline = Instant::now() + PROMPT_TIMEOUT;
    let mut seen = Vec::new();
    let mut buf = [0; 256];

    while Instant::now() < deadline {
        match device.read(&mut buf) {
            Some(n) => {
                seen.extend_from_slice(&buf[..n]);

                if seen.ends_with(b"> ") {
                    return;
                }

                // Bound the memory a chatty boot can cost us.
                if seen.len() > 8192 {
                    seen.drain(..seen.len() - 2);
                }
            }
            None => {
                if device.take_reconnected() {
                    return;
                }

                wait(
                    &device.fd,
                    PollFlags::POLLIN,
                    Some(Duration::from_millis(100)),
                );
            }
        }
    }
}

/// Poll `fd` for `events`; whether any arrived within `timeout` (`None` =
/// forever).
fn wait(fd: &OwnedFd, events: PollFlags, timeout: Option<Duration>) -> bool {
    let timeout = timeout
        .map(|t| PollTimeout::try_from(t.as_millis()).unwrap_or(PollTimeout::MAX))
        .unwrap_or(PollTimeout::NONE);

    let mut fds = [PollFd::new(fd.as_fd(), events)];

    matches!(poll(&mut fds, timeout), Ok(n) if n > 0)
}

/// Open `device` as a raw, non-canonical tty at `baud` - the console the
/// firmware's CLI speaks on.
///
/// Non-blocking: reads are `poll`-paced (see [`wait`]), and a blocking open
/// or read could otherwise hang on a half-dead device with no way to honor a
/// deadline.
fn open_raw(device: &str, baud: u32) -> Result<OwnedFd, nix::Error> {
    let fd = open(
        device,
        OFlag::O_RDWR | OFlag::O_NOCTTY | OFlag::O_NONBLOCK,
        Mode::empty(),
    )?;

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
