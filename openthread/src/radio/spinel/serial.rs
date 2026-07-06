//! [`SerialPort`]: a host (`std`) async serial byte stream over a `/dev/tty*`
//! device, for driving an `ot-rcp` from a Linux/macOS host over USB.
//!
//! `SerialPort` implements [`embedded_io_async::Read`] + [`embedded_io_async::Write`],
//! so it plugs straight into [`UartSpinelTransport`](super::UartSpinelTransport)
//! (which adds the HDLC framing the UART wire needs):
//!
//! ```ignore
//! use openthread::radio::spinel::{SerialPort, SpinelRadio, UartSpinelTransport};
//!
//! let serial = SerialPort::open("/dev/ttyUSB0", 115_200)?;
//! let radio = SpinelRadio::new(UartSpinelTransport::new(serial));
//! ot.run(radio).await
//! ```
//!
//! # The drain thread
//!
//! The tty is not read directly: a background thread drains it continuously
//! into a pipe, and the async reader consumes the pipe. This mirrors what
//! OpenThread's POSIX host does (its mainloop always polls the RCP fd) and is
//! a *requirement* for some RCP transports, not an optimization: USB-CDC
//! bridges with shallow device-side FIFOs — most notably the ESP32XX
//! USB-Serial-JTAG (64-byte FIFO) — stall the RCP firmware's transmit path
//! when the host stops reading, and the RCP can wedge beyond recovery (chip
//! reset required). The [`SpinelRadio`](super::SpinelRadio) driver reads in
//! bursts around its command exchanges, so without the drain thread even
//! sub-millisecond read gaps under inbound traffic can be fatal on such
//! transports. The pipe (capped at [`PIPE_CAPACITY`]) also absorbs inbound
//! bursts, complementing the driver's own RX-frame queue.
//!
//! # Platform support
//!
//! Currently **Unix only** (`#[cfg(unix)]`). The async I/O uses `async-io`'s
//! readiness-based reactor, which fits a Unix tty file descriptor but not a
//! Windows serial handle (those need overlapped/IOCP I/O, a different path). A
//! Windows implementation behind the same `std` feature is a planned addition.

#![cfg(unix)]

// The crate is `#![no_std]`; this module (gated on the `std` feature) opts back
// into `std`.
extern crate std;

use std::io;
use std::os::fd::{AsFd, OwnedFd};
use std::path::Path;

use async_io::Async;

use nix::fcntl::{open, OFlag};
use nix::poll::{poll, PollFd, PollFlags, PollTimeout};
use nix::sys::stat::Mode;
use nix::sys::termios::{
    cfmakeraw, cfsetspeed, tcsetattr, BaudRate, ControlFlags, InputFlags, SetArg,
};
use nix::unistd::pipe;

/// The drain-pipe capacity requested from the kernel (Linux; rounded up to
/// pages). Sized for a couple of seconds of a saturated 250 kbps 802.15.4
/// link — enough to absorb an inbound burst across the driver's longest
/// command exchange, small enough not to build a deep standing queue under
/// overload.
const PIPE_CAPACITY: usize = 8192;

/// An async serial port over a `/dev/tty*` device (Unix host).
///
/// Configured for raw, 8N1, no-flow-control operation — the mode an `ot-rcp`
/// expects. Reads are served from a pipe fed by a background thread that
/// drains the tty continuously (see the [module docs](self) for why that is
/// required). Wrap it in a
/// [`UartSpinelTransport`](super::UartSpinelTransport) for use with
/// [`SpinelRadio`](super::SpinelRadio).
pub struct SerialPort {
    /// The tty, written to directly.
    tty: Async<OwnedFd>,
    /// Read end of the drain pipe; the drain thread owns the write end and
    /// exits once this is closed (i.e. when the `SerialPort` is dropped).
    rx: Async<OwnedFd>,
}

impl SerialPort {
    /// Open and configure the serial device at `path` (e.g. `/dev/ttyUSB0`) at
    /// the given `baud` rate.
    ///
    /// The port is put into raw mode (8 data bits, no parity, 1 stop bit, no
    /// hardware or software flow control) — the framing the RCP link uses. On
    /// Linux the baud rate is the numeric value directly (e.g. `115_200`); an
    /// unusual rate the platform cannot set surfaces as an error.
    pub fn open(path: impl AsRef<Path>, baud: u32) -> io::Result<Self> {
        // Non-blocking so `async-io` can drive readiness; no controlling tty.
        let fd = open(
            path.as_ref(),
            OFlag::O_RDWR | OFlag::O_NOCTTY | OFlag::O_NONBLOCK,
            Mode::empty(),
        )
        .map_err(io::Error::from)?;

        // Map the numeric baud to the platform's `BaudRate`. Note this is NOT a
        // numeric cast: on many platforms (e.g. Linux) the termios speed constant
        // `B115200` is an *encoded* value (0x1002), not `115200`. `nix`'s
        // `TryFrom<speed_t>` matches those encoded constants, so we translate from
        // the human baud number explicitly.
        let baud = baud_rate(baud)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "unsupported baud rate"))?;

        // Exclusive mode: a tty allows concurrent opens by default, and a
        // second reader (a stale process, a terminal monitor) silently steals
        // bytes from the spinel stream, corrupting both sessions. With
        // `TIOCEXCL`, other non-root opens fail with `EBUSY` instead.
        //
        // SAFETY: `fd` is a valid, owned tty descriptor; `TIOCEXCL` takes no
        // argument.
        if unsafe { nix::libc::ioctl(std::os::fd::AsRawFd::as_raw_fd(&fd), nix::libc::TIOCEXCL) }
            != 0
        {
            return Err(io::Error::last_os_error());
        }

        // Raw mode + baud + 8N1 + no flow control.
        let mut termios = nix::sys::termios::tcgetattr(&fd).map_err(io::Error::from)?;
        cfmakeraw(&mut termios);
        cfsetspeed(&mut termios, baud).map_err(io::Error::from)?;
        // Enable the receiver and ignore modem control lines; drop hardware
        // (RTS/CTS) and software (XON/XOFF) flow control.
        termios.control_flags |= ControlFlags::CLOCAL | ControlFlags::CREAD;
        termios.control_flags &= !ControlFlags::CRTSCTS;
        termios.input_flags &= !(InputFlags::IXON | InputFlags::IXOFF | InputFlags::IXANY);
        tcsetattr(&fd, SetArg::TCSANOW, &termios).map_err(io::Error::from)?;

        // The drain pipe: the thread reads the tty and writes here; `read`
        // consumes the read end via `async-io`.
        let (pipe_rd, pipe_wr) = pipe().map_err(io::Error::from)?;

        // Cap the pipe: it only needs to bridge the driver's short read gaps
        // (a few radio-seconds of inbound), and a deep pipe just adds standing
        // latency under saturation (bufferbloat). The drain thread *drops* on
        // a full pipe rather than blocking (see `drain`), so the cap bounds
        // queueing latency without ever back-pressuring the device. Linux-only:
        // other Unixes keep their fixed (typically 16-64 KiB) pipe size.
        #[cfg(target_os = "linux")]
        let _ = nix::fcntl::fcntl(
            &pipe_wr,
            nix::fcntl::FcntlArg::F_SETPIPE_SZ(PIPE_CAPACITY as _),
        );

        // Non-blocking pipe writes, so a full pipe results in dropped bytes
        // (never in the drain thread parking and the tty going unread).
        nix::fcntl::fcntl(&pipe_wr, nix::fcntl::FcntlArg::F_SETFL(OFlag::O_NONBLOCK))
            .map_err(io::Error::from)?;

        // The thread polls its own dup of the tty fd. (`O_NONBLOCK` is a
        // property of the shared open file description, so the thread pairs
        // `poll` with non-blocking reads rather than relying on blocking I/O.)
        let tty_for_thread = fd.try_clone()?;

        std::thread::Builder::new()
            .name("ot-serial-drain".into())
            .spawn(move || drain(tty_for_thread, pipe_wr))?;

        Ok(Self {
            tty: Async::new(fd)?,
            rx: Async::new(pipe_rd)?,
        })
    }
}

/// The drain-thread loop: move bytes from the tty to the pipe until either
/// side goes away (tty error/EOF, or the pipe's read end — the `SerialPort` —
/// is dropped, which surfaces as `EPIPE`/`POLLERR` on the write end).
fn drain(tty: OwnedFd, pipe_wr: OwnedFd) {
    let mut buf = [0u8; 1024];

    loop {
        // Wait for tty data; also watch the pipe's write end so the thread
        // exits promptly when the `SerialPort` is dropped even if the tty has
        // gone quiet (a closed-read-end pipe reports `POLLERR`).
        let mut fds = [
            PollFd::new(tty.as_fd(), PollFlags::POLLIN),
            PollFd::new(pipe_wr.as_fd(), PollFlags::POLLERR),
        ];
        match poll(&mut fds, PollTimeout::NONE) {
            Ok(_) => (),
            Err(nix::errno::Errno::EINTR) => continue,
            Err(_) => break,
        }
        if fds[1]
            .revents()
            .is_some_and(|r| r.intersects(PollFlags::POLLERR))
        {
            break;
        }

        match nix::unistd::read(&tty, &mut buf) {
            Ok(0) | Err(nix::errno::Errno::EIO) => break, // tty EOF / unplugged
            Ok(n) => {
                // Never park on a full pipe — the whole point of this thread
                // is that the tty is *always* drained. Bytes that do not fit
                // are dropped, like a UART RX overrun: the clipped HDLC frame
                // fails its FCS and the loss recovers via the spinel
                // per-command acks and MAC/upper-layer retries.
                let mut rest = &buf[..n];
                while !rest.is_empty() {
                    match nix::unistd::write(&pipe_wr, rest) {
                        Ok(written) => rest = &rest[written..],
                        Err(nix::errno::Errno::EAGAIN) => break, // full — drop
                        Err(nix::errno::Errno::EINTR) => (),
                        Err(_) => return, // consumer gone
                    }
                }
            }
            Err(nix::errno::Errno::EAGAIN) | Err(nix::errno::Errno::EINTR) => (),
            Err(_) => break,
        }
    }
}

/// Translate a human baud number (e.g. `115_200`) into the platform's termios
/// [`BaudRate`]. Returns `None` for a rate the platform does not define.
fn baud_rate(baud: u32) -> Option<BaudRate> {
    Some(match baud {
        9_600 => BaudRate::B9600,
        19_200 => BaudRate::B19200,
        38_400 => BaudRate::B38400,
        57_600 => BaudRate::B57600,
        115_200 => BaudRate::B115200,
        230_400 => BaudRate::B230400,
        460_800 => BaudRate::B460800,
        921_600 => BaudRate::B921600,
        _ => return None,
    })
}

impl embedded_io_async::ErrorType for SerialPort {
    type Error = io::Error;
}

impl embedded_io_async::Read for SerialPort {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // Served from the drain pipe (see the module docs). `read_with`
        // re-arms readiness and retries whenever the raw read would block; it
        // resolves with the first non-`WouldBlock` result.
        self.rx
            .read_with(|fd| nix::unistd::read(fd, buf).map_err(io::Error::from))
            .await
    }
}

impl embedded_io_async::Write for SerialPort {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tty
            .write_with(|fd| nix::unistd::write(fd, buf).map_err(io::Error::from))
            .await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // Bytes are handed to the kernel by `write`; there is no userspace buffer
        // to flush. (Draining the kernel TX queue via `tcdrain` is not needed for
        // the request/response cadence of the spinel driver.)
        Ok(())
    }
}
