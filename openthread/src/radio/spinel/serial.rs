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
use std::os::fd::OwnedFd;
use std::path::Path;

use async_io::Async;

use nix::fcntl::{open, OFlag};
use nix::sys::stat::Mode;
use nix::sys::termios::{
    cfmakeraw, cfsetspeed, tcsetattr, BaudRate, ControlFlags, InputFlags, SetArg,
};

/// An async serial port over a `/dev/tty*` device (Unix host).
///
/// Configured for raw, 8N1, no-flow-control operation — the mode an `ot-rcp`
/// expects. Wrap it in a
/// [`UartSpinelTransport`](super::UartSpinelTransport) for use with
/// [`SpinelRadio`](super::SpinelRadio). See the [module docs](self).
pub struct SerialPort {
    inner: Async<OwnedFd>,
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

        Ok(Self {
            inner: Async::new(fd)?,
        })
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
        // `read_with` re-arms readiness and retries whenever the raw read would
        // block; it resolves with the first non-`WouldBlock` result.
        self.inner
            .read_with(|fd| nix::unistd::read(fd, buf).map_err(io::Error::from))
            .await
    }
}

impl embedded_io_async::Write for SerialPort {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.inner
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
