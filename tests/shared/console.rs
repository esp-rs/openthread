//! The node's CLI console: where the harness talks to the stack.
//!
//! The board decides which binding exists, so this is the part that varies:
//!
//! - **UART** (default): `UARTE0`, for a board with a UART-to-USB bridge - an
//!   nRF52840-DK's J-Link virtual COM port, or a plain USB-serial adapter on
//!   two pins.
//! - **USB CDC** (`console-usb`): the nRF's own USB peripheral, presented to
//!   the host as a serial port - for a board whose only serial port *is* USB,
//!   like a XIAO nRF52840 or an nRF52840 dongle, neither of which has a bridge
//!   to talk through.
//!
//! Everything here is binding-agnostic on purpose: a console is a byte sink
//! plus a byte source, and only the two tasks that move bytes to and from the
//! wire care which peripheral is underneath.
//!
//! # Why output goes through a pipe
//!
//! OpenThread's CLI hands its output to a **synchronous** callback, called from
//! wherever in the stack the command happened to be. USB CDC has no synchronous
//! write - a packet write is an `await` - so a callback cannot write to that
//! device at all. It fills [`OUT`] instead, and a task drains that to the wire.
//!
//! The UART binding could have written synchronously, but goes through the pipe
//! anyway: it is the shape USB needs, and it keeps the stack from blocking on a
//! byte-by-byte UART while it is servicing a radio.
//!
//! Overflow drops output rather than blocking. A blocked write here would stall
//! the stack (including the radio it is driving), and the harness reports a
//! truncated line far more usefully than it reports a node that stopped
//! breathing.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;

/// CLI output on its way from the stack's synchronous callback to whichever
/// console task is draining it.
///
/// Sized for the largest burst a single command produces in ONE synchronous
/// sweep - and that is much bigger than intuition suggests: `srp server
/// service` on the upstream `test_srp_many_services_mtu_check` scenario emits
/// several KB (many services, 63-char instance names, six subtypes each) in a
/// single `cli_input_line` call, during which the drain task cannot run at
/// all. A burst that does not fit is silently truncated (see [`out`]), which
/// the harness sees as garbled or missing lines - so this buys headroom with
/// plain RAM, and [`drained`] keeps the *steady state* empty between
/// commands.
static OUT: Pipe<CriticalSectionRawMutex, 16384> = Pipe::new();

/// Longest CLI line the harness can send us. The upstream node driver sends
/// dataset TLVs as hex, which is what sets this.
pub const LINE_MAX: usize = 512;

/// The CLI's output sink, as handed to `OpenThread::cli_init`.
///
/// Synchronous by necessity (see the module docs), and lossy under pressure by
/// choice.
pub fn out(bytes: &[u8]) {
    // `try_write` takes what fits and reports the rest; a partial write means
    // the console is behind, and there is nothing useful to do about it here.
    let _ = OUT.try_write(bytes);
}

/// Take the next chunk of pending output, waiting if there is none.
pub async fn read_out(buf: &mut [u8]) -> usize {
    OUT.read(buf).await
}

/// How many bytes sit in the output pipe right now (diagnostic).
pub fn pending() -> usize {
    OUT.len()
}

/// Wait until every byte queued so far has been taken by the console task.
///
/// The command loop awaits this after each CLI command, BEFORE reading the
/// next one: the harness is strictly command-response paced, so parking here
/// hands the executor to the drain task until the response is fully on the
/// wire - cooperative backpressure that makes command/response traffic
/// lossless without ever blocking the (synchronous) output callback.
pub async fn drained() {
    while !OUT.is_empty() {
        // The drain task is the one emptying the pipe; yielding is what lets
        // it run. `yield_now` alone can spin ahead of a slow wire, so pace it
        // with the smallest real delay.
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
    }
}

/// Assembles CLI lines from console input, echoing as it goes.
///
/// The harness expects its command lines echoed back. On the host DUT over a
/// pty the kernel provides that; here nothing does, so the node echoes what it
/// reads - which is also what a person on a serial console expects.
pub struct LineReader {
    line: heapless::Vec<u8, LINE_MAX>,
}

impl LineReader {
    pub const fn new() -> Self {
        Self {
            line: heapless::Vec::new(),
        }
    }

    /// Feed one received byte; returns whether it completed a line.
    pub fn push(&mut self, byte: u8) -> bool {
        match byte {
            b'\r' | b'\n' => {
                out(b"\r\n");
                true
            }
            byte => {
                // An over-long line loses its tail rather than reaching the
                // interpreter truncated in the middle of an argument.
                let _ = self.line.push(byte);
                out(&[byte]);

                false
            }
        }
    }

    /// The line assembled so far. Non-UTF8 input is the harness's problem, not
    /// ours - it reads as empty rather than reaching the interpreter.
    pub fn line(&self) -> &str {
        core::str::from_utf8(&self.line).unwrap_or("")
    }

    pub fn clear(&mut self) {
        self.line.clear();
    }
}
