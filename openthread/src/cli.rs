//! OpenThread's C command-line interpreter, surfaced on the Rust API.
//!
//! The CLI is the standard control surface of OpenThread test/DUT binaries:
//! the upstream test harness (`tests/scripts/thread-cert`), its `expect`
//! suite, and the Thread certification harness (THCI) all drive devices by
//! sending CLI command lines and parsing the textual output. This module lets
//! a binary built on this crate be driven the same way: feed input lines via
//! [`OpenThread::cli_input_line`], receive output through the sink registered
//! with [`OpenThread::cli_init`].
//!
//! The C CLI operates on the same OpenThread instance as the rest of this
//! crate's API, so CLI-triggered state changes are observable through the
//! regular API and vice versa.

use core::cell::Cell;
use core::ffi::{c_char, c_void};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;

use crate::sys::{otCliInputLine, otError_OT_ERROR_INVALID_ARGS, otr_cli_init};
use crate::{OpenThread, OtError};

/// Maximum accepted CLI input line, NUL terminator included. Sized for the
/// longest command the upstream harness sends (`dataset set active <hex>`
/// with a maximal 254-byte dataset TLV, i.e. 508 hex characters).
const INPUT_MAX: usize = 640;

/// The CLI output sink registered via [`OpenThread::cli_init`].
///
/// Process-global, like the C CLI itself: `otCliInit` binds the interpreter to
/// the (single) OpenThread instance with no de-registration.
static OUTPUT: Mutex<CriticalSectionRawMutex, Cell<Option<fn(&[u8])>>> =
    Mutex::new(Cell::new(None));

impl OpenThread<'_> {
    /// Initialize the C CLI on this OpenThread instance.
    ///
    /// `output` receives every chunk of CLI output as raw bytes - typically a
    /// `\r\n`-terminated line or a piece of one (the CLI formats output in
    /// multiple small writes; chunk boundaries are not line boundaries).
    /// It is invoked synchronously from within CLI processing: from
    /// [`cli_input_line`](OpenThread::cli_input_line) for direct command
    /// output, or from the OpenThread run loop for asynchronously produced
    /// output (ping replies, scan tables, the trailing `Done`). It must not
    /// call back into OpenThread.
    ///
    /// Call once; a subsequent call replaces the output sink of the (single)
    /// C interpreter.
    pub fn cli_init(&self, output: fn(&[u8])) {
        let mut ot = self.activate();
        let state = ot.state();

        OUTPUT.lock(|cell| cell.set(Some(output)));

        unsafe { otr_cli_init(state.ot.instance, core::ptr::null_mut()) }
    }

    /// Feed one input line (without line terminator) to the C CLI,
    /// processing it synchronously.
    ///
    /// Errors with `INVALID_ARGS` if the line exceeds [`INPUT_MAX`] - 1 bytes;
    /// command execution errors are not reported here but on the CLI's own
    /// output (`Error <n>: ...`), as the harnesses expect.
    pub fn cli_input_line(&self, line: &str) -> Result<(), OtError> {
        // NUL-terminated scratch copy: the CLI tokenizes the line in place.
        let mut buf = [0_u8; INPUT_MAX];

        if line.len() >= buf.len() {
            return Err(OtError::new(otError_OT_ERROR_INVALID_ARGS));
        }

        buf[..line.len()].copy_from_slice(line.as_bytes());

        let mut ot = self.activate();
        let _ = ot.state();

        unsafe { otCliInputLine(buf.as_mut_ptr() as *mut c_char) };

        Ok(())
    }
}

/// Called by the C shim (`openthread-sys`, `cli_shim.c`) with each formatted
/// chunk of CLI output.
#[no_mangle]
extern "C" fn otr_cli_output(_context: *mut c_void, output: *const c_char, len: usize) {
    let sink = OUTPUT.lock(|cell| cell.get());

    if let Some(sink) = sink {
        let output = unsafe { core::slice::from_raw_parts(output.cast::<u8>(), len) };
        sink(output);
    }
}
