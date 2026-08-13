//! OpenThread's C command-line interpreter, surfaced on the Rust API.
//!
//! The CLI is the standard control surface of OpenThread test binaries too.

use core::cell::Cell;
use core::ffi::{c_char, c_void};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;

use crate::sys::{otCliInputLine, otError_OT_ERROR_INVALID_ARGS, otr_cli_init};
use crate::{OpenThread, OtError};

/// Maximum accepted CLI input line, NUL terminator included.
///
/// Sized for the longest command the upstream harness sends
/// (`dataset set active <hex>` with a maximal 254-byte dataset TLV, i.e. 508 hex characters).
const INPUT_MAX: usize = 640;

/// The CLI output sink registered via [`OpenThread::cli_init`].
static OUTPUT: Mutex<CriticalSectionRawMutex, Cell<Option<fn(&[u8])>>> =
    Mutex::new(Cell::new(None));

impl OpenThread<'_> {
    /// Initialize the C CLI on this OpenThread instance.
    ///
    /// Arguments:
    /// - `output` receives every chunk of CLI output as raw bytes - typically a
    ///   `\r\n`-terminated line or a piece of one.
    ///   NOTE: Do NOT call-back into OpenThread from this output sink.
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
