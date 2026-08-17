//! Low-level bindings to the OpenThread library.
//!
//! # External dependencies
//!
//! Openthread depends on functions from the C standard library. These are not
//! provided directly by this crate, but are expected to be linked by the user
//! in some way.
//!
//! The following functions are required:
//!
//! - `exit`
//! - `iscntrl`
//! - `isprint`
//! - `memchr`
//! - `memcmp`
//! - `memcpy`
//! - `memmove`
//! - `memset`
//! - `strchr`
//! - `strcmp`
//! - `strcpy`
//! - `strlen`
//! - `strncpy`
//! - `strstr`
//!
//! The following functions are required, but this crate already provides an
//! implementation for them:
//!
//! - `snprintf`
//! - `vsnprintf`
//!
//! Note that this list is likely to change over time.

#![no_std]
#![allow(unknown_lints)]

pub use self::bindings::*;

// Make sure mbedtls-rs-sys is linked (when using the external MbedTLS).
#[cfg(feature = "mbedtls-rs-sys")]
use mbedtls_rs_sys as _;

// The CLI output bridge (see `gen/support/src/cli_shim.c`): initializes the C
// CLI with a callback that formats each output chunk and forwards the bytes to
// the `otr_cli_output` Rust function (implemented by the `openthread` crate).
// Declared by hand rather than generated: the symbol lives in the shim, not in
// an OpenThread public header.
#[cfg(feature = "cli")]
extern "C" {
    pub fn otr_cli_init(instance: *mut otInstance, context: *mut core::ffi::c_void);
}

#[allow(
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    dead_code,
    unnecessary_transmutes,
    clippy::all
)]
mod bindings {
    #[cfg(not(target_os = "espidf"))]
    include!(env!("OPENTHREAD_SYS_BINDINGS_FILE"));

    #[cfg(target_os = "espidf")]
    pub use esp_idf_sys::*;
}
