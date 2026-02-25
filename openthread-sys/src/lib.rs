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

pub use bindings::*;

#[allow(
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    dead_code,
    unnecessary_transmutes,
    clippy::all
)]
pub mod bindings {
    #[cfg(not(target_os = "espidf"))]
    include!(env!("OPENTHREAD_SYS_BINDINGS_FILE"));

    #[cfg(target_os = "espidf")]
    pub use esp_idf_sys::*;
}
