//! The traits a platform implements for the [`openthread`](https://crates.io/crates/openthread)
//! crate:
//! - [`Radio`]: the IEEE 802.15.4 radio
//! - [`Settings`]: the storage of OpenThread's settings
//!
//! Radio drivers and settings stores depend on this crate only, rather than on `openthread`,
//! which re-exports everything here.

#![no_std]
#![allow(async_fn_in_trait)]

pub use radio::*;
pub use settings::*;

pub mod radio;
pub mod settings;
