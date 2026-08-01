//! Shared plumbing for the `openthread` integration-test binaries.
//!
//! The binaries in `src/bin` are simulation nodes: full `openthread` stacks
//! (Rust platform - embassy alarm, tasklet pumping, software MAC) whose
//! 802.15.4 "RF" is UDP multicast on the loopback interface, wire-compatible
//! with the upstream OpenThread C simulation platform. See [`sim_radio`].

pub mod sim_radio;
