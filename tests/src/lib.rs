//! Shared plumbing for the `openthread` integration-test binaries.
//!
//! The binaries in `src/bin` are simulation nodes: full `openthread` stacks
//! (Rust platform - embassy alarm, tasklet pumping, software MAC) on a
//! simulated 802.15.4 medium, wire-compatible with the upstream OpenThread C
//! simulation platform in both of its flavors:
//!
//! - real time: UDP multicast frames on the loopback interface
//!   ([`sim_radio`]);
//! - virtual time: frames and time as events of the upstream Python
//!   simulator's lockstep protocol ([`vt`], [`executor`]).
//!
//! With the `hw` feature they can also drop the simulated medium entirely and
//! drive a real 802.15.4 co-processor over a serial link ([`hw_radio`]) -
//! same binary, same harness, real RF.

pub mod executor;
pub mod hw_radio;
pub mod settings;
pub mod sim_radio;
pub mod vt;
