//! The `cargo test` smoke layer: two `cli_node` processes must form a Thread
//! network over the UDP-multicast simulated medium, driven exactly the way
//! the upstream OpenThread harness drives its nodes - CLI command lines in,
//! textual replies out.
//!
//! This exercises the whole Rust platform stack end-to-end in seconds, with
//! no Python harness or C peer build: process startup, the embassy alarm,
//! tasklet pumping, the software MAC (ACK generation and awaiting), the sim
//! radio's wire protocol, and the CLI front-end plumbing the certification
//! suites depend on. Attaching requires a bidirectional MLE handshake with
//! acknowledged unicasts, so nothing short of a working radio path passes.
//! If the joiner ends up a leader of its own partition instead, the nodes
//! could not hear each other - the test fails fast rather than by timeout.
//!
//! Anything beyond this belongs to the upstream suites (`cargo xtask itest`),
//! which cover the same binary far more thoroughly - this file only has to
//! answer "is the stack alive at all" quickly enough to run on every build.

use std::time::Duration;

mod common;

use common::{port_base, CliNode, DATASET};

/// Generous per-command budget (debug builds; commands are near-instant).
const CMD: Duration = Duration::from_secs(10);

#[test]
fn two_cli_nodes_form_network() {
    let port_base = port_base(21000);

    let mut leader = CliNode::spawn(1, port_base);

    // Fresh node: Thread disabled, and the CLI is responsive.
    let state = leader.cmd("state", CMD);
    assert_eq!(state, ["disabled"], "unexpected initial state");

    // The exact command sequence node.py uses to bring a node up.
    leader.cmd(&format!("dataset set active {DATASET}"), CMD);
    leader.cmd("ifconfig up", CMD);
    leader.cmd("thread start", CMD);

    leader.wait_state("leader", Duration::from_secs(60));

    // A second node joins over the same medium.
    let mut joiner = CliNode::spawn(2, port_base);
    joiner.cmd(&format!("dataset set active {DATASET}"), CMD);
    joiner.cmd("ifconfig up", CMD);
    joiner.cmd("thread start", CMD);

    joiner.wait_states(&["child", "router"], &["detached"], Duration::from_secs(120));

    // The leader's own view agrees.
    leader.wait_state("leader", Duration::from_secs(10));
}
