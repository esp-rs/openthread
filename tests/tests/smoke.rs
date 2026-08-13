//! The `cargo test` smoke layer:
//! Two `cli_node` processes must form a Thread network over the UDP-multicast simulated medium,
//! driven exactly the way the upstream OpenThread harness drives its nodes - 
//! CLI command lines in, textual replies out.
//!
//! This exercises the whole Rust platform stack end-to-end in seconds, with
//! no Python harness or C peer build.

use core::time::Duration;

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

    joiner.wait_states(
        &["child", "router"],
        &["detached"],
        Duration::from_secs(120),
    );

    // The leader's own view agrees.
    leader.wait_state("leader", Duration::from_secs(10));
}
