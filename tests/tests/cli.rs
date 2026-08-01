//! Drives a `cli_ftd` node exactly the way the upstream OpenThread harness
//! drives a DUT - CLI command lines in, textual replies out - and proves the
//! two node flavors interoperate: a CLI-driven node forms the network, an
//! API-driven `sim_node` attaches to it over the same simulated medium.

use std::time::Duration;

mod common;

use common::{port_base, CliNode, SimNode, DATASET};

/// Generous per-command budget (debug builds; commands are near-instant).
const CMD: Duration = Duration::from_secs(10);

#[test]
fn cli_driven_node_forms_network() {
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

    // An API-driven node joins the CLI-driven node's network: both flavors
    // speak the same simulated medium, mirroring mixed Rust/C topologies.
    let joiner = SimNode::spawn(2, port_base);
    let role = joiner.wait_role(
        &["Child", "Router"],
        &["Disabled", "Detached"],
        Duration::from_secs(120),
    );

    // The leader's own view agrees.
    leader.wait_state("leader", Duration::from_secs(10));

    println!("network formed: CLI node = leader, API node = {role}");
}
