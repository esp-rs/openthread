//! Smoke test for the simulated radio medium: two `sim_node` processes must
//! form a Thread network over UDP-multicast "RF" on localhost - the first
//! becoming Leader, the second attaching to it.
//!
//! This exercises the whole Rust platform stack end-to-end (embassy alarm,
//! tasklet pumping, the software MAC incl. ACK generation/awaiting, and the
//! sim radio's wire protocol): attaching requires a bidirectional
//! MLE handshake with acknowledged unicasts, so nothing short of a working
//! radio path passes it. If the second node ends up Leader instead, the nodes
//! could not hear each other (two singleton partitions) - the radio path is
//! broken, and the test fails fast rather than by timeout.

use std::time::Duration;

mod common;

use common::{port_base, SimNode};

#[test]
fn two_nodes_form_network() {
    let port_base = port_base(17000);

    let first = SimNode::spawn(1, port_base);
    first.wait_role(
        &["Leader"],
        &["Disabled", "Detached"],
        Duration::from_secs(60),
    );

    let second = SimNode::spawn(2, port_base);
    let role = second.wait_role(
        &["Child", "Router"],
        &["Disabled", "Detached"],
        Duration::from_secs(120),
    );

    println!("network formed: node 1 = Leader, node 2 = {role}");
}
