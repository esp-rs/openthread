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

use std::io::{BufRead, BufReader};
use std::process::{Child, Command, Stdio};
use std::sync::mpsc;
use std::time::{Duration, Instant};

/// An Operational Dataset (TLV hex) for the test network; channel 25,
/// so nodes only talk after OpenThread switches the radio config there.
const DATASET: &str = "000300001901020fd80208b566147d38e384200e080000639c5d67a3bd0510c490f58d4be0d5eaeb0f09b395d1ae17030d4e4553542d50414e2d304644380708fd7d4f8232cb00000410a7e08419ae47c177fb91bcfcec789aa50c0402a0f77835060004001fffe0";

/// A spawned `sim_node` process with its stdout lines drained on a thread
/// (so the child never blocks on a full pipe, and waits can time out).
struct Node {
    node_id: u16,
    child: Child,
    lines: mpsc::Receiver<String>,
}

impl Node {
    fn spawn(node_id: u16, port_base: u16) -> Self {
        let mut child = Command::new(env!("CARGO_BIN_EXE_sim_node"))
            .arg(node_id.to_string())
            .env("PORT_BASE", port_base.to_string())
            .env("SIM_NODE_DATASET", DATASET)
            .stdout(Stdio::piped())
            .spawn()
            .expect("spawn sim_node");

        let stdout = child.stdout.take().unwrap();
        let (sender, lines) = mpsc::channel();

        std::thread::spawn(move || {
            for line in BufReader::new(stdout).lines() {
                let Ok(line) = line else {
                    break;
                };
                if sender.send(line).is_err() {
                    break;
                }
            }
        });

        Self {
            node_id,
            child,
            lines,
        }
    }

    /// Wait until the node reports one of `roles`; any role outside both
    /// `roles` and `interim` fails immediately (wrong-partition detection).
    fn wait_role(&self, roles: &[&str], interim: &[&str], timeout: Duration) -> String {
        let deadline = Instant::now() + timeout;

        loop {
            let remaining = deadline
                .checked_duration_since(Instant::now())
                .unwrap_or_else(|| {
                    panic!("node {}: timed out waiting for role {roles:?}", self.node_id)
                });

            let line = self
                .lines
                .recv_timeout(remaining)
                .unwrap_or_else(|_| {
                    panic!("node {}: timed out waiting for role {roles:?}", self.node_id)
                });

            let Some(role) = line.strip_prefix("role: ") else {
                continue;
            };

            if roles.contains(&role) {
                break role.to_string();
            }

            assert!(
                interim.contains(&role),
                "node {}: unexpected role {role:?} while waiting for {roles:?}",
                self.node_id
            );
        }
    }
}

impl Drop for Node {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

#[test]
fn two_nodes_form_network() {
    // A port base away from the harness default 9000, varied per test process
    // so that concurrent invocations use disjoint simulation media.
    let port_base = 17000 + (std::process::id() % 4000) as u16;

    let first = Node::spawn(1, port_base);
    first.wait_role(
        &["Leader"],
        &["Disabled", "Detached"],
        Duration::from_secs(60),
    );

    let second = Node::spawn(2, port_base);
    let role = second.wait_role(
        &["Child", "Router"],
        &["Disabled", "Detached"],
        Duration::from_secs(120),
    );

    println!("network formed: node 1 = Leader, node 2 = {role}");
}
