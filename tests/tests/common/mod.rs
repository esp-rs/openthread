//! Helpers for the smoke test: spawning and driving `cli_node` processes.

use std::io::{BufRead, BufReader, Write};
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::mpsc;
use std::time::{Duration, Instant};

/// An Operational Dataset (TLV hex) for the test networks; channel 25, so
/// nodes only talk after OpenThread switches the radio config there.
pub const DATASET: &str = "000300001901020fd80208b566147d38e384200e080000639c5d67a3bd0510c490f58d4be0d5eaeb0f09b395d1ae17030d4e4553542d50414e2d304644380708fd7d4f8232cb00000410a7e08419ae47c177fb91bcfcec789aa50c0402a0f77835060004001fffe0";

/// A port base for an isolated instance of the simulated radio medium: away
/// from the harness default 9000, inside the caller's `range`, varied per
/// test process so concurrent invocations use disjoint media.
pub fn port_base(range: u16) -> u16 {
    range + (std::process::id() % 4000) as u16
}

/// Drain a child's stdout into a channel on a thread, so the child never
/// blocks on a full pipe and reads can time out.
fn drain_stdout(child: &mut Child) -> mpsc::Receiver<String> {
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

    lines
}

/// A spawned `cli_node` process, driven the way the upstream harness drives a
/// DUT: command lines in, textual replies out.
pub struct CliNode {
    node_id: u16,
    child: Child,
    stdin: ChildStdin,
    lines: mpsc::Receiver<String>,
}

impl CliNode {
    pub fn spawn(node_id: u16, port_base: u16) -> Self {
        let mut child = Command::new(env!("CARGO_BIN_EXE_cli_node"))
            .arg(node_id.to_string())
            .env("PORT_BASE", port_base.to_string())
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()
            .expect("spawn cli_node");

        let stdin = child.stdin.take().unwrap();
        let lines = drain_stdout(&mut child);

        Self {
            node_id,
            child,
            stdin,
            lines,
        }
    }

    /// Run one CLI command: send it, collect its output lines until the
    /// terminating `Done`. Panics on a CLI `Error <n>: ...` reply or timeout.
    pub fn cmd(&mut self, cmd: &str, timeout: Duration) -> Vec<String> {
        writeln!(self.stdin, "{cmd}").unwrap();
        self.stdin.flush().unwrap();

        let deadline = Instant::now() + timeout;
        let mut output = Vec::new();
        let mut echoed = false;

        loop {
            let remaining = deadline
                .checked_duration_since(Instant::now())
                .unwrap_or_else(|| panic!("node {}: `{cmd}` timed out", self.node_id));

            let line = self
                .lines
                .recv_timeout(remaining)
                .unwrap_or_else(|_| panic!("node {}: `{cmd}` timed out", self.node_id));

            // Strip the `\r` (CLI lines end `\r\n`) and any `> ` prompt the
            // interpreter emits without a newline, which the line reader glues
            // to the front of the next line.
            let mut line = line.trim_end();
            while let Some(unprompted) = line.strip_prefix("> ") {
                line = unprompted;
            }

            if line.is_empty() {
                continue;
            }

            // The node echoes the command when driven over pipes (as the
            // upstream harness expects); the echo is not output.
            if !echoed && line == cmd {
                echoed = true;
                continue;
            }

            if line == "Done" {
                break output;
            }

            assert!(
                !line.starts_with("Error "),
                "node {}: `{cmd}` failed: {line}",
                self.node_id
            );

            output.push(line.to_string());
        }
    }

    /// Poll `state` until one of `states` is reported; any state outside
    /// both `states` and `interim` fails immediately (e.g. a joiner becoming
    /// `leader` means a second partition: the nodes cannot hear each other).
    pub fn wait_states(&mut self, states: &[&str], interim: &[&str], timeout: Duration) {
        let deadline = Instant::now() + timeout;

        loop {
            let output = self.cmd("state", Duration::from_secs(10));

            if output.iter().any(|line| states.contains(&line.as_str())) {
                break;
            }

            for line in &output {
                assert!(
                    interim.contains(&line.as_str()) || line == "disabled",
                    "node {}: unexpected state {line:?} while waiting for {states:?}",
                    self.node_id
                );
            }

            assert!(
                Instant::now() < deadline,
                "node {}: timed out waiting for state {states:?} (last: {output:?})",
                self.node_id
            );

            std::thread::sleep(Duration::from_millis(500));
        }
    }

    /// Poll `state` until it reports `state`, or panic after `timeout`.
    pub fn wait_state(&mut self, state: &str, timeout: Duration) {
        let deadline = Instant::now() + timeout;

        loop {
            let output = self.cmd("state", Duration::from_secs(10));

            if output.iter().any(|line| line == state) {
                break;
            }

            assert!(
                Instant::now() < deadline,
                "node {}: timed out waiting for state {state:?} (last: {output:?})",
                self.node_id
            );

            std::thread::sleep(Duration::from_millis(500));
        }
    }
}

impl Drop for CliNode {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}
