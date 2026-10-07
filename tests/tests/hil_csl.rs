//! CSL transmission between a parent and a Synchronized Sleepy End Device
//! (SSED), over real radios.
//!
//! The hardware counterpart of upstream's `v1_2_test_csl_transmission`, which
//! checks its scenario on the frames a simulated medium lets it sniff. Real air
//! has no sniffer here, so this checks what the frames are for instead: while
//! CSL is on, the SSED does not poll within this test's timescales (only at the
//! CSL timeout), so every ping the leader gets through to it travelled in one
//! of its CSL receive windows. That covers both ends at once - the parent
//! timing its transmissions (the CSL transmitter) and the child sampling the
//! channel when it said it would (the CSL receiver).
//!
//! Node 1 is the leader (CSL transmitter), node 2 the SSED (CSL receiver),
//! each a board from `OT_HW_PORTS` - see `openthread_tests::hw` (an `mcu`
//! board's firmware, or an RCP this crate drives). Real time, minutes long,
//! and only with hardware attached, so it runs only when asked for:
//!
//! ```sh
//! OT_HW_PORTS=/dev/ttyACM0@460800=rcp,/dev/ttyACM1=mcu \
//!     cargo test --features hw --test hil_csl -- --nocapture
//! ```
//!
//! `HIL_CSL_CHANNEL` picks the CSL channel of the off-network-channel phase
//! (default 26). Real air is shared: a channel under a busy Wi-Fi channel (12
//! sits on Wi-Fi channel 1) loses frames for reasons that have nothing to do
//! with CSL.

#![cfg(feature = "hw")]

use core::time::Duration;
use std::thread::sleep;

mod common;

use common::CliNode;

const LEADER: u16 = 1;
const SSED: u16 = 2;

/// The CSL period (500 ms) and timeout (30 s): upstream's defaults.
const CSL_PERIOD_US: u32 = 500_000;
const CSL_TIMEOUT_S: u32 = 30;

/// How many pings each phase sends: enough for a timing that is only right
/// some of the time to show.
const PINGS: usize = 10;

/// A ping's reply budget: several CSL periods.
const PING_TIMEOUT: Duration = Duration::from_secs(5);

/// The per-command budget (a bridged board answers within milliseconds).
const CMD: Duration = Duration::from_secs(10);

/// The network channel: away from the CSL channel of phase 2.
const NETWORK_CHANNEL: u8 = 11;

#[test]
fn csl_transmission() {
    let ports = std::env::var("OT_HW_PORTS").unwrap_or_default();
    if ports.split(',').filter(|port| !port.is_empty()).count() < 2 {
        eprintln!("hil_csl: needs two boards in OT_HW_PORTS, skipping");
        return;
    }

    let csl_channel: u8 = std::env::var("HIL_CSL_CHANNEL")
        .ok()
        .map(|channel| channel.parse().expect("HIL_CSL_CHANNEL: not a channel"))
        .unwrap_or(26);

    // The port base only matters on the simulated medium; hardware nodes
    // ignore it.
    let mut leader = CliNode::spawn(LEADER, 0);
    let mut ssed = CliNode::spawn(SSED, 0);

    // A network of its own, with fresh credentials: on real air a fixed
    // dataset may belong to a real network in range (the smoke test's names
    // a Nest network), which the leader would then join instead of forming
    // its own.
    leader.cmd("dataset init new", CMD);
    leader.cmd(&format!("dataset channel {NETWORK_CHANNEL}"), CMD);
    leader.cmd("dataset commit active", CMD);
    leader.cmd("ifconfig up", CMD);
    leader.cmd("thread start", CMD);
    leader.wait_states(&["leader"], &["detached"], Duration::from_secs(60));

    let dataset = leader.cmd("dataset active -x", CMD).concat();

    // A sleepy child (rx-off-when-idle, minimal device), on CSL.
    ssed.cmd("mode -", CMD);
    ssed.cmd(&format!("csl period {CSL_PERIOD_US}"), CMD);
    ssed.cmd(&format!("csl timeout {CSL_TIMEOUT_S}"), CMD);
    ssed.cmd(&format!("dataset set active {dataset}"), CMD);
    ssed.cmd("ifconfig up", CMD);
    ssed.cmd("thread start", CMD);
    ssed.wait_states(&["child"], &["detached"], Duration::from_secs(60));

    eprintln!("SSED CSL: {:?}", ssed.cmd("csl", CMD));

    let rloc = ssed.rloc();

    let pings = |leader: &mut CliNode, count: usize, size: usize| {
        (0..count)
            .filter(|_| leader.ping(&rloc, size, PING_TIMEOUT))
            .count()
    };

    // 1. Reachable through its CSL windows: single frames...
    assert_eq!(
        pings(&mut leader, PINGS, 8),
        PINGS,
        "CSL on the network channel"
    );
    // ... and fragmented ones, one fragment per window.
    assert_eq!(pings(&mut leader, 3, 300), 3, "fragmented, by CSL");

    // 2. On a CSL channel other than the network's: the parent transmits
    //    there, and the child samples there.
    ssed.cmd(&format!("csl channel {csl_channel}"), CMD);
    sleep(Duration::from_secs(1));
    assert_eq!(
        pings(&mut leader, PINGS, 8),
        PINGS,
        "CSL on channel {csl_channel}"
    );

    ssed.cmd("csl channel 0", CMD);
    sleep(Duration::from_secs(1));
    assert_eq!(
        pings(&mut leader, PINGS, 8),
        PINGS,
        "CSL back on the network channel"
    );

    // 3. Without CSL - and without polling - it is unreachable: what got
    //    through above did so by CSL. Turning CSL off is itself an exchange
    //    with the parent (a Child Update, answered while the child polls
    //    fast), so it is given time to settle first: after that the SED polls
    //    only at its default poll period, minutes away.
    ssed.cmd("csl period 0", CMD);
    sleep(Duration::from_secs(15));
    let reply = leader.ping_reply(&rloc, 8, PING_TIMEOUT);
    assert!(reply.is_none(), "reachable with CSL off: {reply:?}");

    // 4. Back on CSL, it is reachable again.
    ssed.cmd(&format!("csl period {CSL_PERIOD_US}"), CMD);
    sleep(Duration::from_secs(2));
    assert_eq!(pings(&mut leader, PINGS, 8), PINGS, "CSL back on");

    // 5. Still synchronized after idling well beyond the CSL timeout: the
    //    child's keep-alives keep the parent's schedule in step.
    sleep(Duration::from_secs(CSL_TIMEOUT_S as u64 * 2));
    assert_eq!(pings(&mut leader, PINGS, 8), PINGS, "CSL after idling");
}
