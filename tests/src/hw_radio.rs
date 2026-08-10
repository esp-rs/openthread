//! The port map shared by the two hardware tiers, and the RCP tier's radio.
//!
//! Both tiers put real RF under the unmodified upstream scenarios; they differ
//! in where the `openthread` stack runs, and therefore in what a "node" is:
//!
//! - **RCP** ([`NodeKind::Rcp`]): the stack runs on the host, in the ordinary
//!   `cli_ftd` node process, and its radio is a [`SpinelRadio`] driving an
//!   802.15.4 co-processor over serial. Exercises the spinel driver and its
//!   transport.
//! - **MCU** ([`NodeKind::Mcu`]): the stack runs as *firmware* on the board,
//!   and the node process is only `serial_bridge`, piping the harness's
//!   stdin/stdout to the device's console. The only tier that exercises the
//!   crate's own radio drivers, `MacRadio`'s real ACK deadlines and
//!   `ProxyRadio`'s executor split.
//! - **C posix host** ([`NodeKind::CPosix`]): the *upstream* OpenThread posix
//!   host (`ot-cli`) drives the co-processor instead of this crate - a golden
//!   reference node on real RF. Pairing the DUT against one isolates any
//!   interop failure to the DUT, the same role `--peers c` plays in
//!   simulation.
//!
//! # The port map
//!
//! A node learns which board is *its* board from `OT_HW_PORTS`: a
//! comma-separated list indexed by node id (which the harness assigns from 1
//! upwards). Each entry is
//!
//! ```text
//! <device>[@<baud>][=<kind>]
//! ```
//!
//! so a mixed rig - the usual one, since pairing a device under test against a
//! known-good RCP node is what isolates a failure to the device - reads:
//!
//! ```text
//! OT_HW_PORTS=/dev/ttyACM0=mcu,/dev/ttyACM1=rcp
//! ```
//!
//! or, DUT against the upstream posix host on the second dongle:
//!
//! ```text
//! OT_HW_PORTS=/dev/ttyACM0=rcp,/dev/ttyACM1=cposix
//! ```
//!
//! The kind defaults to `rcp`. The baud defaults to `OT_HW_BAUD`, and that to
//! [`DEFAULT_BAUD`]; a device whose console or link is USB CDC ignores it
//! entirely, so it only matters behind a real UART bridge.
//!
//! A test needing more nodes than the map has entries cannot run; the `xtask`
//! side keeps that from happening by skipping such tests up front, and a node
//! asked for an entry that is not there fails loudly rather than silently
//! falling back to a simulated radio - a hardware run that quietly stopped
//! being one would be worse than no run at all.

use std::env;

#[cfg(feature = "hw")]
use openthread::spinel::{
    SerialPort, SpinelRadio, SpinelRadioResources, UartSpinelTransport, UartTransportResources,
};

#[cfg(feature = "hw")]
use static_cell::ConstStaticCell;

/// The environment variable carrying the node-id-indexed port map.
pub const PORTS_VAR: &str = "OT_HW_PORTS";

/// The environment variable overriding the default link speed.
pub const BAUD_VAR: &str = "OT_HW_BAUD";

/// The link speed assumed when nothing says otherwise, matching `RCP_BAUD` of
/// the host examples (`examples/std`) - what stock `ot-rcp` firmware and the
/// usual console come up at.
pub const DEFAULT_BAUD: u32 = 115_200;

/// What kind of node a board hosts - see the module docs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeKind {
    /// A co-processor: the stack runs here, its radio is over there.
    Rcp,
    /// A board running the whole stack as firmware; the node process only
    /// bridges the harness to its console.
    Mcu,
    /// A co-processor driven by the *upstream* OpenThread posix host
    /// (`ot-cli`) rather than this crate - a golden reference node.
    CPosix,
}

/// One node's board: its device, link speed, and what runs on it.
#[derive(Debug, Clone)]
pub struct Node {
    /// The serial device.
    pub device: String,
    /// The link speed.
    pub baud: u32,
    /// What is running on the far end.
    pub kind: NodeKind,
}

/// The board serving `node_id`, or `None` when no port map is set at all
/// (i.e. this is not a hardware run).
///
/// Panics if a map *is* set but has no entry for this node: that is a
/// misconfigured rig, and falling back to a simulated radio would quietly
/// invalidate the whole run.
pub fn node_for(node_id: u16) -> Option<Node> {
    let ports = env::var(PORTS_VAR).ok()?;

    let ports: Vec<&str> = ports
        .split(',')
        .map(str::trim)
        .filter(|port| !port.is_empty())
        .collect();

    if ports.is_empty() {
        return None;
    }

    // Node ids start at 1, and index the map in order.
    let entry = usize::from(node_id)
        .checked_sub(1)
        .and_then(|index| ports.get(index))
        .unwrap_or_else(|| {
            panic!(
                "{PORTS_VAR} lists {} board(s), but this is node {node_id}: \
                 a hardware run needs one board per node",
                ports.len(),
            )
        });

    Some(parse_entry(entry))
}

/// Split a `<device>[@<baud>][=<kind>]` entry into its parts.
fn parse_entry(entry: &str) -> Node {
    let (rest, kind) = match entry.rsplit_once('=') {
        Some((rest, kind)) => (
            rest,
            match kind {
                "rcp" => NodeKind::Rcp,
                "mcu" => NodeKind::Mcu,
                "cposix" => NodeKind::CPosix,
                other => {
                    panic!("{PORTS_VAR}: `{other}` in `{entry}` is not `rcp`, `mcu` or `cposix`")
                }
            },
        ),
        None => (entry, NodeKind::Rcp),
    };

    let (device, baud) = match rest.rsplit_once('@') {
        Some((device, baud)) => (
            device,
            baud.parse().unwrap_or_else(|_| {
                panic!("{PORTS_VAR}: `{baud}` in `{entry}` is not a baud rate")
            }),
        ),
        None => (rest, default_baud()),
    };

    Node {
        device: device.to_string(),
        baud,
        kind,
    }
}

/// The link speed for entries that do not carry one of their own.
fn default_baud() -> u32 {
    env::var(BAUD_VAR)
        .ok()
        .and_then(|baud| baud.parse().ok())
        .unwrap_or(DEFAULT_BAUD)
}

/// The RCP tier's radio, once bound to its serial port.
#[cfg(feature = "hw")]
pub type HwRadio = SpinelRadio<'static, UartSpinelTransport<'static, SerialPort>>;

/// Open `device` and build the node's radio on top of it.
///
/// The radio and transport buffers live in `const`-constructed statics
/// (`.bss`), so they never travel through the stack - the same shape the host
/// examples use. A node owns exactly one radio for the life of the process,
/// which is what makes taking them from statics sound here (and why
/// `hw_probe`, which walks several ports, allocates its own instead).
#[cfg(feature = "hw")]
pub fn radio(device: &str, baud: u32) -> HwRadio {
    static RADIO_RESOURCES: ConstStaticCell<SpinelRadioResources> =
        ConstStaticCell::new(SpinelRadioResources::new());
    static UART_RESOURCES: ConstStaticCell<UartTransportResources> =
        ConstStaticCell::new(UartTransportResources::new());

    let serial = SerialPort::open(device, baud)
        .unwrap_or_else(|err| panic!("open RCP serial port {device}: {err}"));

    SpinelRadio::new(
        UartSpinelTransport::new(serial, UART_RESOURCES.take()),
        RADIO_RESOURCES.take(),
    )
}
