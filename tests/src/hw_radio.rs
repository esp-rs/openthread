//! The hardware tier's radio: a real 802.15.4 RCP on a serial port, in place
//! of the simulated medium the other node flavors use.
//!
//! The node process itself is unchanged - the upstream harness still spawns
//! `$OT_CLI_PATH <node id>` and drives it over stdin/stdout - so only the
//! bottom of the stack differs: instead of [`crate::sim_radio`] /
//! [`crate::vt`], the radio is a [`SpinelRadio`] speaking spinel over HDLC to
//! a co-processor that owns the actual RF.
//!
//! # The port map
//!
//! A node learns *its* port from `OT_HW_PORTS`: a comma-separated list of
//! device paths, indexed by node id (which the harness assigns from 1
//! upwards). So with
//!
//! ```text
//! OT_HW_PORTS=/dev/ttyACM0,/dev/ttyACM1
//! ```
//!
//! node 1 drives the RCP on `ttyACM0` and node 2 the one on `ttyACM1`.
//!
//! Link speed is per device, because a rig is easily heterogeneous - an
//! ESP32xx RCP comes up at 460800 where the stock firmware elsewhere uses
//! 115200. An entry may therefore carry its own rate:
//!
//! ```text
//! OT_HW_PORTS=/dev/ttyACM0,/dev/ttyUSB0@460800
//! ```
//!
//! Entries without one fall back to `OT_HW_BAUD`, and that to
//! [`DEFAULT_BAUD`]. A device exposing a USB CDC serial port ignores the rate
//! entirely - it only matters behind a real UART bridge.
//!
//! A test needing more nodes than there are ports cannot run; the `xtask`
//! side is what keeps that from happening (it skips such tests up front),
//! and a node asked for a port that is not in the map fails loudly rather
//! than silently falling back to a simulated radio - a hardware run that
//! quietly stopped being one would be worse than no run at all.
//!
//! # Why this tier exists
//!
//! The simulation suites cover the stack above the radio exhaustively, but
//! they drive a radio that is itself Rust simulation code. This tier puts a
//! real co-processor, a real serial link and real RF underneath the same
//! unmodified upstream scenarios - which is the only way [`SpinelRadio`] and
//! its transport get exercised at all.

use std::env;

use openthread::spinel::{
    SerialPort, SpinelRadio, SpinelRadioResources, UartSpinelTransport, UartTransportResources,
};

use static_cell::ConstStaticCell;

/// The environment variable carrying the node-id-indexed list of RCP serial
/// devices - see the module docs.
pub const PORTS_VAR: &str = "OT_HW_PORTS";

/// The environment variable overriding the serial link speed.
pub const BAUD_VAR: &str = "OT_HW_BAUD";

/// The default RCP link speed, matching `RCP_BAUD` of the host examples
/// (`examples/std`), i.e. what stock `ot-rcp` firmware comes up at.
pub const DEFAULT_BAUD: u32 = 115_200;

/// The node's radio, once bound to its serial port.
pub type HwRadio = SpinelRadio<'static, UartSpinelTransport<'static, SerialPort>>;

/// The serial device this node should drive and the speed to drive it at, or
/// `None` when no port map is set at all (i.e. this is not a hardware run).
pub fn port_for(node_id: u16) -> Option<(String, u32)> {
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
    let port = ports
        .get(usize::from(node_id).checked_sub(1)?)
        .unwrap_or_else(|| {
            panic!(
                "{PORTS_VAR} lists {} port(s), but this is node {node_id}: \
                 a hardware run needs one RCP per node",
                ports.len(),
            )
        });

    Some(split_baud(port))
}

/// Split a `<device>[@<baud>]` entry into its parts, filling in the fallback
/// speed when the entry carries none.
fn split_baud(port: &str) -> (String, u32) {
    match port.rsplit_once('@') {
        Some((device, baud)) => (
            device.to_string(),
            baud.parse()
                .unwrap_or_else(|_| panic!("{PORTS_VAR}: `{baud}` in `{port}` is not a baud rate")),
        ),
        None => (port.to_string(), default_baud()),
    }
}

/// The link speed for entries that do not carry one of their own.
fn default_baud() -> u32 {
    env::var(BAUD_VAR)
        .ok()
        .and_then(|baud| baud.parse().ok())
        .unwrap_or(DEFAULT_BAUD)
}

/// Open `port` and build the node's radio on top of it.
///
/// The radio and transport buffers live in `const`-constructed statics
/// (`.bss`), so they never travel through the stack - the same shape the
/// host examples use.
pub fn radio(port: &str, baud: u32) -> HwRadio {
    static RADIO_RESOURCES: ConstStaticCell<SpinelRadioResources> =
        ConstStaticCell::new(SpinelRadioResources::new());
    static UART_RESOURCES: ConstStaticCell<UartTransportResources> =
        ConstStaticCell::new(UartTransportResources::new());

    let serial = SerialPort::open(port, baud)
        .unwrap_or_else(|err| panic!("open RCP serial port {port}: {err}"));

    SpinelRadio::new(
        UartSpinelTransport::new(serial, UART_RESOURCES.take()),
        RADIO_RESOURCES.take(),
    )
}
