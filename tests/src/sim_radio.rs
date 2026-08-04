//! A [`Radio`] implementation whose "RF medium" is UDP multicast on the
//! loopback interface, wire-compatible with the upstream OpenThread C
//! simulation platform (`examples/platforms/simulation` in the OpenThread
//! repo, real-time mode).
//!
//! Wire compatibility is the whole point: a node built on this radio can join
//! a simulated 802.15.4 network together with upstream `ot-cli-ftd`/`ot-rcp`
//! simulation binaries, and is visible to the passive sniffer of the upstream
//! Python test harness (`tests/scripts/thread-cert`) - which is what lets the
//! upstream e2e suites drive nodes built on this crate.
//!
//! The protocol (as implemented by the C platform's `simul_utils.c`/`radio.c`):
//!
//! - All nodes join multicast group `224.0.0.116` on `127.0.0.1` and bind
//!   their RX socket to `(224.0.0.116, port base)` with
//!   `SO_REUSEADDR`/`SO_REUSEPORT`, so any number of nodes coexist on one host.
//! - A node's TX socket is bound to `(127.0.0.1, port base + node id)` with
//!   multicast loopback enabled: the *source port* of a datagram identifies
//!   the sending node. That is also how a node recognizes - and discards -
//!   its own looped-back frames.
//! - One datagram is one radio frame: a channel byte followed by the full
//!   PSDU. The last two PSDU bytes carry a real FCS (CRC-16/KERMIT), computed
//!   by the sender, so that harness sniffers see valid frames.
//! - A frame is accepted only if its channel byte matches the channel the
//!   radio currently listens on; RSSI is reported as a fixed value, exactly
//!   like the C simulation.
//!
//! Like the nRF driver, this is a bare PHY: no ACKs, no address filtering, no
//! frame security. Wrap it in [`MacRadio`](openthread::MacRadio) so the
//! crate's software MAC provides those - which conveniently makes the sim
//! exercise the very code paths real PHY-only hardware uses.

use core::fmt;

use std::io;
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};

use async_io::Async;

use openthread::{Config, PsduMeta, Radio, RadioCaps, RadioError, RadioErrorKind};

use socket2::{Domain, Protocol, Socket, Type};

/// The multicast group of the simulated radio medium.
const GROUP: Ipv4Addr = Ipv4Addr::new(224, 0, 0, 116);

/// The interface the simulation runs on by default (the C platform's `-L`
/// option selects another).
const LOCAL: Ipv4Addr = Ipv4Addr::LOCALHOST;

/// The default port base (`PORT_BASE` env var overrides, as in the C platform).
const DEFAULT_PORT_BASE: u16 = 9000;

/// The maximum simulated network size the upstream harness assumes
/// (`OPENTHREAD_SIMULATION_MAX_NETWORK_SIZE`); only used in the `PORT_OFFSET`
/// port-base displacement formula, which must match the C platform's so that
/// parallel harness runs pick non-overlapping port ranges.
const MAX_NETWORK_SIZE: u16 = 33;

/// The RSSI the C simulation platform stamps on every received frame.
pub(crate) const SIM_RSSI: i8 = -20;

/// Maximum PSDU size of an 802.15.4 frame (FCS included).
pub(crate) const PSDU_MAX: usize = openthread::sys::OT_RADIO_FRAME_MAX_SIZE as usize;

/// The error type of [`SimRadio`].
#[derive(Debug)]
pub enum SimRadioError {
    /// A TX PSDU too short to carry an FCS or longer than a frame.
    TxInvalid,
    /// Socket I/O failed.
    Io(io::Error),
}

impl RadioError for SimRadioError {
    fn kind(&self) -> RadioErrorKind {
        match self {
            Self::TxInvalid => RadioErrorKind::TxInvalid,
            Self::Io(_) => RadioErrorKind::Other,
        }
    }
}

impl fmt::Display for SimRadioError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TxInvalid => write!(f, "invalid TX PSDU"),
            Self::Io(err) => write!(f, "socket I/O error: {err}"),
        }
    }
}

impl From<io::Error> for SimRadioError {
    fn from(err: io::Error) -> Self {
        Self::Io(err)
    }
}

/// A [`Radio`] on the simulated UDP-multicast radio medium.
pub struct SimRadio {
    node_id: u16,
    port_base: u16,
    rx: Async<UdpSocket>,
    tx: Async<UdpSocket>,
    config: Config,
    /// Set by [`Radio::sleep`]; the next `receive` flushes everything that
    /// accumulated in the socket while asleep (contract point 5: a sleeping
    /// radio misses frames - the C simulation radio drops them at arrival,
    /// flushing on wake is the queue-based equivalent).
    slept: bool,
}

impl SimRadio {
    /// Create a radio for simulation node `node_id`, with the port base taken
    /// from the `PORT_BASE`/`PORT_OFFSET` env vars exactly like the C
    /// simulation platform does - so binaries using this constructor are
    /// drop-in nodes for the upstream test harness, which pre-arranges those
    /// vars for parallel runs.
    pub fn new(node_id: u16) -> Result<Self, SimRadioError> {
        Self::new_with_port_base(node_id, port_base_from_env())
    }

    /// Create a radio for simulation node `node_id` on an explicit port base
    /// (i.e. an explicit, isolated instance of the radio medium).
    pub fn new_with_port_base(node_id: u16, port_base: u16) -> Result<Self, SimRadioError> {
        Self::new_with(node_id, port_base, LOCAL)
    }

    /// Create a radio for simulation node `node_id` on an explicit port base
    /// and local interface address (the C platform's `-L` option; the
    /// harness uses distinct loopback addresses to model multiple hosts).
    pub fn new_with(node_id: u16, port_base: u16, local: Ipv4Addr) -> Result<Self, SimRadioError> {
        Ok(Self {
            node_id,
            port_base,
            rx: Async::new(Self::rx_socket(port_base, local)?)?,
            tx: Async::new(Self::tx_socket(port_base + node_id, local)?)?,
            config: Config::new(),
            slept: false,
        })
    }

    fn rx_socket(port_base: u16, local: Ipv4Addr) -> io::Result<UdpSocket> {
        let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;

        // All nodes (and the harness sniffer) bind the same group port.
        socket.set_reuse_address(true)?;
        socket.set_reuse_port(true)?;

        socket.set_multicast_if_v4(&local)?;
        socket.join_multicast_v4(&GROUP, &local)?;

        // Bound to the group address itself, so the socket receives exactly
        // the simulation traffic and no stray unicast.
        socket.bind(&SocketAddrV4::new(GROUP, port_base).into())?;

        Ok(socket.into())
    }

    fn tx_socket(port: u16, local: Ipv4Addr) -> io::Result<UdpSocket> {
        let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;

        socket.set_multicast_if_v4(&local)?;
        // All nodes live on this one host: without loopback the datagrams
        // would reach nobody. The price is that a node receives its own
        // frames too - discarded in `receive` by source port.
        socket.set_multicast_loop_v4(true)?;

        // The source port is the node's identity on the medium.
        socket.bind(&SocketAddrV4::new(local, port).into())?;

        Ok(socket.into())
    }
}

impl Radio for SimRadio {
    type Error = SimRadioError;

    async fn init(&mut self) -> Result<RadioCaps, Self::Error> {
        // A bare PHY, like the nRF driver: `MacRadio` / OpenThread handle
        // ACKs, filtering, retries and frame security in software.
        Ok(RadioCaps::default())
    }

    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error> {
        self.config = config.clone();

        // `set_config` is the first call of any post-sleep operation, which
        // makes it the precise wake boundary: everything in the socket at
        // this point arrived while asleep and is discarded (contract point
        // 5), while nothing legitimate can be lost - our own wake-up
        // transmission (e.g. a data poll) has not gone out yet, so no reply
        // to it can exist. Flushing any later (say, on the first `receive`,
        // which for a poll happens inside the ACK wait) would race the
        // parent's microsecond-scale ACK on the loopback medium.
        if self.slept {
            self.slept = false;

            let mut buf = [0; PSDU_MAX + 1];
            while self.rx.as_ref().recv_from(&mut buf).is_ok() {}
        }

        Ok(())
    }

    async fn transmit(
        &mut self,
        psdu: &[u8],
        _cca: bool, // The simulated channel is always idle
        _ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Self::Error> {
        if !(2..=PSDU_MAX).contains(&psdu.len()) {
            return Err(SimRadioError::TxInvalid);
        }

        let mut msg = [0; PSDU_MAX + 1];
        msg[0] = self.config.channel;
        msg[1..1 + psdu.len()].copy_from_slice(psdu);
        patch_fcs(&mut msg[1..1 + psdu.len()]);

        self.tx
            .send_to(
                &msg[..1 + psdu.len()],
                SocketAddr::from((GROUP, self.port_base)),
            )
            .await?;

        // No TX ACK offload (see `init` caps): reception of the ACK - when the
        // frame requests one - is `MacRadio`'s software emulation's job.
        Ok(None)
    }

    async fn receive(&mut self, psdu_buf: &mut [u8]) -> Result<PsduMeta, Self::Error> {
        loop {
            let mut msg = [0; PSDU_MAX + 1];

            let (len, sender) = self.rx.recv_from(&mut msg).await?;

            let SocketAddr::V4(sender) = sender else {
                continue;
            };

            // The sending node is identified by its source port.
            if sender.port().wrapping_sub(self.port_base) == self.node_id {
                continue;
            }

            // Datagram = channel byte + PSDU (>= FCS).
            if !(3..=PSDU_MAX + 1).contains(&len) {
                continue;
            }

            let channel = msg[0];
            if channel != self.config.channel {
                continue;
            }

            let psdu_len = len - 1;
            if psdu_len > psdu_buf.len() {
                continue;
            }

            psdu_buf[..psdu_len].copy_from_slice(&msg[1..len]);

            break Ok(PsduMeta {
                len: psdu_len,
                channel,
                rssi: Some(SIM_RSSI),
            });
        }
    }

    async fn sleep(&mut self) -> Result<(), Self::Error> {
        self.slept = true;
        Ok(())
    }
}

/// Compute the effective port base the same way the C simulation platform
/// does: `PORT_BASE + PORT_OFFSET * (max network size + 1)`, defaulting to
/// `PORT_BASE` 9000 and `PORT_OFFSET` 0.
pub fn port_base_from_env() -> u16 {
    let port_base = env_u16("PORT_BASE").unwrap_or(DEFAULT_PORT_BASE);
    let port_offset = env_u16("PORT_OFFSET").unwrap_or(0);

    port_base + port_offset * (MAX_NETWORK_SIZE + 1)
}

fn env_u16(name: &str) -> Option<u16> {
    std::env::var(name).ok()?.parse().ok()
}

/// Overwrite the last two PSDU bytes with the frame's FCS
/// (CRC-16/KERMIT over the preceding bytes, LSB first), as the C simulation
/// platform does on every transmitted frame - the OpenThread stack hands the
/// PSDU over with the FCS bytes unfilled (real radio hardware computes the
/// FCS itself on the way out).
pub(crate) fn patch_fcs(psdu: &mut [u8]) {
    let fcs_offset = psdu.len() - 2;

    let mut fcs = 0_u16;
    for &byte in &psdu[..fcs_offset] {
        fcs ^= byte as u16;
        for _ in 0..u8::BITS {
            fcs = if fcs & 1 != 0 {
                (fcs >> 1) ^ 0x8408 // Reflected 0x1021
            } else {
                fcs >> 1
            };
        }
    }

    psdu[fcs_offset] = fcs as u8;
    psdu[fcs_offset + 1] = (fcs >> 8) as u8;
}

#[cfg(test)]
mod tests {
    /// The CRC-16/KERMIT check value for the input "123456789".
    #[test]
    fn fcs() {
        let mut psdu = *b"123456789\0\0";
        super::patch_fcs(&mut psdu);
        assert_eq!([psdu[9], psdu[10]], [0x89, 0x21]); // 0x2189, LSB first
    }
}
