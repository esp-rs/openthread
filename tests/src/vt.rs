//! The virtual-time side of the simulated radio: the event link to the
//! upstream Python simulator (`tests/scripts/thread-cert/simulator.py`,
//! `VirtualTime`) and a [`Radio`] whose frames travel as simulator events.
//!
//! Wire protocol (identical to the C `virtual_time` platform): the node
//! binds UDP `127.0.0.1:(port base + node id)` and exchanges packed events
//! with the simulator at `port base`. An event is an 11-byte header - `u64`
//! delay (µs, relative to the *node's* clock), `u8` type, `u16` data length,
//! all little-endian - followed by the data. A transmitted frame is a
//! `RADIO_RECEIVED` event (channel byte + PSDU) with delay 1; the simulator
//! fans it out to every other node *and echoes it back to the sender* (the C
//! platform's TX-done signal - here the echo is simply dropped, since
//! [`Radio::transmit`] completes eagerly). Event pacing and the virtual
//! clock live in the executor (see [`crate::executor`]).

use core::fmt;

use std::collections::VecDeque;
use std::io;
use std::net::{Ipv4Addr, UdpSocket};
use std::os::fd::{AsFd, BorrowedFd};
use std::sync::Arc;

use openthread::{Config, PsduMeta, Radio, RadioCaps, RadioError, RadioErrorKind};

use crate::sim_radio::{patch_fcs, port_base_from_env, PSDU_MAX, SIM_RSSI};

/// Simulator event types (the subset a CLI node exchanges).
const EVENT_ALARM_FIRED: u8 = 0;
const EVENT_RADIO_RECEIVED: u8 = 1;

/// The packed event header: `u64` delay + `u8` type + `u16` data length.
const EVENT_HEADER: usize = 11;

/// Maximum event data (the C platform's `OT_EVENT_DATA_MAX_SIZE`).
const EVENT_DATA_MAX: usize = 1024;

/// An 802.15.4 frame carried by a radio event.
pub struct VtFrame {
    channel: u8,
    len: usize,
    psdu: [u8; PSDU_MAX],
}

impl VtFrame {
    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The MAC sequence number, for trace correlation.
    pub fn seq(&self) -> u8 {
        self.psdu.get(2).copied().unwrap_or(0)
    }
}

/// A received simulator event, minus the already-consumed delay.
pub enum VtEventKind {
    /// Time advance only.
    Alarm,
    /// A frame on the simulated medium.
    RadioFrame(VtFrame),
    /// An event type this node does not model.
    Other(u8),
}

/// Frames on their way from the executor's event loop to [`VtRadio`].
///
/// Unbounded, because the C platform this mirrors never drops either (it
/// processes each event synchronously): attach-time bursts (advertisements,
/// responses, ACKs, plus the echoes of own transmissions) can outpace the
/// radio task, which interleaves OT processing - and possibly virtual-time
/// waits, served by further simulator events - between reads, so any fixed
/// bound would either drop frames or deadlock the time advance. Memory
/// stays bounded in practice: the lockstep protocol paces the producer.
static VT_RX: VtRxQueue = VtRxQueue {
    state: std::sync::Mutex::new(VtRxState {
        frames: VecDeque::new(),
        waker: None,
    }),
};

struct VtRxState {
    frames: VecDeque<VtFrame>,
    waker: Option<core::task::Waker>,
}

struct VtRxQueue {
    state: std::sync::Mutex<VtRxState>,
}

impl VtRxQueue {
    fn push(&self, frame: VtFrame) {
        let mut state = self.state.lock().unwrap();
        state.frames.push_back(frame);
        if let Some(waker) = state.waker.take() {
            waker.wake();
        }
    }

    async fn receive(&self) -> VtFrame {
        core::future::poll_fn(|cx| {
            let mut state = self.state.lock().unwrap();
            match state.frames.pop_front() {
                Some(frame) => core::task::Poll::Ready(frame),
                None => {
                    state.waker = Some(cx.waker().clone());
                    core::task::Poll::Pending
                }
            }
        })
        .await
    }

    fn try_receive(&self) -> Option<VtFrame> {
        self.state.lock().unwrap().frames.pop_front()
    }
}

/// Hand a received frame to the radio (called by the executor's run loop).
pub(crate) fn deliver_frame(frame: VtFrame) {
    VT_RX.push(frame);
}

/// The event link to the simulator; cheaply cloneable, the clones share one
/// socket (the executor receives, [`VtRadio`] transmits).
#[derive(Clone)]
pub struct VtLink {
    sock: Arc<UdpSocket>,
    sim_port: u16,
}

impl VtLink {
    /// Bind the event link of simulation node `node_id`, with the port base
    /// taken from `PORT_BASE`/`PORT_OFFSET` exactly like the C platform.
    pub fn new(node_id: u16) -> io::Result<Self> {
        let port_base = port_base_from_env();
        let sock = UdpSocket::bind((Ipv4Addr::LOCALHOST, port_base + node_id))?;

        Ok(Self {
            sock: Arc::new(sock),
            sim_port: port_base,
        })
    }

    /// The socket to `poll(2)` for incoming events.
    pub fn fd(&self) -> BorrowedFd<'_> {
        self.sock.as_fd()
    }

    /// Report "idle; next alarm in `delay` µs" to the simulator.
    ///
    /// Sent on every idle pass, like the C platform sends one per `select`
    /// entry: beyond superseding the alarm, the sleep event is also the
    /// synchronization ack the simulator's barriers wait for - after
    /// delivering us an event, and after `go(0, nodeid)` marked us awake on
    /// CLI input. De-duplicating it deadlocks those barriers.
    pub fn send_sleep(&self, delay: u64) {
        self.send_event(delay, EVENT_ALARM_FIRED, &[]);
    }

    fn send_event(&self, delay: u64, kind: u8, data: &[u8]) {
        let mut event = [0; EVENT_HEADER + EVENT_DATA_MAX];
        event[..8].copy_from_slice(&delay.to_le_bytes());
        event[8] = kind;
        event[9..11].copy_from_slice(&(data.len() as u16).to_le_bytes());
        event[EVENT_HEADER..EVENT_HEADER + data.len()].copy_from_slice(data);

        self.sock
            .send_to(
                &event[..EVENT_HEADER + data.len()],
                (Ipv4Addr::LOCALHOST, self.sim_port),
            )
            .expect("send simulator event");
    }

    /// Receive one event; call only when the socket is readable. Returns the
    /// delay to advance the virtual clock by, and the event itself.
    pub fn recv(&self) -> io::Result<(u64, VtEventKind)> {
        let mut buf = [0; EVENT_HEADER + EVENT_DATA_MAX];
        let len = self.sock.recv(&mut buf)?;

        if len < EVENT_HEADER {
            return Err(io::Error::other("simulator event shorter than its header"));
        }

        let delay = u64::from_le_bytes(buf[..8].try_into().unwrap());
        let kind = buf[8];
        let data_len = u16::from_le_bytes(buf[9..11].try_into().unwrap()) as usize;
        let data = &buf[EVENT_HEADER..(EVENT_HEADER + data_len).min(len)];

        let kind = match kind {
            EVENT_ALARM_FIRED => VtEventKind::Alarm,
            // Channel byte + PSDU, which must at least carry an FCS.
            EVENT_RADIO_RECEIVED if (3..=PSDU_MAX + 1).contains(&data.len()) => {
                let mut frame = VtFrame {
                    channel: data[0],
                    len: data.len() - 1,
                    psdu: [0; PSDU_MAX],
                };
                frame.psdu[..frame.len].copy_from_slice(&data[1..]);

                VtEventKind::RadioFrame(frame)
            }
            other => VtEventKind::Other(other),
        };

        Ok((delay, kind))
    }
}

/// The error type of [`VtRadio`].
#[derive(Debug)]
pub enum VtRadioError {
    /// A TX PSDU too short to carry an FCS or longer than a frame.
    TxInvalid,
}

impl RadioError for VtRadioError {
    fn kind(&self) -> RadioErrorKind {
        match self {
            Self::TxInvalid => RadioErrorKind::TxInvalid,
        }
    }
}

impl fmt::Display for VtRadioError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TxInvalid => write!(f, "invalid TX PSDU"),
        }
    }
}

/// A [`Radio`] on the virtual-time simulated medium.
///
/// Like [`SimRadio`](crate::sim_radio::SimRadio), a bare PHY meant to be
/// wrapped in `MacRadio` - ACKs and filtering happen in software, and their
/// timing (ACK waits, retries) runs on virtual time like everything else.
pub struct VtRadio {
    link: VtLink,
    config: Config,
    /// Copies of recently transmitted frames: the simulator echoes every
    /// transmission back to its sender (the C platform's TX-done signal),
    /// and those echoes must not surface as received frames. A queue,
    /// because back-to-back unacked transmissions can outpace the echoes.
    echoes: VecDeque<(usize, [u8; PSDU_MAX])>,
    /// Set by [`Radio::sleep`]; the next `receive` flushes what accumulated
    /// while asleep (contract point 5), still settling echo bookkeeping.
    slept: bool,
}

impl VtRadio {
    pub fn new(link: VtLink) -> Self {
        Self {
            link,
            config: Config::new(),
            echoes: VecDeque::new(),
            slept: false,
        }
    }

    /// Drop the echo of an own transmission - matched against the OLDEST
    /// outstanding entry only. Echoes return in transmit order (the
    /// simulator stamps events monotonically), and an echo always precedes
    /// any *foreign* frame with identical bytes: that matters for ACKs,
    /// which are content-degenerate (FCF+seq+FCS) - when two neighbors' MAC
    /// sequence numbers coincide, the peer's ACK **to us** is byte-identical
    /// to the echo of an ACK **we** sent, and a match-anywhere filter would
    /// eat the real ACK, blinding `MacRadio`'s ACK wait into retransmission
    /// storms.
    fn consume_echo(&mut self, frame: &VtFrame) -> bool {
        if let Some((len, echo)) = self.echoes.front() {
            if *len == frame.len && echo[..*len] == frame.psdu[..frame.len] {
                self.echoes.pop_front();
                return true;
            }
        }

        false
    }
}

impl Radio for VtRadio {
    type Error = VtRadioError;

    async fn init(&mut self) -> Result<RadioCaps, Self::Error> {
        // A bare PHY, like `SimRadio`: `MacRadio` emulates the MAC offloads.
        Ok(RadioCaps::default())
    }

    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error> {
        self.config = config.clone();

        // The wake boundary (see `SimRadio::set_config` for the rationale;
        // here the ordering is deterministic, but symmetry keeps the two
        // radios honest): frames queued while asleep are missed - though
        // the echoes of own pre-sleep transmissions among them must still
        // settle the echo bookkeeping, or the FIFO matching desyncs.
        if self.slept {
            self.slept = false;

            while let Some(frame) = VT_RX.try_receive() {
                let _ = self.consume_echo(&frame);
            }
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
            return Err(VtRadioError::TxInvalid);
        }

        let mut data = [0; PSDU_MAX + 1];
        data[0] = self.config.channel;
        data[1..1 + psdu.len()].copy_from_slice(psdu);
        patch_fcs(&mut data[1..1 + psdu.len()]);

        // The simulator will echo this frame back exactly once, so entries
        // pair up with arriving echoes and the queue cannot grow unbounded.
        let mut echo = [0; PSDU_MAX];
        echo[..psdu.len()].copy_from_slice(&data[1..1 + psdu.len()]);
        self.echoes.push_back((psdu.len(), echo));

        log::trace!(
            "VT tx: len={} seq={}",
            psdu.len(),
            psdu.get(2).copied().unwrap_or(0)
        );

        // The 1µs delay is what the C platform uses for every transmission.
        self.link
            .send_event(1, EVENT_RADIO_RECEIVED, &data[..1 + psdu.len()]);

        Ok(None)
    }

    async fn receive(&mut self, psdu_buf: &mut [u8]) -> Result<PsduMeta, Self::Error> {
        loop {
            let frame = VT_RX.receive().await;

            if self.consume_echo(&frame) {
                continue;
            }

            let psdu = &frame.psdu[..frame.len];

            // Diagnostics: an incoming frame matching a NON-front echo entry
            // means echoes do come back out of order - the FIFO premise fails.
            if let Some(index) = self
                .echoes
                .iter()
                .position(|(len, echo)| *len == frame.len && echo[..*len] == *psdu)
            {
                log::warn!(
                    "VT echo out of order: matches entry {index} of {}, seq={} len={}",
                    self.echoes.len(),
                    frame.seq(),
                    frame.len
                );
            }

            if frame.channel != self.config.channel {
                continue;
            }

            if frame.len > psdu_buf.len() {
                continue;
            }

            psdu_buf[..frame.len].copy_from_slice(psdu);

            break Ok(PsduMeta {
                len: frame.len,
                channel: frame.channel,
                rssi: Some(SIM_RSSI),
            });
        }
    }

    async fn sleep(&mut self) -> Result<(), Self::Error> {
        self.slept = true;
        Ok(())
    }
}
