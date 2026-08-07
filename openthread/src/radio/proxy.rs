use core::cell::{Cell, RefCell};
use core::fmt::Debug;
use core::future::Future;
use core::mem::MaybeUninit;

use embassy_futures::select::{select, Either};

use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_sync::zerocopy_channel::{Channel, Receiver, Sender};

use crate::fmt::Bytes;
use crate::sys::{OT_RADIO_FRAME_MAX_SIZE, OT_RADIO_RSSI_INVALID};
use crate::{Config, PsduMeta, Radio, RadioCaps, RadioError as _, RadioErrorKind, SrcMatchConfig};

/// The resources for the radio proxy.
pub struct ProxyRadioResources {
    rx_buf: MaybeUninit<[ProxyRadioFrame; 1]>,
    state: MaybeUninit<ProxyRadioState<'static>>,
}

impl ProxyRadioResources {
    /// Create a new set of radio proxy resources.
    pub const fn new() -> Self {
        Self {
            rx_buf: MaybeUninit::uninit(),
            state: MaybeUninit::uninit(),
        }
    }
}

impl Default for ProxyRadioResources {
    fn default() -> Self {
        Self::new()
    }
}

/// A type that allows to offload the execution (TX/RX) of the actual PHY `Radio` impl
/// to a separate - possibly higher-priority - executor.
///
/// Running the PHY radio in a separate higher priority executor is particularly desirable in the cases where it
/// cannot do MAC-offloading (ACKs and filtering) in hardware, and hence the `MacRadio` wrapper is used to handle
/// these tasks in software. Due to timing constraints with ACKs and filtering, this task should have a higher
/// priority than all other `OpenThread`-related tasks.
///
/// This is achieved by splitting the radio into two types:
/// - `ProxyRadio`, which is a radio proxy that implements the `Radio` trait and is to be used by the main execution
///   by passing it to `OpenThread::run`
/// - `PhyRadioRunner`, which is `Send` and therefore can be sent to a separate executor - to run the radio.
///   Invoke `PhyRadioRunner::run(<the-phy-radio>, <delay-provider>).await` in that separate executor.
///
/// # Wire protocol
///
/// The two halves talk over two independent primitives, because the two
/// directions have genuinely different semantics:
///
/// - **Commands** (everything except [`Radio::receive`]) use a *rendezvous*:
///   a single shared slot holding at most one outstanding command plus its
///   response, guarded by a blocking mutex, with a signal in each direction.
///   There is no queue, and that is the point: cancelling a command is simply
///   overwriting (or clearing) the slot, so the two halves can never disagree
///   about which command is in flight.
///
/// - **Received frames** flow the other way over their own channel, decoupled
///   from the command rendezvous. A frame is committed to the channel with no
///   await in between, and taken out of it only once fully copied, so dropping
///   either side's future can never lose a frame - which is exactly what
///   [`Radio::receive`]'s cancellation-safety contract demands.
pub struct ProxyRadio<'a> {
    /// The received frames channel: the runner pushes, we pop.
    rx: Receiver<'a, CriticalSectionRawMutex, ProxyRadioFrame>,
    /// The command rendezvous shared with the runner.
    exchange: &'a Mutex<CriticalSectionRawMutex, RefCell<Exchange>>,
    /// Raised by us whenever we publish or withdraw a command.
    cmd: &'a Signal<CriticalSectionRawMutex, ()>,
    /// Raised by the runner whenever it publishes a response.
    resp: &'a Signal<CriticalSectionRawMutex, ()>,
    /// The PHY radio's capabilities, published by the runner. `init` awaits it.
    caps: &'a Signal<CriticalSectionRawMutex, RadioCaps>,
}

impl<'a> ProxyRadio<'a> {
    const INIT_RX: [ProxyRadioFrame; 1] = [ProxyRadioFrame::new()];

    /// Create a new `ProxyRadio` and its `PhyRadioRunner` instances.
    ///
    /// Arguments:
    /// - `resources`: The radio proxy resources
    pub fn new(resources: &'a mut ProxyRadioResources) -> (Self, PhyRadioRunner<'a>) {
        resources.rx_buf.write(Self::INIT_RX);

        let rx_buf = unsafe { resources.rx_buf.assume_init_mut() };

        resources.state.write(ProxyRadioState::new(unsafe {
            core::mem::transmute::<&mut [ProxyRadioFrame; 1], &'static mut [ProxyRadioFrame; 1]>(
                rx_buf,
            )
        }));

        let state = unsafe { resources.state.assume_init_mut() };

        state.split()
    }

    /// Hand `request` to the PHY radio and await its response.
    ///
    /// Publishing replaces whatever command was in the slot and clears the
    /// response to it in the same breath. Dropping this future simply empties
    /// the slot - so cancellation needs no acknowledgement from the runner, and
    /// there is never a stale response to drain.
    async fn exec(&mut self, request: ProxyRadioRequest) -> ProxyRadioResponse {
        // Copied out of `self` so the drop guard below borrows these directly
        // rather than through `self` (which is mutably borrowed by the await).
        let exchange = self.exchange;
        let cmd = self.cmd;
        let resp = self.resp;

        trace!("ProxyRadio, command: {:?}", request);

        // Signals are raised and cleared under the same lock as the slot they
        // describe, so that a publish/withdraw by us and a take by the runner
        // can never interleave into a lost or a stale wakeup.
        exchange.lock(|exchange| {
            let mut exchange = exchange.borrow_mut();

            exchange.request = Some(request);
            exchange.response = None;

            resp.reset();
            cmd.signal(());
        });

        let completed = Cell::new(false);

        let _guard = scopeguard::guard((), |_| {
            if !completed.get() {
                exchange.lock(|exchange| {
                    let mut exchange = exchange.borrow_mut();

                    // A response already in the slot means the runner is done
                    // with this command and has nothing left to interrupt -
                    // signalling it would only cost it a restarted `receive`.
                    let interrupted = exchange.response.take().is_none();

                    exchange.request = None;

                    if interrupted {
                        cmd.signal(());
                    }
                });

                trace!("ProxyRadio, command withdrawn");
            }
        });

        let response = loop {
            resp.wait().await;

            if let Some(response) = exchange.lock(|exchange| exchange.borrow_mut().response.take())
            {
                break response;
            }
        };

        completed.set(true);

        trace!("ProxyRadio, response: {:?}", response);

        response
    }
}

impl Radio for ProxyRadio<'_> {
    type Error = RadioErrorKind;

    async fn init(&mut self) -> Result<RadioCaps, Self::Error> {
        // The actual radio (and its `init`) lives on the `PhyRadioRunner`, which
        // publishes the discovered capabilities to the shared `caps` signal.
        // Await them here so the proxy reports the real, runtime-discovered set —
        // both PHY and MAC — just like every other radio. (The runner must be
        // running for TX/RX to work at all, so this resolves once it has brought
        // the radio up.)
        let caps = self.caps.wait().await;

        // Put it back, so that a second `init` answers instead of hanging.
        self.caps.signal(caps);

        Ok(caps)
    }

    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error> {
        self.exec(ProxyRadioRequest::Config(config.clone()))
            .await
            .result
    }

    async fn set_src_match_config(&mut self, config: &SrcMatchConfig) -> Result<(), Self::Error> {
        self.exec(ProxyRadioRequest::SrcMatch(config.clone()))
            .await
            .result
    }

    async fn set_receive(&mut self, channel: u8) -> Result<(), Self::Error> {
        self.exec(ProxyRadioRequest::Receive { channel })
            .await
            .result
    }

    async fn set_sleep(&mut self) -> Result<(), Self::Error> {
        self.exec(ProxyRadioRequest::Sleep).await.result
    }

    async fn energy_scan(&mut self, channel: u8, duration_millis: u16) -> Result<i8, Self::Error> {
        let response = self
            .exec(ProxyRadioRequest::EnergyScan {
                channel,
                duration_millis,
            })
            .await;

        response.result.map(|_| response.energy)
    }

    async fn transmit(
        &mut self,
        psdu: &[u8],
        channel: u8,
        power: i8,
        cca_threshold: Option<i8>,
        ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Self::Error> {
        trace!("ProxyRadio, about to transmit: {}", Bytes(psdu));

        let response = self
            .exec(ProxyRadioRequest::Transmit {
                psdu: unwrap!(heapless::Vec::from_slice(psdu)),
                channel,
                power,
                cca_threshold,
            })
            .await;

        let psdu_meta = (ack_psdu_buf.is_some() && !response.psdu.is_empty()).then_some(PsduMeta {
            len: response.psdu.len(),
            channel: response.psdu_channel,
            rssi: response.psdu_rssi,
        });

        if let Some(ack_psdu_buf) = ack_psdu_buf {
            if psdu_meta.is_some() {
                ack_psdu_buf[..response.psdu.len()].copy_from_slice(&response.psdu);
            } else {
                ack_psdu_buf.fill(0);
            }
        }

        response.result.map(|_| psdu_meta)
    }

    async fn receive(&mut self, psdu_buf: &mut [u8]) -> Result<PsduMeta, Self::Error> {
        trace!("ProxyRadio, about to receive");

        // Cancellation-safe by construction: the only await is the channel pop,
        // and the frame is not consumed (`receive_done`) until it has been fully
        // copied out, with no await in between.
        let frame = self.rx.receive().await;

        let result = frame.result;

        if let Ok(psdu_meta) = &result {
            psdu_buf[..psdu_meta.len].copy_from_slice(&frame.psdu[..psdu_meta.len]);
        }

        self.rx.receive_done();

        trace!("ProxyRadio, receive done: {:?}", result);

        result
    }
}

/// A type modeling the running of the PHY radio - the other side of the `ProxyRadio` pipe.
pub struct PhyRadioRunner<'a> {
    /// The received frames channel: we push, the proxy pops.
    rx: Sender<'a, CriticalSectionRawMutex, ProxyRadioFrame>,
    /// The command rendezvous shared with the proxy.
    exchange: &'a Mutex<CriticalSectionRawMutex, RefCell<Exchange>>,
    /// Raised by the proxy whenever it publishes or withdraws a command.
    cmd: &'a Signal<CriticalSectionRawMutex, ()>,
    /// Raised by us whenever we publish a response.
    resp: &'a Signal<CriticalSectionRawMutex, ()>,
    /// The signal on which we publish the actual radio's capabilities (after
    /// running its `init`) so the `ProxyRadio` half can report them.
    caps: &'a Signal<CriticalSectionRawMutex, RadioCaps>,
}

impl PhyRadioRunner<'_> {
    /// Run the PHY radio.
    ///
    /// The radio must offload the complete MAC
    /// ([`MacCapabilities::REQUIRED`](crate::MacCapabilities::REQUIRED)), or
    /// this method panics - wrap a bare PHY in a [`MacRadio`](crate::MacRadio)
    /// first. Doing
    /// the wrapping *here*, on the runner's side, is the whole point of the
    /// proxy: the software MAC's ACK deadlines then get this (higher-priority)
    /// executor to meet them in.
    ///
    /// Arguments:
    /// - `radio`: The PHY radio to run.
    pub async fn run<R>(&mut self, mut radio: R) -> !
    where
        R: Radio,
    {
        // Bring the radio up before serving commands and publish its capabilities
        // to the `ProxyRadio` half (which is what the OpenThread stack queries).
        // This is the runtime caps handshake across the executor boundary: the
        // actual radio lives here, so only here can its `init` run. On failure we
        // publish a default (empty) set rather than leave the proxy's `init`
        // waiting forever; the radio may still recover lazily on the first
        // command.
        let caps = match radio.init().await {
            Ok(caps) => caps,
            Err(e) => {
                warn!("PhyRadioRunner, radio init failed: {:?}", dbg2fmt!(e));
                RadioCaps::default()
            }
        };

        caps.mac.assert_required();

        self.caps.signal(caps);

        debug!("PhyRadioRunner, running");

        let cmd = self.cmd;

        // Whether the radio is currently in receive state, i.e. whether the last
        // state command was `Receive` rather than `Sleep`. The RX pump below is
        // gated on it: `Radio::receive` is what physically drives the receiver
        // in the PHY drivers, so pumping it unconditionally would keep the
        // receiver powered through the sleep periods of a sleepy end device -
        // and deliver frames that a sleeping radio is supposed to miss.
        let mut receiving = false;

        loop {
            // Taking the command and clearing its signal happen under the same
            // lock the proxy publishes under, so a command issued right at this
            // moment is either taken here or leaves its signal standing - never
            // dropped, and never mistaken for a cancellation of what we take.
            let taken = self.exchange.lock(|exchange| {
                let taken = exchange.borrow_mut().request.take();

                if taken.is_some() {
                    cmd.reset();
                }

                taken
            });

            if let Some(request) = taken {
                trace!("PhyRadioRunner, processing command: {:?}", request);

                let mut response = ProxyRadioResponse::new();

                // A command owns the radio for its whole duration - in
                // particular, the RX pump below is not polled while a
                // `transmit` is waiting for its ACK, as the `Radio` contract
                // requires. Only a *newer* command may interrupt it.
                if Self::with_cancel(Self::process(&mut radio, &request, &mut response), cmd)
                    .await
                    .is_some()
                {
                    trace!("PhyRadioRunner, command done: {:?}", response);

                    if response.result.is_ok() {
                        match request {
                            ProxyRadioRequest::Receive { .. } => receiving = true,
                            ProxyRadioRequest::Sleep => receiving = false,
                            _ => (),
                        }
                    }

                    self.publish(response);
                } else {
                    trace!("PhyRadioRunner, command cancelled");
                }
            } else if receiving {
                // Idle and receiving: pump received frames to the proxy until a
                // command shows up.
                Self::with_cancel(Self::pump_rx(&mut radio, &mut self.rx), cmd).await;
            } else {
                // Idle and sleeping: nothing to do until a command shows up.
                cmd.wait().await;
            }
        }
    }

    /// Publish `response` as the answer to the command we took.
    ///
    /// We emptied the command slot when we took it, so a slot that is occupied
    /// again means the proxy has published a *newer* command while we were
    /// executing this one: the response has no recipient and must not be left
    /// behind for that newer command to pick up.
    fn publish(&self, response: ProxyRadioResponse) {
        let published = self.exchange.lock(|exchange| {
            let mut exchange = exchange.borrow_mut();

            if exchange.request.is_none() {
                exchange.response = Some(response);
                self.resp.signal(());

                true
            } else {
                false
            }
        });

        if !published {
            trace!("PhyRadioRunner, response dropped (command superseded)");
        }
    }

    /// Receive a single frame into the proxy's frame channel.
    ///
    /// Lossless under cancellation: the frame is committed with `send_done`
    /// immediately after `receive` returns, with no await in between, so a
    /// dropped future is always dropped either before anything was received or
    /// after the frame was handed over.
    async fn pump_rx<R>(radio: &mut R, rx: &mut Sender<'_, impl RawMutex, ProxyRadioFrame>)
    where
        R: Radio,
    {
        let frame = rx.send().await;

        frame.result = radio
            .receive(&mut frame.psdu)
            .await
            .map_err(|e| e.kind())
            .inspect(|psdu_meta| trace!("PhyRadioRunner, got frame: {:?}", psdu_meta));

        rx.send_done();
    }

    /// Execute a single command against the PHY radio, filling in `response`.
    ///
    /// May be cancelled at any await point by a newer command; a cancelled
    /// command produces no response at all (the proxy is no longer waiting for
    /// one - it withdrew or replaced it).
    async fn process<R>(
        radio: &mut R,
        request: &ProxyRadioRequest,
        response: &mut ProxyRadioResponse,
    ) where
        R: Radio,
    {
        match request {
            ProxyRadioRequest::Config(config) => {
                response.result = radio.set_config(config).await.map_err(|e| e.kind());
            }
            ProxyRadioRequest::SrcMatch(config) => {
                response.result = radio
                    .set_src_match_config(config)
                    .await
                    .map_err(|e| e.kind());
            }
            ProxyRadioRequest::Receive { channel } => {
                response.result = radio.set_receive(*channel).await.map_err(|e| e.kind());
            }
            ProxyRadioRequest::Sleep => {
                response.result = radio.set_sleep().await.map_err(|e| e.kind());
            }
            ProxyRadioRequest::EnergyScan {
                channel,
                duration_millis,
            } => {
                response.result = radio
                    .energy_scan(*channel, *duration_millis)
                    .await
                    .map_err(|e| e.kind())
                    .map(|energy| response.energy = energy);
            }
            ProxyRadioRequest::Transmit {
                psdu,
                channel,
                power,
                cca_threshold,
            } => {
                unwrap!(response.psdu.resize_default(response.psdu.capacity()));

                let result = radio
                    .transmit(
                        psdu,
                        *channel,
                        *power,
                        *cca_threshold,
                        Some(&mut response.psdu),
                    )
                    .await
                    .map_err(|e| e.kind());

                if let Ok(Some(psdu_meta)) = &result {
                    response.psdu.truncate(psdu_meta.len);
                    response.psdu_channel = psdu_meta.channel;
                    response.psdu_rssi = psdu_meta.rssi;
                } else {
                    // No ACK frame returned
                    response.psdu.clear();
                }

                response.result = result.map(|_| ());
            }
        }
    }

    async fn with_cancel<F>(fut: F, cancel: &Signal<impl RawMutex, ()>) -> Option<F::Output>
    where
        F: Future,
    {
        match select(fut, cancel.wait()).await {
            Either::First(result) => Some(result),
            Either::Second(_) => None,
        }
    }
}

// Should be safe because while not (yet) marked formally as such, zerocopy-channel's
// `Sender` is `Send`, as long as the critical section is `Send` + `Sync`
// (which is the case as we use `CriticalSectionRawMutex`), and `ProxyRadioFrame`
// is `Send` (which is the case).
//
// The blocking mutex and the signals are obviously `Send` + `Sync`.
unsafe impl Send for PhyRadioRunner<'_> {}

const PSDU_LEN: usize = OT_RADIO_FRAME_MAX_SIZE as _;

/// The state of the proxy radio
///
/// This state is borrowed and shared between
/// the two ends of the pipe: the proxy radio, and the PHY radio runner.
struct ProxyRadioState<'a> {
    /// The received frames channel from the PHY radio
    rx: Channel<'a, CriticalSectionRawMutex, ProxyRadioFrame>,
    /// The command rendezvous
    exchange: Mutex<CriticalSectionRawMutex, RefCell<Exchange>>,
    /// The signal raised by the proxy radio when it publishes or withdraws a command
    cmd: Signal<CriticalSectionRawMutex, ()>,
    /// The signal raised by the PHY radio runner when it publishes a response
    resp: Signal<CriticalSectionRawMutex, ()>,
    /// The PHY radio's capabilities, published by the runner (which owns the
    /// actual radio and runs its `init`) once, and awaited by `ProxyRadio::init`.
    /// This is how the proxy learns the caps at runtime instead of baking them
    /// in — the actual radio lives on the runner's (possibly separate) executor.
    caps: Signal<CriticalSectionRawMutex, RadioCaps>,
}

impl<'a> ProxyRadioState<'a> {
    /// Create a new proxy radio state.
    ///
    /// Arguments:
    /// - `rx_buf`: The received frames buffer
    fn new(rx_buf: &'a mut [ProxyRadioFrame; 1]) -> Self {
        Self {
            rx: Channel::new(rx_buf),
            exchange: Mutex::new(RefCell::new(Exchange::new())),
            cmd: Signal::new(),
            resp: Signal::new(),
            caps: Signal::new(),
        }
    }

    /// Split the state into the proxy radio and the PHY radio runner.
    fn split(&mut self) -> (ProxyRadio<'_>, PhyRadioRunner<'_>) {
        let (rx_sender, rx_receiver) = self.rx.split();

        (
            ProxyRadio {
                rx: rx_receiver,
                exchange: &self.exchange,
                cmd: &self.cmd,
                resp: &self.resp,
                caps: &self.caps,
            },
            PhyRadioRunner {
                rx: rx_sender,
                exchange: &self.exchange,
                cmd: &self.cmd,
                resp: &self.resp,
                caps: &self.caps,
            },
        )
    }
}

/// The command rendezvous between the two halves of the proxy.
///
/// At most one command is ever outstanding, which is why this is a single slot
/// rather than a queue: `OpenThread` drives the radio one operation at a time,
/// and abandoning an operation is expressed by *replacing* what is in the slot,
/// not by queueing a cancellation behind it.
///
/// The slot doubles as the token that keeps the two halves in agreement, so no
/// sequencing beyond it is needed:
/// - the runner *takes* a command by emptying the slot, and answers only if it
///   is still empty when it finishes - an occupied slot means a newer command
///   has landed and this response has no recipient;
/// - publishing a command clears the response in the same locked section, so a
///   response for an abandoned command is always overwritten before it can be
///   mistaken for the answer to the next one.
///
/// Both signals are likewise raised and cleared under this lock, so a command
/// issued at the exact moment the runner takes one can neither be lost nor be
/// mistaken for a cancellation of what was taken.
struct Exchange {
    /// The command awaiting execution, if the runner has not taken it yet.
    request: Option<ProxyRadioRequest>,
    /// The response to the outstanding command, once the runner has completed it.
    response: Option<ProxyRadioResponse>,
}

impl Exchange {
    /// Create a new, empty command rendezvous.
    const fn new() -> Self {
        Self {
            request: None,
            response: None,
        }
    }
}

/// A proxy radio command: a single [`Radio`] operation to be executed by the
/// PHY radio on the runner's executor.
///
/// [`Radio::receive`] is deliberately absent - received frames flow over their
/// own channel ([`ProxyRadioFrame`]) rather than as a request/response pair.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum ProxyRadioRequest {
    /// [`Radio::set_config`]
    Config(Config),
    /// [`Radio::set_src_match_config`]
    SrcMatch(SrcMatchConfig),
    /// [`Radio::set_receive`]
    Receive { channel: u8 },
    /// [`Radio::set_sleep`]
    Sleep,
    /// [`Radio::energy_scan`]
    EnergyScan { channel: u8, duration_millis: u16 },
    /// [`Radio::transmit`]
    Transmit {
        psdu: heapless::Vec<u8, PSDU_LEN>,
        channel: u8,
        power: i8,
        cca_threshold: Option<i8>,
    },
}

/// A proxy radio response.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct ProxyRadioResponse {
    /// The result of the operation
    result: Result<(), RadioErrorKind>,
    /// The maximum energy observed, for a successful energy scan
    energy: i8,
    /// The received ACK PSDU, for a successful transmit (might be empty)
    psdu: heapless::Vec<u8, PSDU_LEN>,
    /// The channel on which the ACK frame was received
    psdu_channel: u8,
    /// The RSSI of the received ACK frame, if the radio supports appending it
    /// at the end of the frame
    psdu_rssi: Option<i8>,
}

impl ProxyRadioResponse {
    /// Create a new empty proxy radio response.
    const fn new() -> Self {
        Self {
            result: Ok(()),
            energy: OT_RADIO_RSSI_INVALID as i8,
            psdu: heapless::Vec::new(),
            psdu_channel: 0,
            psdu_rssi: None,
        }
    }
}

/// A frame received by the PHY radio, on its way to the proxy.
struct ProxyRadioFrame {
    /// The outcome of the receive operation - the frame meta-data on success
    result: Result<PsduMeta, RadioErrorKind>,
    /// The received PSDU, valid up to `result`'s length on success
    psdu: [u8; PSDU_LEN],
}

impl ProxyRadioFrame {
    /// Create a new empty proxy radio frame.
    const fn new() -> Self {
        Self {
            result: Ok(PsduMeta {
                len: 0,
                channel: 0,
                rssi: None,
            }),
            psdu: [0; PSDU_LEN],
        }
    }
}
