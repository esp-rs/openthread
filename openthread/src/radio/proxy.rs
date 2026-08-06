use core::cell::Cell;
use core::fmt::Debug;
use core::future::Future;
use core::mem::MaybeUninit;

use embassy_futures::select::{select, Either};

use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex};
use embassy_sync::signal::Signal;
use embassy_sync::zerocopy_channel::{Channel, Receiver, Sender};

use crate::fmt::Bytes;
use crate::sys::OT_RADIO_FRAME_MAX_SIZE;
use crate::{Config, PsduMeta, Radio, RadioCaps, RadioError as _, RadioErrorKind};
use crate::{MacRadio, MacRadioTimer};

/// The resources for the radio proxy.
pub struct ProxyRadioResources {
    request_buf: MaybeUninit<[ProxyRadioRequest; 1]>,
    response_buf: MaybeUninit<[ProxyRadioResponse; 1]>,
    state: MaybeUninit<ProxyRadioState<'static>>,
}

impl ProxyRadioResources {
    /// Create a new set of radio proxy resources.
    pub const fn new() -> Self {
        Self {
            request_buf: MaybeUninit::uninit(),
            response_buf: MaybeUninit::uninit(),
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
pub struct ProxyRadio<'a> {
    /// The request channel to the PHY radio
    request: Sender<'a, CriticalSectionRawMutex, ProxyRadioRequest>,
    /// The response channel from the PHY radio
    response: Receiver<'a, CriticalSectionRawMutex, ProxyRadioResponse>,
    /// Whether the current response was cancelled
    /// If set to `true`, this means we have to drop the pending response from the driver
    cancelled: Cell<bool>,
    /// The signal to indicate a new request to the PHY radio, so that
    /// the PHY radio can cancel the current request (if any) and start processing the new one
    cancel: &'a Signal<CriticalSectionRawMutex, ()>,
    /// The PHY radio's capabilities, published by the runner. `init` awaits it.
    caps: &'a Signal<CriticalSectionRawMutex, RadioCaps>,
    /// The current radio configuration
    config: Config,
}

impl<'a> ProxyRadio<'a> {
    const INIT_REQUEST: [ProxyRadioRequest; 1] = [ProxyRadioRequest::new()];
    const INIT_RESPONSE: [ProxyRadioResponse; 1] = [ProxyRadioResponse::new()];

    /// Create a new `ProxyRadio` and its `PhyRadioRunner` instances.
    ///
    /// Arguments:
    /// - `resources`: The radio proxy resources
    pub fn new(resources: &'a mut ProxyRadioResources) -> (Self, PhyRadioRunner<'a>) {
        resources.request_buf.write(Self::INIT_REQUEST);
        resources.response_buf.write(Self::INIT_RESPONSE);

        let request_buf = unsafe { resources.request_buf.assume_init_mut() };
        let response_buf = unsafe { resources.response_buf.assume_init_mut() };

        resources.state.write(ProxyRadioState::new(
            unsafe {
                core::mem::transmute::<
                    &mut [ProxyRadioRequest; 1],
                    &'static mut [ProxyRadioRequest; 1],
                >(request_buf)
            },
            unsafe {
                core::mem::transmute::<
                    &mut [ProxyRadioResponse; 1],
                    &'static mut [ProxyRadioResponse; 1],
                >(response_buf)
            },
        ));

        let state = unsafe { resources.state.assume_init_mut() };

        state.split()
    }

    /// Clear any cancelled response from the driver
    async fn process_cancelled(&mut self) {
        if self.cancelled.get() {
            self.response.receive().await;

            self.cancelled.set(false);
            self.response.receive_done();

            trace!("ProxyRadio, cancelled response cleared");
        }
    }
}

impl Radio for ProxyRadio<'_> {
    type Error = RadioErrorKind;

    async fn init(&mut self) -> Result<RadioCaps, Self::Error> {
        // The actual radio (and its `init`) lives on the `PhyRadioRunner`, which
        // publishes the discovered capabilities to the shared `caps` signal.
        // Await them here so the proxy reports the real, runtime-discovered set —
        // both PHY and MAC — just like every other radio. (The runner wraps the
        // radio in a `MacRadio`, so the MAC set it publishes is the full offload
        // set; the runner must be running for TX/RX to work at all, so this
        // resolves once it has brought the radio up.)
        //
        // TODO: the proxy's request/response protocol carries only TX/RX, so
        // `Radio::energy_scan` is *not* forwarded to the PHY radio: the proxy
        // keeps the default implementation and reports "no measurement". This
        // costs nothing today (no proxied radio can measure channel energy —
        // see the `esp`/`nrf` notes); forward it through the protocol if one
        // ever can.
        Ok(self.caps.wait().await)
    }

    async fn set_config(&mut self, config: &Config) -> Result<(), Self::Error> {
        // There is no separate command for updating the configuration
        // The updated configuration is always valid for the next request
        self.config = config.clone();
        Ok(())
    }

    async fn transmit(
        &mut self,
        psdu: &[u8],
        cca: bool,
        ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Self::Error> {
        trace!("ProxyRadio, about to transmit: {}", Bytes(psdu));

        self.process_cancelled().await;

        {
            let req = self.request.send().await;

            req.tx = true;
            req.config = self.config.clone();
            req.cca = cca;
            req.psdu.clear();
            unwrap!(req.psdu.extend_from_slice(psdu));

            trace!("ProxyRadio, transmit request sent: {:?}", req);

            self.request.send_done();
        }

        self.cancelled.set(true);
        let _guard = scopeguard::guard((), |_| {
            if self.cancelled.get() {
                trace!("ProxyRadio, transmit request cancelled");
                self.cancel.signal(())
            }
        });

        trace!("ProxyRadio, waiting for transmit response");

        let resp = self.response.receive().await;

        trace!("ProxyRadio, transmit response received: {:?}", resp);

        let psdu_meta = (ack_psdu_buf.is_some() && !resp.psdu.is_empty()).then_some(PsduMeta {
            len: resp.psdu.len(),
            channel: resp.psdu_channel,
            rssi: resp.psdu_rssi,
        });

        if let Some(ack_psdu_buf) = ack_psdu_buf {
            if psdu_meta.is_some() {
                ack_psdu_buf[..resp.psdu.len()].copy_from_slice(&resp.psdu);
            } else {
                ack_psdu_buf.fill(0);
            }
        }

        let result = resp.result.map(|_| psdu_meta);

        self.cancelled.set(false);
        self.response.receive_done();

        trace!("ProxyRadio, transmit response done");

        result
    }

    async fn receive(&mut self, psdu_buf: &mut [u8]) -> Result<PsduMeta, Self::Error> {
        trace!("ProxyRadio, about to receive");

        self.process_cancelled().await;

        {
            let req = self.request.send().await;

            req.tx = false;
            req.config = self.config.clone();
            req.psdu.clear();

            trace!("ProxyRadio, receive request sent: {:?}", req);

            self.request.send_done();
        }

        self.cancelled.set(true);
        let _guard = scopeguard::guard((), |_| {
            if self.cancelled.get() {
                trace!("ProxyRadio, receive request cancelled");
                self.cancel.signal(());
            }
        });

        trace!("ProxyRadio, waiting for receive response");

        let resp = self.response.receive().await;

        trace!("ProxyRadio, receive response received: {:?}", resp);

        match resp.result {
            Ok(()) => {
                let len = resp.psdu.len();
                psdu_buf[..len].copy_from_slice(&resp.psdu);

                let psdu_meta = PsduMeta {
                    len,
                    channel: resp.psdu_channel,
                    rssi: resp.psdu_rssi,
                };

                self.cancelled.set(false);
                self.response.receive_done();

                Ok(psdu_meta)
            }
            Err(e) => {
                self.cancelled.set(false);
                self.response.receive_done();
                Err(e)
            }
        }
    }
}

/// A type modeling the running of the PHY radio - the other side of the `ProxyRadio` pipe.
pub struct PhyRadioRunner<'a> {
    /// The request channel from the proxy radio
    request: Receiver<'a, CriticalSectionRawMutex, ProxyRadioRequest>,
    /// The response channel to the proxy radio
    response: Sender<'a, CriticalSectionRawMutex, ProxyRadioResponse>,
    /// The signal to indicate a new request from the proxy radio
    /// which means we have to cancel processing the current request (if any)
    cancel: &'a Signal<CriticalSectionRawMutex, ()>,
    /// The signal on which we publish the actual radio's capabilities (after
    /// running its `init`) so the `ProxyRadio` half can report them.
    caps: &'a Signal<CriticalSectionRawMutex, RadioCaps>,
}

impl PhyRadioRunner<'_> {
    /// Run the PHY radio.
    ///
    /// Arguments:
    /// - `radio`: The PHY radio to run.
    /// - `delay`: The delay implementation to use.
    pub async fn run<R, T>(&mut self, radio: R, delay: T) -> !
    where
        R: Radio,
        T: MacRadioTimer,
    {
        let mut radio = MacRadio::new(radio, delay);

        // Bring the radio up before serving requests and publish its capabilities
        // to the `ProxyRadio` half (which is what the OpenThread stack queries).
        // `radio` is a `MacRadio`, so the MAC set is the full offload set. This is
        // the runtime caps handshake across the executor boundary: the actual
        // radio lives here, so only here can its `init` run. On failure we publish
        // a default (empty) set rather than leave the proxy's `init` waiting
        // forever; the radio may still recover lazily on the first request.
        let caps = match radio.init().await {
            Ok(caps) => caps,
            Err(e) => {
                warn!("PhyRadioRunner, radio init failed: {:?}", e);
                RadioCaps::default()
            }
        };
        self.caps.signal(caps);

        debug!("PhyRadioRunner, running");

        loop {
            {
                let request = self.request.receive().await;

                if Self::process(&mut radio, request, &mut self.response, self.cancel)
                    .await
                    .is_some()
                {
                    trace!("PhyRadioRunner, processing done");
                } else {
                    // Processing was cancelled by a new request, need to send an "interrupted" response

                    let response = self.response.send().await;

                    response.psdu.clear();
                    response.result = Err(RadioErrorKind::Other);
                    response.psdu_channel = 0;
                    response.psdu_rssi = None;

                    self.response.send_done();

                    trace!("PhyRadioRunner, processing cancelled");
                }

                self.request.receive_done();
            }
        }
    }

    // Process a single request (TX or RX) by first updating the driver configuration
    // (driver should skip that if the new configuration is the same as the current one),
    // and then transmitting or receiving the frame.
    //
    // Updating the configuration, as well as the TX/RX operation might be cancelled at
    // any moment, if a new request arrives.
    async fn process<T>(
        mut radio: T,
        request: &mut ProxyRadioRequest,
        response_sender: &mut Sender<'_, impl RawMutex, ProxyRadioResponse>,
        cancel: &Signal<impl RawMutex, ()>,
    ) -> Option<()>
    where
        T: Radio,
    {
        let response = Self::with_cancel(response_sender.send(), cancel).await?;

        response.psdu.clear();
        response.psdu_channel = 0;
        response.psdu_rssi = None;

        trace!("PhyRadioRunner, processing request: {:?}", request);

        // Always first set the configuration relevant for the current TX/RX request
        // The PHY driver should have intelligence to skip the configuration update if the new
        // configuration is the same as the current one
        let result = Self::with_cancel(radio.set_config(&request.config), cancel)
            .await?
            .map_err(|e| e.kind());

        trace!("PhyRadioRunner, configuration set: {:?}", result);

        let result = if result.is_err() {
            // Setting driver configuration resulted in an error, so skip the rest of the processing
            result
        } else {
            unwrap!(response.psdu.resize_default(response.psdu.capacity()));

            let result = if request.tx {
                Self::with_cancel(
                    radio.transmit(&request.psdu, request.cca, Some(&mut response.psdu)),
                    cancel,
                )
                .await?
                .map_err(|e| e.kind())
            } else {
                Self::with_cancel(radio.receive(&mut response.psdu), cancel)
                    .await?
                    .map_err(|e| e.kind())
                    .map(Some)
            };

            if let Ok(Some(psdu_meta)) = &result {
                response.psdu.truncate(psdu_meta.len);
                response.psdu_channel = psdu_meta.channel;
                response.psdu_rssi = psdu_meta.rssi;
            } else {
                // No frame returned, so clear the response fields
                response.psdu.clear();
            }

            result.map(|_| ())
        };

        response.result = result;

        trace!("PhyRadioRunner, processed response: {:?}", response);

        response_sender.send_done();

        Some(())
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
// `Receiver` and `Sender` are `Send`, as long as the critical section is `Send` + `Sync`
// (which is the case as we use `CriticalSectionRawMutex`), and the `ProxyRadioRequest` and
// `ProxyRadioResponse` are `Send` (which is the case).
//
// The signals are obviously `Send` + `Sync`.
unsafe impl Send for PhyRadioRunner<'_> {}

const PSDU_LEN: usize = OT_RADIO_FRAME_MAX_SIZE as _;

/// The state of the proxy radio
///
/// This state is borrowed and shared between
/// the two ends of the pipe: the proxy radio, and the PHY radio runner.
struct ProxyRadioState<'a> {
    /// The request channel to the PHY radio
    request: Channel<'a, CriticalSectionRawMutex, ProxyRadioRequest>,
    /// The response channel from the PHY radio
    response: Channel<'a, CriticalSectionRawMutex, ProxyRadioResponse>,
    /// The signal to indicate a new request to the PHY radio
    cancel: Signal<CriticalSectionRawMutex, ()>,
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
    /// - `request_buf`: The request buffer
    /// - `response_buf`: The response buffer
    fn new(
        request_buf: &'a mut [ProxyRadioRequest; 1],
        response_buf: &'a mut [ProxyRadioResponse; 1],
    ) -> Self {
        Self {
            request: Channel::new(request_buf),
            response: Channel::new(response_buf),
            cancel: Signal::new(),
            caps: Signal::new(),
        }
    }

    /// Split the state into the proxy radio and the PHY radio runner.
    fn split(&mut self) -> (ProxyRadio<'_>, PhyRadioRunner<'_>) {
        let (request_sender, request_receiver) = self.request.split();
        let (response_sender, response_receiver) = self.response.split();

        (
            ProxyRadio {
                request: request_sender,
                response: response_receiver,
                cancelled: Cell::new(false),
                cancel: &self.cancel,
                caps: &self.caps,
                config: Config::new(),
            },
            PhyRadioRunner {
                request: request_receiver,
                response: response_sender,
                cancel: &self.cancel,
                caps: &self.caps,
            },
        )
    }
}

/// A proxy radio request.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct ProxyRadioRequest {
    /// Transmit or receive
    tx: bool,
    /// Whether to perform clear channel assessment (CCA) before transmitting the frame
    /// (only relevant for TX requests, should be ignored for RX requests)
    cca: bool,
    /// The radio configuration for the TX/RX operation
    config: Config,
    /// The PSDU to transmit for the TX operation
    psdu: heapless::Vec<u8, PSDU_LEN>,
}

impl ProxyRadioRequest {
    /// Create a new empty proxy radio request.
    const fn new() -> Self {
        Self {
            tx: false,
            cca: false,
            config: Config::new(),
            psdu: heapless::Vec::new(),
        }
    }
}

/// A proxy radio response.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct ProxyRadioResponse {
    /// The result of the TX/RX operation
    result: Result<(), RadioErrorKind>,
    /// The received PSDU, if the operation was successful:
    /// - For TX: the received ACK PSDU (might be empty)
    /// - For RX: the received frame PSDU
    psdu: heapless::Vec<u8, PSDU_LEN>,
    /// The channel on which the frame was received:
    /// - For TX: the channel on which the ACK frame was received
    /// - For RX: the channel on which the regular frame was received
    psdu_channel: u8,
    /// The RSSI of the received frame, if the radio supports appending it at the end of the frame:
    /// - For TX: the RSSI of the received ACK frame
    /// - For RX: the RSSI of the received frame
    psdu_rssi: Option<i8>,
}

impl ProxyRadioResponse {
    /// Create a new empty proxy radio response.
    const fn new() -> Self {
        Self {
            result: Ok(()),
            psdu: heapless::Vec::new(),
            psdu_channel: 0,
            psdu_rssi: None,
        }
    }
}
