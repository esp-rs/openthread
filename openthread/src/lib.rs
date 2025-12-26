//! A safe API for OpenThread (`openthread-sys`)

#![no_std]
#![allow(unknown_lints)]
#![allow(async_fn_in_trait)]
#![allow(clippy::uninlined_format_args)]

use core::cell::{RefCell, RefMut};
use core::ffi::c_void;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::net::{Ipv6Addr, SocketAddrV6};
use core::pin::pin;
use core::ptr::addr_of_mut;

use embassy_futures::select::{Either, Either3};

use embassy_time::Instant;

use fmt::Bytes;
use portable_atomic::Ordering;

use platform::{OT_ACTIVE_STATE, OT_REFCNT};

use signal::Signal;

pub use rand_core::RngCore as OtRngCore;

pub use dataset::*;
pub use fmt::Bytes as BytesFmt;
pub use nat64::*;
pub use netdata::*;
pub use openthread_sys as sys;
pub use radio::*;
pub use scan::*;
pub use settings::*;
#[cfg(feature = "srp")]
pub use srp::*;
#[cfg(feature = "udp")]
pub use udp::*;

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;

mod dataset;
#[cfg(all(feature = "edge-nal", feature = "udp"))]
pub mod enal;
#[cfg(feature = "embassy-net-driver-channel")]
pub mod enet;
#[cfg(feature = "esp-radio")]
pub mod esp;
mod nat64;
mod netdata;
#[cfg(feature = "embassy-nrf")]
pub mod nrf;
mod platform;
mod radio;
mod scan;
mod settings;
mod signal;
#[cfg(feature = "srp")]
mod srp;
#[cfg(feature = "udp")]
mod udp;

use sys::{
    otChangedFlags, otDeviceRole, otDeviceRole_OT_DEVICE_ROLE_CHILD,
    otDeviceRole_OT_DEVICE_ROLE_DETACHED, otDeviceRole_OT_DEVICE_ROLE_DISABLED,
    otDeviceRole_OT_DEVICE_ROLE_LEADER, otDeviceRole_OT_DEVICE_ROLE_ROUTER, otError,
    otError_OT_ERROR_ABORT, otError_OT_ERROR_CHANNEL_ACCESS_FAILURE, otError_OT_ERROR_DROP,
    otError_OT_ERROR_NONE, otError_OT_ERROR_NOT_FOUND, otError_OT_ERROR_NO_ACK,
    otError_OT_ERROR_NO_BUFS, otInstance, otInstanceFinalize, otInstanceInitSingle, otIp6Address,
    otIp6GetUnicastAddresses, otIp6IsEnabled, otIp6NewMessageFromBuffer, otIp6Send,
    otIp6SetEnabled, otIp6SetReceiveCallback, otLinkModeConfig, otMessage, otMessageFree,
    otMessagePriority_OT_MESSAGE_PRIORITY_NORMAL, otMessageRead, otMessageSettings,
    otOperationalDataset, otOperationalDatasetTlvs, otPlatAlarmMilliFired, otPlatRadioReceiveDone,
    otPlatRadioTxDone, otPlatRadioTxStarted, otRadioCaps, otRadioFrame, otSetStateChangedCallback,
    otTaskletsProcess, otThreadGetDeviceRole, otThreadGetExtendedPanId, otThreadSetEnabled,
    otThreadSetLinkMode, OT_RADIO_CAPS_ACK_TIMEOUT, OT_RADIO_CAPS_CSMA_BACKOFF,
    OT_RADIO_CAPS_RX_ON_WHEN_IDLE, OT_RADIO_FRAME_MAX_SIZE,
};

/// A newtype wrapper over the native OpenThread error type (`otError`).
///
/// This type is used to represent errors that can occur when interacting with the OpenThread library.
///
/// Brings extra ergonomics to the error handling, by providing a more Rust-like API.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OtError(otError);

impl OtError {
    /// Create a new `OtError` from a raw `otError` value.
    pub const fn new(value: otError) -> Self {
        Self(value)
    }

    /// Convert to the raw `otError` value.
    pub fn into_inner(self) -> otError {
        self.0
    }
}

impl From<otError> for OtError {
    fn from(value: otError) -> Self {
        Self(value)
    }
}

/// A macro for converting an `otError` value to a `Result<(), OtError>` value.
macro_rules! ot {
    ($code: expr) => {{
        match $code {
            $crate::sys::otError_OT_ERROR_NONE => Ok(()),
            err => Err($crate::OtError::new(err)),
        }
    }};
}

pub(crate) use ot;

/// An extension trait for converting a `Result<(), OtError>` to a raw `otError` OpenThread error code.
pub trait IntoOtCode {
    /// Convert the `Result<(), OtError>` to a raw `otError` OpenThread error code.
    fn into_ot_code(self) -> otError;
}

impl IntoOtCode for Result<(), OtError> {
    fn into_ot_code(self) -> otError {
        match self {
            Ok(_) => otError_OT_ERROR_NONE,
            Err(e) => e.into_inner(),
        }
    }
}

/// A type representing one OpenThread instance.
pub struct OpenThread<'a> {
    state: &'a RefCell<OtState<'a>>,
    #[cfg(feature = "udp")]
    udp_state: Option<&'a RefCell<OtUdpState<'a>>>,
    #[cfg(feature = "srp")]
    srp_state: Option<&'a RefCell<OtSrpState<'a>>>,
}

impl<'a> OpenThread<'a> {
    /// Create a new OpenThread instance.
    ///
    /// Arguments:
    /// - `ieee_eui64`: The IEEE EUI-64 address of the Radio device.
    /// - `rng`: A mutable reference to a random number generator that will be used by OpenThread.
    /// - `resources`: A mutable reference to the OpenThread resources.
    ///
    /// Returns:
    /// - In case there were no errors related to initializing the OpenThread library, the OpenThread instance.
    pub fn new(
        ieee_eui64: [u8; 8],
        rng: &'a mut dyn OtRngCore,
        settings: &'a mut dyn Settings,
        resources: &'a mut OtResources,
    ) -> Result<Self, OtError> {
        if OT_REFCNT
            .compare_exchange(0, 1, Ordering::SeqCst, Ordering::SeqCst)
            .is_err()
        {
            // `OpenThread` is already instantiated; can't instantiate another instance
            // until all `OpenThread` instances are dropped
            Err(OtError::new(otError_OT_ERROR_NO_BUFS))?;
        }

        // Needed so that we convert from the the actual `'a` lifetime of `rng` to the fake `'static` lifetime in `OtResources`
        let state = resources.init(
            ieee_eui64,
            unsafe {
                core::mem::transmute::<&'a mut dyn OtRngCore, &'static mut dyn OtRngCore>(rng)
            },
            unsafe {
                core::mem::transmute::<&'a mut dyn Settings, &'static mut dyn Settings>(settings)
            },
        );

        let state = unsafe {
            core::mem::transmute::<&RefCell<OtState<'static>>, &'a RefCell<OtState<'a>>>(state)
        };

        let mut this = Self {
            state,
            #[cfg(feature = "udp")]
            udp_state: None,
            #[cfg(feature = "srp")]
            srp_state: None,
        };

        this.init()?;

        Ok(this)
    }

    /// Create a new OpenThread instance with support for native OpenThread UDP sockets.
    ///
    /// Arguments:
    /// - `ieee_eui64`: The IEEE EUI-64 address of the Radio device.
    /// - `rng`: A mutable reference to a random number generator that will be used by OpenThread.
    /// - `resources`: A mutable reference to the OpenThread resources.
    /// - `udp_resources`: A mutable reference to the OpenThread UDP resources.
    ///
    /// Returns:
    /// - In case there were no errors related to initializing the OpenThread library, the OpenThread instance.
    #[cfg(feature = "udp")]
    pub fn new_with_udp<const UDP_SOCKETS: usize, const UDP_RX_SZ: usize>(
        ieee_eui64: [u8; 8],
        rng: &'a mut dyn OtRngCore,
        settings: &'a mut dyn Settings,
        resources: &'a mut OtResources,
        udp_resources: &'a mut OtUdpResources<UDP_SOCKETS, UDP_RX_SZ>,
    ) -> Result<Self, OtError> {
        // Needed so that we convert from the the actual `'a` lifetime of `rng` to the fake `'static` lifetime in `OtResources`
        let state = resources.init(
            ieee_eui64,
            unsafe {
                core::mem::transmute::<&'a mut dyn OtRngCore, &'static mut dyn OtRngCore>(rng)
            },
            unsafe {
                core::mem::transmute::<&'a mut dyn Settings, &'static mut dyn Settings>(settings)
            },
        );

        let state = unsafe {
            core::mem::transmute::<&RefCell<OtState<'static>>, &'a RefCell<OtState<'a>>>(state)
        };

        let udp_state = udp_resources.init();
        let udp_state = unsafe {
            core::mem::transmute::<&RefCell<OtUdpState<'static>>, &'a RefCell<OtUdpState<'a>>>(
                udp_state,
            )
        };

        let mut this = Self {
            state,
            udp_state: Some(udp_state),
            #[cfg(feature = "srp")]
            srp_state: None,
        };

        this.init()?;

        Ok(this)
    }

    /// Create a new OpenThread instance with support for native OpenThread SRP services.
    ///
    /// Arguments:
    /// - `ieee_eui64`: The IEEE EUI-64 address of the Radio device.
    /// - `rng`: A mutable reference to a random number generator that will be used by OpenThread.
    /// - `resources`: A mutable reference to the OpenThread resources.
    /// - `srp_resources`: A mutable reference to the OpenThread SRP resources.
    ///
    /// Returns:
    /// - In case there were no errors related to initializing the OpenThread library, the OpenThread instance.
    #[cfg(feature = "srp")]
    pub fn new_with_srp<const SRP_SVCS: usize, const SRP_BUF_SZ: usize>(
        ieee_eui64: [u8; 8],
        rng: &'a mut dyn OtRngCore,
        settings: &'a mut dyn Settings,
        resources: &'a mut OtResources,
        srp_resources: &'a mut OtSrpResources<SRP_SVCS, SRP_BUF_SZ>,
    ) -> Result<Self, OtError> {
        // Needed so that we convert from the the actual `'a` lifetime of `rng` to the fake `'static` lifetime in `OtResources`
        let state = resources.init(
            ieee_eui64,
            unsafe {
                core::mem::transmute::<&'a mut dyn OtRngCore, &'static mut dyn OtRngCore>(rng)
            },
            unsafe {
                core::mem::transmute::<&'a mut dyn Settings, &'static mut dyn Settings>(settings)
            },
        );

        let state = unsafe {
            core::mem::transmute::<&RefCell<OtState<'static>>, &'a RefCell<OtState<'a>>>(state)
        };

        let srp_state = srp_resources.init();
        let srp_state = unsafe {
            core::mem::transmute::<&RefCell<OtSrpState<'static>>, &'a RefCell<OtSrpState<'a>>>(
                srp_state,
            )
        };

        let mut this = Self {
            state,
            #[cfg(feature = "udp")]
            udp_state: None,
            srp_state: Some(srp_state),
        };

        this.init()?;

        Ok(this)
    }

    /// Create a new OpenThread instance with support for native OpenThread UDP sockets and SRP services.
    ///
    /// Arguments:
    /// - `ieee_eui64`: The IEEE EUI-64 address of the Radio device.
    /// - `rng`: A mutable reference to a random number generator that will be used by OpenThread.
    /// - `resources`: A mutable reference to the OpenThread resources.
    /// - `udp_resources`: A mutable reference to the OpenThread UDP resources.
    /// - `srp_resources`: A mutable reference to the OpenThread SRP resources.
    ///
    /// Returns:
    /// - In case there were no errors related to initializing the OpenThread library, the OpenThread instance.
    #[cfg(all(feature = "udp", feature = "srp"))]
    pub fn new_with_udp_srp<
        const UDP_SOCKETS: usize,
        const UDP_RX_SZ: usize,
        const SRP_SVCS: usize,
        const SRP_BUF_SZ: usize,
    >(
        ieee_eui64: [u8; 8],
        rng: &'a mut dyn OtRngCore,
        settings: &'a mut dyn Settings,
        resources: &'a mut OtResources,
        udp_resources: &'a mut OtUdpResources<UDP_SOCKETS, UDP_RX_SZ>,
        srp_resources: &'a mut OtSrpResources<SRP_SVCS, SRP_BUF_SZ>,
    ) -> Result<Self, OtError> {
        // Needed so that we convert from the the actual `'a` lifetime of `rng` to the fake `'static` lifetime in `OtResources`
        // Needed so that we convert from the the actual `'a` lifetime of `rng` to the fake `'static` lifetime in `OtResources`
        let state = resources.init(
            ieee_eui64,
            unsafe {
                core::mem::transmute::<&'a mut dyn OtRngCore, &'static mut dyn OtRngCore>(rng)
            },
            unsafe {
                core::mem::transmute::<&'a mut dyn Settings, &'static mut dyn Settings>(settings)
            },
        );

        let state = unsafe {
            core::mem::transmute::<&RefCell<OtState<'static>>, &'a RefCell<OtState<'a>>>(state)
        };

        let udp_state = udp_resources.init();
        let udp_state = unsafe {
            core::mem::transmute::<&RefCell<OtUdpState<'static>>, &'a RefCell<OtUdpState<'a>>>(
                udp_state,
            )
        };

        let srp_state = srp_resources.init();
        let srp_state = unsafe {
            core::mem::transmute::<&RefCell<OtSrpState<'static>>, &'a RefCell<OtSrpState<'a>>>(
                srp_state,
            )
        };

        let mut this = Self {
            state,
            udp_state: Some(udp_state),
            srp_state: Some(srp_state),
        };

        this.init()?;

        Ok(this)
    }

    /// Return the IEEE EUI-64 address of the Radio device.
    pub fn ieee_eui64(&self) -> [u8; 8] {
        let mut ot = self.activate();
        let state = ot.state();

        state.ot.ieee_eui64
    }

    /// Return the Thread network status.
    pub fn net_status(&self) -> NetStatus {
        let mut ot = self.activate();
        let state = ot.state();

        let device_role = unsafe { otThreadGetDeviceRole(state.ot.instance) }.into();
        let ext_pan_id = unsafe { otThreadGetExtendedPanId(state.ot.instance).as_ref() };

        NetStatus {
            role: device_role,
            ext_pan_id: ext_pan_id.map(|id| u64::from_be_bytes(id.m8)),
            ip6_enabled: unsafe { otIp6IsEnabled(state.ot.instance) },
        }
    }

    /// Brings the OpenThread IPv6 interface up or down.
    pub fn enable_ipv6(&self, enable: bool) -> Result<(), OtError> {
        let mut ot = self.activate();
        let state = ot.state();

        ot!(unsafe { otIp6SetEnabled(state.ot.instance, enable) })
    }

    /// Enable or disable the reception of IPv6 packets.
    ///
    /// If not necessary, reception should be disabled, because this consumes memory.
    pub fn enable_ipv6_rx(&self, enable: bool) {
        let mut ot = self.activate();
        let state = ot.state();

        if enable {
            state.ot.rx_ipv6_enabled = true;
        } else {
            state.ot.rx_ipv6_enabled = false;
            if let Some(msg) = state.ot.rx_ipv6.try_take() {
                unsafe {
                    otMessageFree(msg);
                }
            }
        }
    }

    /// This function starts/stops the Thread protocol operation.
    ///
    /// TODO: The interface must be up when calling this function.
    pub fn enable_thread(&self, enable: bool) -> Result<(), OtError> {
        let mut ot = self.activate();
        let state = ot.state();

        ot!(unsafe { otThreadSetEnabled(state.ot.instance, enable) })
    }

    /// Set the Thread link mode configuration.
    ///
    /// Arguments:
    /// - `rx_on_when_idle`: If true, the device keeps its receiver on when idle.
    ///   This is required for devices that need to receive unsolicited messages.
    /// - `device_type`: If true, the device is a Full Thread Device (FTD).
    /// - `network_data`: If true, the device requests full Network Data.
    pub fn set_link_mode(
        &self,
        rx_on_when_idle: bool,
        device_type: bool,
        network_data: bool,
    ) -> Result<(), OtError> {
        let mut ot = self.activate();
        let state = ot.state();

        // Update radio config rx_when_idle to match link mode.
        // OpenThread doesn't call otPlatRadioSetRxOnWhenIdle, so we must
        // update the radio config directly here to ensure the ESP radio
        // driver keeps the radio awake when needed.
        if state.ot.radio_conf.rx_when_idle != rx_on_when_idle {
            info!(
                "Updating radio config rx_when_idle: {} -> {}",
                state.ot.radio_conf.rx_when_idle, rx_on_when_idle
            );
            state.ot.radio_conf.rx_when_idle = rx_on_when_idle;
        }

        let mode = otLinkModeConfig {
            _bitfield_align_1: [],
            _bitfield_1: otLinkModeConfig::new_bitfield_1(
                rx_on_when_idle,
                device_type,
                network_data,
            ),
        };

        ot!(unsafe { otThreadSetLinkMode(state.ot.instance, mode) })
    }

    /// Gets the list of IPv6 addresses currently assigned to the Thread interface
    ///
    /// Arguments:
    /// - `f`: A closure that will be called for each IPv6 address available.
    ///   Once called for all addresses, the closure will be called with `None`.
    pub fn ipv6_addrs<F>(&self, mut f: F) -> Result<(), OtError>
    where
        F: FnMut(Option<(Ipv6Addr, u8)>) -> Result<(), OtError>,
    {
        let mut ot = self.activate();
        let state = ot.state();

        let mut addrs_ptr = unsafe { otIp6GetUnicastAddresses(state.ot.instance) };

        while !addrs_ptr.is_null() {
            let addrs = unwrap!(unsafe { addrs_ptr.as_ref() });

            f(Some((
                unsafe { addrs.mAddress.mFields.m8 }.into(),
                addrs.mPrefixLength,
            )))?;

            addrs_ptr = addrs.mNext;
        }

        f(None)
    }

    /// Wait for the OpenThread stack to change its state.
    ///
    /// NOTE:
    /// It is not advised to call this method concurrently from multiple async tasks
    /// because it uses a single waker registration. Thus, while the method will not panic,
    /// the tasks will fight with each other by each re-registering its own waker, thus keeping the CPU constantly busy.
    pub async fn wait_changed(&self) {
        poll_fn(move |cx| self.activate().state().ot.changes.poll_wait(cx)).await;
    }

    /// Run the OpenThread stack with the provided radio implementation.
    ///
    /// Arguments:
    /// - `radio`: The radio to be used by the OpenThread stack.
    ///
    /// NOTE:
    /// It is not advised to call this method concurrently from multiple async tasks
    /// because it uses a single waker registration. Thus, while the method will not panic,
    /// the tasks will fight with each other by each re-registering its own waker, thus keeping the CPU constantly busy.
    ///
    /// NOTE:
    /// If the provided radio does not implement some of the MAC capabilities required by OpenThread (`MacCapabilities`)
    /// it is advisable to use `ProxyRadio` and `PhyRadioRunner` to run the radio in a higher priority executor, where
    /// the radio MAC capabilities (which are then emulated in software) can meet their timing deadlines.
    pub async fn run<R>(&self, radio: R) -> !
    where
        R: Radio,
    {
        let mut radio = pin!(self.run_radio(radio, EmbassyTimeTimer));
        let mut alarm = pin!(self.run_alarm());
        let mut openthread = pin!(self.run_tasklets());

        let result =
            embassy_futures::select::select3(&mut radio, &mut alarm, &mut openthread).await;

        match result {
            Either3::First(r) | Either3::Second(r) | Either3::Third(r) => r,
        }
    }

    /// Wait for an IPv6 packet to be available.
    ///
    /// NOTE:
    /// It is not advised to call this method concurrently from multiple async tasks
    /// because it uses a single waker registration. Thus, while the method will not panic,
    /// the tasks will fight with each other by each re-registering its own waker, thus keeping the CPU constantly busy.
    pub async fn wait_rx_available(&self) -> Result<(), OtError> {
        trace!("Waiting for IPv6 packet reception availability");

        poll_fn(move |cx| self.activate().state().ot.rx_ipv6.poll_wait_signaled(cx)).await;

        Ok(())
    }

    /// Receive an IPv6 packet.
    /// If there is no packet available, this function will async-wait until a packet is available.
    ///
    /// Arguments:
    /// - `buf`: A mutable reference to a buffer where the received packet will be stored.
    ///
    /// Returns:
    /// - The length of the received packet.
    ///
    /// NOTE:
    /// It is not advised to call this method concurrently from multiple async tasks
    /// because it uses a single waker registration. Thus, while the method will not panic,
    /// the tasks will fight with each other by each re-registering its own waker, thus keeping the CPU constantly busy.
    pub async fn rx(&self, buf: &mut [u8]) -> Result<usize, OtError> {
        if buf.is_empty() {
            return Ok(0);
        }

        trace!("Waiting for IPv6 packet reception");

        let msg = poll_fn(move |cx| self.activate().state().ot.rx_ipv6.poll_wait(cx)).await;

        let _ot = self.activate();

        let len = unsafe { otMessageRead(msg, 0, buf.as_mut_ptr() as *mut _, buf.len() as _) as _ };

        unsafe {
            otMessageFree(msg);
        }

        trace!("Received IPv6 packet: {}", Bytes(&buf[..len]));

        Ok(len)
    }

    /// Transmit an IPv6 packet.
    ///
    /// Arguments:
    /// - `packet`: The packet to be transmitted.
    pub fn tx(&self, packet: &[u8]) -> Result<(), OtError> {
        self.activate().tx_ip6(packet)?;

        trace!("Transmitted IPv6 packet: {}", Bytes(packet));

        Ok(())
    }

    /// Initialize the OpenThread state, by:
    /// - Initializing the OpenThread C library (returning the OpenThread singleton) TBD: Support more than one OT instance in future
    /// - Setting the state change callback into the OpenThread C library
    /// - Setting the IPv6 receive callback into the OpenThread C library
    ///
    /// NOTE: This method assumes that tbe `OtState` contents is already initialized
    /// (i.e. all signals are in their initial values, and the data which represents OpenThread C types is all zeroed-out)
    fn init(&mut self) -> Result<(), OtError> {
        {
            // TODO: Not ideal but we have to activate even before we have the instance
            // Reason: `otPlatEntropyGet` is called back
            let mut ot = self.activate();
            let state = ot.state();

            // Initialize the OpenThread instance
            state.ot.instance = unsafe { otInstanceInitSingle() };

            info!("OpenThread instance initialized");

            ot!(unsafe {
                otSetStateChangedCallback(
                    state.ot.instance,
                    Some(OtContext::plat_c_change_callback),
                    state.ot.instance as *mut _,
                )
            })?;

            unsafe {
                otIp6SetReceiveCallback(
                    state.ot.instance,
                    Some(OtContext::plat_c_ip6_receive_callback),
                    state.ot.instance as *mut _,
                )
            }

            #[cfg(feature = "srp")]
            unsafe {
                crate::sys::otSrpClientSetCallback(
                    state.ot.instance,
                    Some(OtContext::plat_c_srp_state_change_callback),
                    state.ot.instance as *mut _,
                )
            }
        }

        Ok(())
    }

    /// An async loop that waits until the latest alarm (if any) expires and then notifies the OpenThread C library
    /// Based on `embassy-time` for simplicity and for achieving platform-neutrality.
    async fn run_alarm(&self) -> ! {
        let alarm = || poll_fn(move |cx| self.activate().state().ot.alarm.poll_wait(cx));

        loop {
            trace!("Waiting for trigger alarm request");

            let Some(mut when) = alarm().await else {
                continue;
            };

            trace!(
                "Got trigger alarm request: {}, waiting for it to trigger",
                when
            );

            loop {
                let result =
                    embassy_futures::select::select(alarm(), embassy_time::Timer::at(when)).await;

                match result {
                    Either::First(new_when) => {
                        if let Some(new_when) = new_when {
                            trace!("Alarm interrupted by a new alarm: {}", new_when);
                            when = new_when;
                        } else {
                            trace!("Alarm cancelled");
                            break;
                        }
                    }
                    Either::Second(_) => {
                        trace!("Alarm triggered, notifying OT main loop");

                        {
                            let mut ot = self.activate();
                            let state = ot.state();

                            unsafe { otPlatAlarmMilliFired(state.ot.instance) };
                        }

                        break;
                    }
                }
            }
        }
    }

    /// An async loop that sends or receives IEEE 802.15.4 frames, based on commands issued by the OT loop
    ///
    /// Needs to be a separate async loop, because OpenThread C is unaware of async/await and futures,
    /// however, the Radio driver is async.
    ///
    /// Arguments:
    /// - `radio`: The radio to be used by the OpenThread stack.
    /// - `delay`: The delay implementation to be used by the OpenThread stack.
    async fn run_radio<R, T>(&self, radio: R, timer: T) -> !
    where
        R: Radio,
        T: MacRadioTimer,
    {
        /// Fill the OpenThread frame structure based on the PSDU data returned by the radio
        fn fill_frame(
            frame: &mut otRadioFrame,
            frame_psdu: &mut [u8; OT_RADIO_FRAME_MAX_SIZE as _],
            psdu_meta: PsduMeta,
            psdu: &[u8],
        ) {
            /// Convert from RSSI (Received Signal Strength Indicator) to LQI (Link Quality
            /// Indication)
            ///
            /// RSSI is a measure of incoherent (raw) RF power in a channel. LQI is a
            /// cumulative value used in multi-hop networks to assess the cost of a link.
            fn rssi_to_lqi(rssi: i8) -> u8 {
                if rssi < -80 {
                    0
                } else if rssi > -30 {
                    0xff
                } else {
                    let lqi_convert = ((rssi as u32).wrapping_add(80)) * 255;
                    (lqi_convert / 50) as u8
                }
            }

            let rssi = psdu_meta.rssi.unwrap_or(0);

            frame_psdu[..psdu.len()].copy_from_slice(psdu);
            frame.mLength = psdu.len() as _;
            frame.mRadioType = 1; // TODO: Figure out what is this
            frame.mChannel = psdu_meta.channel;
            frame.mInfo.mRxInfo.mRssi = rssi;
            frame.mInfo.mRxInfo.mLqi = rssi_to_lqi(rssi);
            frame.mInfo.mRxInfo.mTimestamp = Instant::now().as_micros(); // TODO: Not precise
        }

        let mut radio = MacRadio::new(radio, timer);

        let radio_cmd = || poll_fn(move |cx| self.activate().state().ot.radio.poll_wait(cx));

        loop {
            trace!("Waiting for radio command");

            let mut cmd = radio_cmd().await;
            trace!("Got radio command: {:?}", cmd);

            // TODO: Borrow it from the resources
            let mut psdu_buf = [0_u8; OT_RADIO_FRAME_MAX_SIZE as usize];
            let mut ack_psdu_buf = [0_u8; OT_RADIO_FRAME_MAX_SIZE as usize];

            loop {
                unwrap!(radio.set_config(cmd.conf()).await);

                match cmd {
                    RadioCommand::Tx(_) => {
                        let psdu_len = {
                            let mut ot = self.activate();
                            let state = ot.state();

                            let psdu_len = state.ot.radio_resources.snd_frame.mLength as usize;
                            psdu_buf[..psdu_len]
                                .copy_from_slice(&state.ot.radio_resources.snd_psdu[..psdu_len]);

                            unsafe {
                                otPlatRadioTxStarted(
                                    state.ot.instance,
                                    &mut state.ot.radio_resources.snd_frame,
                                );
                            }

                            psdu_len
                        };

                        trace!(
                            "About to Tx 802.15.4 frame {}",
                            Bytes(&psdu_buf[..psdu_len])
                        );

                        let result = {
                            let mut new_cmd = pin!(radio_cmd());
                            let mut tx = pin!(
                                radio.transmit(&psdu_buf[..psdu_len], Some(&mut ack_psdu_buf))
                            );

                            embassy_futures::select::select(&mut new_cmd, &mut tx).await
                        };

                        match result {
                            Either::First(new_cmd) => {
                                let mut ot = self.activate();
                                let state = ot.state();

                                // Reporting send failure because we got interrupted
                                // by a new command
                                unsafe {
                                    otPlatRadioTxDone(
                                        state.ot.instance,
                                        &mut state.ot.radio_resources.snd_frame,
                                        core::ptr::null_mut(),
                                        otError_OT_ERROR_ABORT,
                                    );
                                }

                                trace!("Tx interrupted by new command: {:?}", new_cmd);

                                cmd = new_cmd;
                            }
                            Either::Second(result) => {
                                let mut ot = self.activate();
                                let state = ot.state();
                                let radio_resources = &mut state.ot.radio_resources;

                                match result {
                                    Ok(maybe_ack_psdu_meta) => {
                                        trace!("Tx done, ack frame: {:?}", maybe_ack_psdu_meta);

                                        let ack_frame_ptr =
                                            if let Some(ack_psdu_meta) = maybe_ack_psdu_meta {
                                                let ack_psdu = &ack_psdu_buf[..ack_psdu_meta.len];

                                                fill_frame(
                                                    &mut radio_resources.ack_frame,
                                                    &mut radio_resources.ack_psdu,
                                                    ack_psdu_meta,
                                                    ack_psdu,
                                                );

                                                &mut radio_resources.ack_frame
                                            } else {
                                                core::ptr::null_mut()
                                            };

                                        unsafe {
                                            otPlatRadioTxDone(
                                                state.ot.instance,
                                                &mut state.ot.radio_resources.snd_frame,
                                                ack_frame_ptr,
                                                otError_OT_ERROR_NONE,
                                            );
                                        }
                                    }
                                    Err(err) => {
                                        trace!("Tx failed: {:?}", err);

                                        unsafe {
                                            otPlatRadioTxDone(
                                                state.ot.instance,
                                                &mut state.ot.radio_resources.snd_frame,
                                                core::ptr::null_mut(),
                                                Self::to_ot_err(err),
                                            );
                                        }
                                    }
                                }

                                break;
                            }
                        }
                    }
                    RadioCommand::Rx(_) => {
                        trace!("Waiting for Rx");

                        let result = {
                            let mut new_cmd = pin!(radio_cmd());
                            let mut rx = pin!(radio.receive(&mut psdu_buf));

                            embassy_futures::select::select(&mut new_cmd, &mut rx).await
                        };

                        match result {
                            Either::First(new_cmd) => {
                                trace!("Rx interrupted by new command: {:?}", new_cmd);

                                cmd = new_cmd;
                            }
                            Either::Second(result) => {
                                let mut ot = self.activate();
                                let state = ot.state();

                                match result {
                                    Ok(rcv_psdu_meta) => {
                                        let rcv_psdu = &psdu_buf[..rcv_psdu_meta.len];

                                        trace!(
                                            "Rx done, got frame: {:?}, {}",
                                            rcv_psdu_meta,
                                            Bytes(rcv_psdu)
                                        );

                                        let instance = state.ot.instance;
                                        let radio_resources = &mut state.ot.radio_resources;

                                        fill_frame(
                                            &mut radio_resources.rcv_frame,
                                            &mut radio_resources.rcv_psdu,
                                            rcv_psdu_meta,
                                            rcv_psdu,
                                        );

                                        unsafe {
                                            otPlatRadioReceiveDone(
                                                instance,
                                                &mut radio_resources.rcv_frame,
                                                otError_OT_ERROR_NONE,
                                            );
                                        }
                                    }
                                    Err(err) => {
                                        trace!("Rx failed: {:?}", err);

                                        // Reporting receive failure because we got a driver error
                                        unsafe {
                                            otPlatRadioReceiveDone(
                                                state.ot.instance,
                                                core::ptr::null_mut(),
                                                Self::to_ot_err(err),
                                            );
                                        }
                                    }
                                }

                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    /// Spins the OpenThread C library loop by processing tasklets if they are pending
    /// or otherwise waiting until notified that there are pending tasklets
    async fn run_tasklets(&self) -> ! {
        loop {
            trace!("About to process Openthread tasklets");

            self.activate().process_tasklets();

            poll_fn(move |cx| self.activate().state().ot.tasklets.poll_wait(cx)).await;
        }
    }

    /// Activates the OpenThread stack.
    ///
    /// IMPORTANT: The OpenThread native C API can ONLY be called when this method is called and
    /// the returned `OpenThread` instance is in scope.
    ///
    /// IMPORTANT: Do NOT hold on the `activate`d `OpenThread` instance accross `.await` points!
    ///
    /// Returns:
    /// - An `OpenThread` instance that represents the activated OpenThread stack.
    ///
    /// What activation means in the context of the `openthread` crate is as follows:
    /// - The global `OT_ACTIVE_STATE` variable is set to the current `OtActiveState` instance (which is a borrowed reference to the current `OtState` instance)
    ///   This is necessary so that when an native OpenThread C API "ot*" function is called, OpenThread can call us "back" via the `otPlat*` API
    /// - While the returned `OpenThread` instance is in scope, the data of `OtState` stays mutably borrowed
    fn activate(&self) -> OtContext<'_> {
        OtContext::activate_for(self)
    }

    fn to_ot_err<E>(err: E) -> otError
    where
        E: RadioError,
    {
        match err.kind() {
            RadioErrorKind::TxFailed => otError_OT_ERROR_CHANNEL_ACCESS_FAILURE,
            RadioErrorKind::TxAckFailed
            | RadioErrorKind::RxAckTimeout
            | RadioErrorKind::RxAckInvalid => otError_OT_ERROR_NO_ACK,
            _ => otError_OT_ERROR_ABORT,
        }
    }
}

impl Drop for OpenThread<'_> {
    fn drop(&mut self) {
        if OT_REFCNT.load(Ordering::SeqCst) == 1 {
            // We are the last clone of `OpenThread` so we should finalize the OpenThread instance

            let mut ot = self.activate();
            let state = ot.state();

            unsafe { otInstanceFinalize(state.ot.instance) };
            info!("OpenThread instance finalized");
        }

        // Decrement the reference count
        OT_REFCNT.fetch_sub(1, Ordering::SeqCst);
    }
}

impl Clone for OpenThread<'_> {
    fn clone(&self) -> Self {
        // Increment the reference count
        OT_REFCNT.fetch_add(1, Ordering::SeqCst);

        Self {
            state: self.state,
            #[cfg(feature = "udp")]
            udp_state: self.udp_state,
            #[cfg(feature = "srp")]
            srp_state: self.srp_state,
        }
    }
}

/// The resources (data) that is necessary for the OpenThread stack to operate.
///
/// A separate type so that it can be allocated outside of the OpenThread futures,
/// thus avoiding expensive mem-moves.
///
/// Can also be statically-allocated.
pub struct OtResources {
    /// The radio resources.
    radio_resources: MaybeUninit<RadioResources>,
    /// The dataset resources.
    dataset_resources: MaybeUninit<DatasetResources>,
    /// The OpenThread state.
    ///
    /// This state borrows the radio and dataset resources thus
    /// making this struct self-referencial.
    /// This is not a problem because the `OpenThread` construction API is designed in such a way,
    /// so that this self-referencial borrowing happens only while `OtResources` itself stays mutably
    /// borrowed, while is the case until the `OpenThread` instance is dropped.
    state: MaybeUninit<RefCell<OtState<'static>>>,
}

impl OtResources {
    /// Create a new `OtResources` instance.
    pub const fn new() -> Self {
        Self {
            radio_resources: MaybeUninit::uninit(),
            dataset_resources: MaybeUninit::uninit(),
            state: MaybeUninit::uninit(),
        }
    }

    /// Initialize the resources, as they start their life as `MaybeUninit` so as to avoid mem-moves.
    ///
    /// Returns:
    /// - A reference to a `RefCell<OtState>` value that represents the initialized OpenThread state.
    fn init(
        &mut self,
        ieee_eui64: [u8; 8],
        rng: &'static mut dyn OtRngCore,
        settings: &'static mut dyn Settings,
    ) -> &RefCell<OtState<'static>> {
        let radio_resources = unsafe { self.radio_resources.assume_init_mut() };
        let dataset_resources = unsafe { self.dataset_resources.assume_init_mut() };

        let radio_resources = unsafe {
            core::mem::transmute::<&mut RadioResources, &'static mut RadioResources>(
                radio_resources,
            )
        };
        let dataset_resources = unsafe {
            core::mem::transmute::<&mut DatasetResources, &'static mut DatasetResources>(
                dataset_resources,
            )
        };

        radio_resources.init();

        self.state.write(RefCell::new(OtState {
            ieee_eui64,
            rng,
            settings,
            scan_callback: None,
            scan_done: Signal::new(),
            radio_resources,
            dataset_resources,
            instance: core::ptr::null_mut(),
            rx_ipv6_enabled: false,
            rx_ipv6: Signal::new(),
            alarm: Signal::new(),
            tasklets: Signal::new(),
            changes: Signal::new(),
            radio: Signal::new(),
            radio_conf: Config::new(),
        }));

        info!("OpenThread resources initialized");

        unsafe { self.state.assume_init_mut() }
    }
}

impl Default for OtResources {
    fn default() -> Self {
        Self::new()
    }
}

/// Thread network status.
#[derive(Clone, Debug, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NetStatus {
    /// The device role in the OpenThread network.
    pub role: DeviceRole,
    /// The extended PAN ID of the network, if the device is connected to a network.
    pub ext_pan_id: Option<u64>,
    /// Whether the IPv6 interface is enabled.
    pub ip6_enabled: bool,
}

/// The device role in the OpenThread network.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceRole {
    /// The device is disabled.
    Disabled,
    /// The device is detached.
    Detached,
    /// The device is a child.
    Child,
    /// The device is a router.
    Router,
    /// The device is a leader.
    Leader,
    /// The device is in some other role.
    Other(otDeviceRole),
}

impl DeviceRole {
    /// Return `true` if the device is connected to the network.
    pub const fn is_connected(&self) -> bool {
        matches!(self, Self::Child | Self::Router | Self::Leader)
    }
}

impl From<otDeviceRole> for DeviceRole {
    #[allow(non_upper_case_globals)]
    #[allow(non_snake_case)]
    fn from(value: otDeviceRole) -> Self {
        match value {
            otDeviceRole_OT_DEVICE_ROLE_DISABLED => Self::Disabled,
            otDeviceRole_OT_DEVICE_ROLE_DETACHED => Self::Detached,
            otDeviceRole_OT_DEVICE_ROLE_CHILD => Self::Child,
            otDeviceRole_OT_DEVICE_ROLE_ROUTER => Self::Router,
            otDeviceRole_OT_DEVICE_ROLE_LEADER => Self::Leader,
            other => Self::Other(other),
        }
    }
}

/// Represents an "activated" `OtState` and potentially an activated `OtUdpState`.
///
/// An activated `OtState`/`OtUdpState` is simply the same state but mutably borrowed,
/// for the duration of the activation.
struct OtActiveState<'a> {
    /// The activated `OtState` instance.
    ot: RefMut<'a, OtState<'a>>,
    /// The activated `OtUdpState` instance.
    #[cfg(feature = "udp")]
    udp: Option<RefMut<'a, OtUdpState<'a>>>,
    /// The activated `OtSrpState` instance.
    #[cfg(feature = "srp")]
    srp: Option<RefMut<'a, OtSrpState<'a>>>,
}

#[allow(clippy::needless_lifetimes)]
impl<'a> OtActiveState<'a> {
    /// A utility to get a reference to the UDP state
    ///
    /// This method will return an error if the `OpenThread` instance was not
    /// initialized with UDP resources.
    #[cfg(feature = "udp")]
    pub(crate) fn udp(&mut self) -> Result<&mut OtUdpState<'a>, OtError> {
        let udp = self
            .udp
            .as_mut()
            .ok_or(OtError::new(crate::sys::otError_OT_ERROR_FAILED))?;

        Ok(udp)
    }

    /// A utility to get a reference to the SRP state
    ///
    /// This method will return an error if the `OpenThread` instance was not
    /// initialized with SRP resources.
    #[cfg(feature = "srp")]
    pub(crate) fn srp(&mut self) -> Result<&mut OtSrpState<'a>, OtError> {
        let srp = self
            .srp
            .as_mut()
            .ok_or(OtError::new(crate::sys::otError_OT_ERROR_FAILED))?;

        Ok(srp)
    }
}

// A hack so that we can store `OtActiveState` in the global `OT_ACTIVE_STATE` variable
// While it is not really `Send`-safe, we _do_ know that there a single C OpenThread instance, and it will
// always call us back from the thread on which we called it.
unsafe impl Send for OtActiveState<'_> {}

/// Represents an activated `OpenThread` instance.
/// See `OtContext::activate_for` for more information.
struct OtContext<'a> {
    callback: bool,
    _t: PhantomData<&'a mut ()>,
}

impl<'a> OtContext<'a> {
    /// Activates the OpenThread C wrapper by (temporarily) putting the OpenThread state
    /// in the global `OT_ACTIVE_STATE` variable, which allows the OpenThread C library to call us back.
    ///
    /// Activation is therefore a cheap operation which is expected to be done often, for a short duration
    /// (ideally, just to call one or a few OpenThread C functions) and should not persist across await points
    /// (see below).
    ///
    /// The reason to have the notion of activation in the first place is because there are multiple async agents that
    /// are willing to operate on the same data, i.e.:
    /// - The radio async loop
    /// - The alarm async loop
    /// - The tasklets processing async loop
    /// - The RX, TX and other futures (which might call into OpenThread C by activating it shortly first)
    ///
    /// All of the above tasks operate on the same data (`OtState` / `OtUdpState`) by mutably borrowing it first, either
    /// directly, or by activating (= creating an `OtContext` type instance) and then calling an OpenThread C API.
    ///
    /// Activation is automatically finished when the `OtContext` instance is dropped.
    ///
    /// NOTE: Do NOT hold references to the `OtContext` instance across `.await` points!
    /// NOTE: Do NOT call `activate` twice without dropping the previous instance!
    ///
    /// The above ^^^ will not lead to a memory corruption, but the code will panic due to an attempt
    /// to mutably borrow the `OtState` `RefCell`d data twice.
    fn activate_for(ot: &OpenThread<'a>) -> Self {
        assert!(unwrap!(unsafe { OT_ACTIVE_STATE.0.get().as_mut() }).is_none());

        let active = OtActiveState {
            ot: ot.state.borrow_mut(),
            #[cfg(feature = "udp")]
            udp: ot.udp_state.map(|u| u.borrow_mut()),
            #[cfg(feature = "srp")]
            srp: ot.srp_state.map(|s| s.borrow_mut()),
        };

        // Needed so that we convert from the fake `'static` lifetime in `OT_ACTIVE_STATE` to the actual `'a` lifetime of `ot`
        {
            *unwrap!(unsafe { OT_ACTIVE_STATE.0.get().as_mut() }) = Some(unsafe {
                core::mem::transmute::<OtActiveState<'_>, OtActiveState<'static>>(active)
            });
        }

        Self {
            callback: false,
            _t: PhantomData,
        }
    }

    /// Obtain the already activated `OtContext` instance when arriving
    /// back from C into our code, via some of the `otPlat*` wrappers.
    ///
    /// This method is called when the OpenThread C library calls us back.
    fn callback(_instance: *const otInstance) -> Self {
        assert!(unwrap!(unsafe { OT_ACTIVE_STATE.0.get().as_mut() }).is_some());

        Self {
            callback: true,
            _t: PhantomData,
        }
    }

    /// Gets a reference to the `OtActiveState` instance owned by this `OtContext` instance.
    fn state(&mut self) -> &mut OtActiveState<'a> {
        unsafe {
            core::mem::transmute::<&'static mut OtActiveState<'static>, &mut OtActiveState<'a>>(
                unwrap!(
                    unwrap!(OT_ACTIVE_STATE.0.get().as_mut()).as_mut(),
                    "OpenThread is not activated"
                ),
            )
        }
    }

    /// Ingest an IPv6 packet into OpenThread.
    fn tx_ip6(&mut self, packet: &[u8]) -> Result<(), OtError> {
        let state = self.state();

        let msg = unsafe {
            otIp6NewMessageFromBuffer(
                state.ot.instance,
                packet.as_ptr(),
                packet.len() as _,
                &otMessageSettings {
                    mLinkSecurityEnabled: true,
                    mPriority: otMessagePriority_OT_MESSAGE_PRIORITY_NORMAL as _,
                },
            )
        };

        if !msg.is_null() {
            let res = unsafe { otIp6Send(state.ot.instance, msg) };
            if res != otError_OT_ERROR_DROP {
                ot!(res)?;

                trace!("Transmitted IPv6 packet: {}", Bytes(packet));
            } else {
                // OpenThread will intentionally drop some multicast and ICMPv6 packets
                // which are not required for the Thread network.
                trace!("Ipv6 message dropped");
            }

            Ok(())
        } else {
            Err(OtError::new(otError_OT_ERROR_NO_BUFS))
        }
    }

    /// Process the tasklets if they are pending.
    fn process_tasklets(&mut self) {
        unsafe { otTaskletsProcess(self.state().ot.instance) };
    }

    unsafe extern "C" fn plat_c_change_callback(flags: otChangedFlags, context: *mut c_void) {
        let instance = context as *mut otInstance;

        Self::callback(instance).plat_changed(flags);
    }

    unsafe extern "C" fn plat_c_ip6_receive_callback(msg: *mut otMessage, context: *mut c_void) {
        let instance = context as *mut otInstance;

        Self::callback(instance).plat_ipv6_received(msg);
    }

    #[cfg(feature = "srp")]
    unsafe extern "C" fn plat_c_srp_state_change_callback(
        _error: otError,
        host_info: *const crate::sys::otSrpClientHostInfo,
        services: *const crate::sys::otSrpClientService,
        removed_services: *const crate::sys::otSrpClientService,
        context: *mut c_void,
    ) {
        let instance = context as *mut otInstance;

        Self::callback(instance).plat_srp_changed(
            unsafe { &*host_info },
            unsafe { services.as_ref() },
            unsafe { removed_services.as_ref() },
        );
    }

    #[cfg(feature = "srp")]
    unsafe extern "C" fn plat_c_srp_auto_start_callback(
        _server_sock_addr: *const crate::sys::otSockAddr,
        context: *mut c_void,
    ) {
        let instance = context as *mut otInstance;

        Self::callback(instance).plat_srp_auto_started();
    }

    //
    // All `plat_*` methods below represent the OpenThread C library calling us back.
    // Note that OpenThread C cannot call us back "randomly", as it is not multithreaded and
    // is completely passive.
    //
    // We can get a callback ONLY in the context of _us_ calling an `ot*` OpenThread C API method first.
    // Before the `ot*` method returns, we might get called back via one or more callbacks.
    //

    fn plat_reset(&mut self) -> Result<(), OtError> {
        todo!()
    }

    fn plat_entropy_get(&mut self, buf: &mut [u8]) -> Result<(), OtError> {
        trace!("Entropy requested");
        self.state().ot.rng.fill_bytes(buf);

        Ok(())
    }

    fn plat_tasklets_signal_pending(&mut self) {
        trace!("Tasklets signaled");
        self.state().ot.tasklets.signal(());
    }

    fn plat_ipv6_received(&mut self, msg: *mut otMessage) {
        trace!("Got ipv6 packet");

        let state = self.state();

        if !state.ot.rx_ipv6_enabled || state.ot.rx_ipv6.signaled() {
            if state.ot.rx_ipv6_enabled {
                // Drop the message because the previous one is not consumed yet
                warn!("Dropping RX Ipv6 message, buffer full");
            } else {
                // Drop the message because the RX is disabled
                trace!("Dropping RX Ipv6 message, RX disabled");
            }

            unsafe {
                otMessageFree(msg);
            }
        } else {
            state.ot.rx_ipv6.signal(msg);
        }
    }

    fn plat_changed(&mut self, _flags: u32) {
        trace!("Plat changed callback");
        self.state().ot.changes.signal(());
    }

    fn plat_now(&mut self) -> u32 {
        trace!("Plat now callback");
        Instant::now().as_millis() as u32
    }

    fn plat_alarm_set(&mut self, at0_ms: u32, adt_ms: u32) -> Result<(), OtError> {
        trace!("Plat alarm set callback: {}, {}", at0_ms, adt_ms);

        let instant = embassy_time::Instant::from_millis(at0_ms as _)
            + embassy_time::Duration::from_millis(adt_ms as _);

        self.state().ot.alarm.signal(Some(instant));

        Ok(())
    }

    fn plat_alarm_clear(&mut self) -> Result<(), OtError> {
        trace!("Plat alarm clear callback");
        self.state().ot.alarm.signal(None);

        Ok(())
    }

    fn plat_radio_ieee_eui64(&mut self, mac: &mut [u8; 8]) {
        trace!("Plat radio IEEE EUI64 callback, MAC: {}", Bytes(mac));
        mac.copy_from_slice(self.state().ot.ieee_eui64.as_ref());
    }

    fn plat_radio_caps(&mut self) -> otRadioCaps {
        let caps =
            (OT_RADIO_CAPS_ACK_TIMEOUT | OT_RADIO_CAPS_CSMA_BACKOFF | OT_RADIO_CAPS_RX_ON_WHEN_IDLE)
                as _;
        trace!("Plat radio caps callback, caps: {}", caps);

        caps
    }

    fn plat_radio_is_enabled(&mut self) -> bool {
        let enabled = true; // TODO
        trace!("Plat radio is enabled callback, enabled: {}", enabled);

        enabled
    }

    fn plat_radio_get_rssi(&mut self) -> i8 {
        let rssi = -128; // TODO
        trace!("Plat radio get RSSI callback, RSSI: {}", rssi);

        rssi
    }

    // from https://github.com/espressif/esp-idf/blob/release/v5.3/components/openthread/src/port/esp_openthread_radio.c#L35
    fn plat_radio_receive_sensititivy(&mut self) -> i8 {
        let sens = 0; // TODO
        trace!(
            "Plat radio receive sensitivity callback, sensitivity: {}",
            sens
        );

        sens
    }

    fn plat_radio_get_promiscuous(&mut self) -> bool {
        let promiscuous = false; // TODO
        trace!(
            "Plat radio get promiscuous callback, promiscuous: {}",
            promiscuous
        );

        promiscuous
    }

    fn plat_radio_enable(&mut self) -> Result<(), OtError> {
        info!("Plat radio enable callback");
        Ok(()) // TODO
    }

    fn plat_radio_disable(&mut self) -> Result<(), OtError> {
        info!("Plat radio disable callback");
        Ok(()) // TODO
    }

    fn plat_radio_set_promiscuous(&mut self, promiscuous: bool) {
        info!(
            "Plat radio set promiscuous callback, promiscuous: {}",
            promiscuous
        );

        let state = self.state();

        if state.ot.radio_conf.promiscuous != promiscuous {
            state.ot.radio_conf.promiscuous = promiscuous;
        }
    }

    fn plat_radio_set_rx_on_when_idle(&mut self, enable: bool) {
        info!(
            "Plat radio set rx_on_when_idle callback, enable: {}",
            enable
        );

        let state = self.state();

        if state.ot.radio_conf.rx_when_idle != enable {
            state.ot.radio_conf.rx_when_idle = enable;
        }
    }

    fn plat_radio_set_extended_address(&mut self, address: u64) {
        info!(
            "Plat radio set extended address callback, addr: 0x{:08x}",
            address
        );

        let state = self.state();

        if state.ot.radio_conf.ext_addr != Some(address) {
            state.ot.radio_conf.ext_addr = Some(address);
        }
    }

    fn plat_radio_set_short_address(&mut self, address: u16) {
        info!(
            "Plat radio set short address callback, addr: 0x{:02x}",
            address
        );

        let state = self.state();

        if state.ot.radio_conf.short_addr != Some(address) {
            state.ot.radio_conf.short_addr = Some(address);
        }
    }

    fn plat_radio_set_pan_id(&mut self, pan_id: u16) {
        info!("Plat radio set PAN ID callback, PAN ID: 0x{:02x}", pan_id);

        let state = self.state();

        if state.ot.radio_conf.pan_id != Some(pan_id) {
            state.ot.radio_conf.pan_id = Some(pan_id);
        }
    }

    fn plat_radio_energy_scan(&mut self, channel: u8, duration: u16) -> Result<(), OtError> {
        info!(
            "Plat radio energy scan callback, channel {}, duration {}",
            channel, duration
        );
        unreachable!()
    }

    fn plat_radio_sleep(&mut self) -> Result<(), OtError> {
        info!("Plat radio sleep callback");
        Ok(()) // TODO
    }

    fn plat_radio_transmit_buffer(&mut self) -> *mut otRadioFrame {
        trace!("Plat radio transmit buffer callback");

        // TODO: This frame is private to us, perhaps don't store it in a RefCell?
        &mut self.state().ot.radio_resources.tns_frame
    }

    fn plat_radio_transmit(&mut self, frame: &otRadioFrame) -> Result<(), OtError> {
        trace!(
            "Plat radio transmit callback: {}, {:?}",
            frame.mLength,
            frame.mPsdu
        );

        let state = self.state();

        let psdu = unsafe { core::slice::from_raw_parts_mut(frame.mPsdu, frame.mLength as _) };

        state.ot.radio_resources.snd_frame = *frame;
        state.ot.radio_resources.snd_psdu[..psdu.len()].copy_from_slice(psdu);
        state.ot.radio_resources.snd_frame.mPsdu =
            addr_of_mut!(state.ot.radio_resources.snd_psdu) as *mut _;

        let mut conf = state.ot.radio_conf.clone();
        conf.channel = frame.mChannel;
        state.ot.radio.signal(RadioCommand::Tx(conf));

        Ok(())
    }

    fn plat_radio_receive(&mut self, channel: u8) -> Result<(), OtError> {
        trace!("Plat radio receive callback, channel: {}", channel);

        let state = self.state();

        let mut conf = state.ot.radio_conf.clone();
        conf.channel = channel;
        state.ot.radio.signal(RadioCommand::Rx(conf));

        Ok(())
    }

    fn plat_settings_init(&mut self, sensitive_keys: &[u16]) {
        info!(
            "Plat settings init callback, sensitive keys: {:?}",
            sensitive_keys
        );
        let state = self.state();
        state.ot.settings.init(sensitive_keys);
    }

    fn plat_settings_deinit(&mut self) {
        info!("Plat settings deinit callback");
        let state = self.state();
        state.ot.settings.deinit();
    }

    fn plat_settings_get(
        &mut self,
        key: u16,
        index: core::ffi::c_int,
        buf: &mut [u8],
    ) -> Result<usize, OtError> {
        trace!(
            "Plat settings get callback, key: {}, index: {}, buf len: {}",
            key,
            index,
            buf.len()
        );

        if index < 0 {
            Err(OtError::new(otError_OT_ERROR_NOT_FOUND))?;
        }

        let state = self.state();

        let settings = &mut state.ot.settings;

        let len = settings
            .get(key, index as _, buf)?
            .ok_or(OtError::new(otError_OT_ERROR_NOT_FOUND))?;

        Ok(len)
    }

    fn plat_settings_set(&mut self, key: u16, value: &[u8]) -> Result<(), OtError> {
        trace!(
            "Plat settings set callback, key: {}, value: {}",
            key,
            Bytes(value)
        );

        let state = self.state();

        let settings = &mut state.ot.settings;

        settings.set(key, value)?;

        Ok(())
    }

    fn plat_settings_add(&mut self, key: u16, value: &[u8]) -> Result<(), OtError> {
        trace!(
            "Plat settings add callback, key: {}, value: {}",
            key,
            Bytes(value)
        );

        let state = self.state();

        let settings = &mut state.ot.settings;

        settings.add(key, value)?;

        Ok(())
    }

    fn plat_settings_delete(&mut self, key: u16, index: core::ffi::c_int) -> Result<(), OtError> {
        trace!(
            "Plat settings delete callback, key: {}, index: {}",
            key,
            index
        );

        if index < 0 {
            Err(OtError::new(otError_OT_ERROR_NOT_FOUND))?;
        }

        let state = self.state();

        let settings = &mut state.ot.settings;

        if !settings.remove(key, Some(index as _))? {
            Err(OtError::new(otError_OT_ERROR_NOT_FOUND))?;
        }

        Ok(())
    }

    fn plat_settings_wipe(&mut self) {
        info!("Plat settings wipe callback");

        unwrap!(self.state().ot.settings.clear());
    }
}

impl Drop for OtContext<'_> {
    fn drop(&mut self) {
        assert!(unwrap!(unsafe { OT_ACTIVE_STATE.0.get().as_mut() }).is_some());

        if !self.callback {
            unwrap!(unsafe { OT_ACTIVE_STATE.0.get().as_mut() }).take();
        }
    }
}

/// The OpenThread state from Rust POV.
///
/// This data lives behind a `RefCell` and is mutably borrowed each time
/// the OpenThread stack is activated, by creating an `OtContext` instance.
struct OtState<'a> {
    /// The OpenThread instance associated with the `OtData` instance.
    instance: *mut otInstance,
    /// The IEEE EUI-64 address of the Radio device.
    ieee_eui64: [u8; 8],
    /// The random number generator associated with the `OtData` instance.
    rng: &'a mut dyn OtRngCore,
    /// The OT settings
    settings: &'a mut dyn Settings,
    /// The callback to invoke when network scanning is in progress
    #[allow(clippy::type_complexity)]
    scan_callback: Option<&'a mut dyn FnMut(Option<&ScanResult>)>,
    /// Indicate that scanning has completed
    scan_done: Signal<()>,
    /// Whether to egress IPv6 packets from OpenThread
    /// If not necessary, this should be disabled, because otherwise the signal below
    /// will be filled with a packet that is not consumed, and the packets of OpenThread
    /// take precious RAM.
    rx_ipv6_enabled: bool,
    /// An Ipv6 packet egressed from OpenThread and waiting to be ingressed somewhere else
    rx_ipv6: Signal<*mut otMessage>,
    /// `Some` in case there is a pending OpenThread awarm which is not due yet
    /// `None` if the existing alarm needs to be cancelled
    alarm: Signal<Option<embassy_time::Instant>>,
    /// The tasklets need to be run. Set by the OpenThread C library via the `otPlatTaskletsSignalPending` callback
    tasklets: Signal<()>,
    /// The OpenThread state has changed. Set by the OpenThread C library via the `otPlatStateChanged` callback
    changes: Signal<()>,
    /// The radio needs to execute the provided command
    radio: Signal<RadioCommand>,
    /// The latest radio configuration from the POV of OpenThread
    radio_conf: radio::Config,
    /// Resources for the radio (PHY data frames and their descriptors)
    radio_resources: &'a mut RadioResources,
    /// Resources for dealing with the operational dataset
    dataset_resources: &'a mut DatasetResources,
}

/// A command for the radio runner to process.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum RadioCommand {
    /// Transmit a frame with the provided configuration
    /// The data of the frame is in `OtData::radio_resources.snd_frame` and `OtData::radio_resources.snd_psdu`
    ///
    /// Once the frame is sent (or an error occurs) OpenThread C will be signalled by calling `otPlatRadioTxDone`
    Tx(Config),
    /// Receive a frame with the provided configuration
    ///
    /// Once the frame is received, it will be copied to `OtData::radio_resources.rcv_frame` and `OtData::radio_resources.rcv_psdu`
    /// and OpenThread C will be signalled by calling `otPlatRadioReceiveDone`
    Rx(Config),
}

impl RadioCommand {
    const fn conf(&self) -> &Config {
        match self {
            Self::Tx(conf) | Self::Rx(conf) => conf,
        }
    }
}

/// Radio-related OpenThread C data carriers
///
/// Note that this structure is self-referential in that its `*_frame` members all
/// contain a pointer to its corresponding `*_psdu` member.
///
/// This is not modelled strictly (i.e. with pinning), because all of these structures are internal and not an API
/// the user can abuse. With that said, care should be taken the self-referencial struct to be properly initialized,
/// before using it and members of the struct should not be swapped-out after that.
///
/// With that said, the structure anyway cannot be moved once we hit the `OpenThread::new*` functions in this crate,
/// because of the signature of the `OpenThread::new*` APIs which **mutably borrow** `OtResources` (and thus this structure too)
/// for the lifetime of the `OpenThread` Rust stack.
struct RadioResources {
    /// The received frame from the radio
    rcv_frame: otRadioFrame,
    /// An empty ACK frame send to `otPlatRadioReceiveDone` TBD why we need that
    ack_frame: otRadioFrame,
    /// A buffer where OpenThread prepares the next frame to be send
    tns_frame: otRadioFrame,
    /// A frame which is to be send to the radio
    snd_frame: otRadioFrame,
    /// The PSDU of the received frame
    rcv_psdu: [u8; OT_RADIO_FRAME_MAX_SIZE as usize],
    /// The PSDU of the frame to be send by the OpenThread C code
    /// OpenThread C code keeps hold of this buffer accross callbacks to us
    tns_psdu: [u8; OT_RADIO_FRAME_MAX_SIZE as usize],
    /// The PSDU of the frame to be send by the radio
    snd_psdu: [u8; OT_RADIO_FRAME_MAX_SIZE as usize],
    /// The PSDU of the ACK frame send to `otPlatRadioReceiveDone`
    ack_psdu: [u8; OT_RADIO_FRAME_MAX_SIZE as usize],
}

impl RadioResources {
    /// Create a new `RadioResources` instance.
    #[allow(unused)]
    const fn new() -> Self {
        unsafe {
            Self {
                rcv_frame: MaybeUninit::zeroed().assume_init(),
                tns_frame: MaybeUninit::zeroed().assume_init(),
                snd_frame: MaybeUninit::zeroed().assume_init(),
                ack_frame: MaybeUninit::zeroed().assume_init(),
                rcv_psdu: MaybeUninit::zeroed().assume_init(),
                tns_psdu: MaybeUninit::zeroed().assume_init(),
                snd_psdu: MaybeUninit::zeroed().assume_init(),
                ack_psdu: MaybeUninit::zeroed().assume_init(),
            }
        }
    }

    /// Initialize the `RadioResources` instance by doing the self-referential magic.
    ///
    /// For this to work, `init` should be called from inside the `Openthread::new*` API methods that create
    /// all of the public-facing APIs, as at that time `OtResources` is already mutably borrowed and cannot move.
    ///
    /// This method should not be called e.g. from the constructor of `OtResources`, as the value can move once
    /// constructed and before being mutable borrowed into the `openthread::new` API method from above.
    fn init(&mut self) {
        self.rcv_frame.mPsdu = addr_of_mut!(self.rcv_psdu) as *mut _;
        self.tns_frame.mPsdu = addr_of_mut!(self.tns_psdu) as *mut _;
        self.snd_frame.mPsdu = addr_of_mut!(self.snd_psdu) as *mut _;
        self.ack_frame.mPsdu = addr_of_mut!(self.ack_psdu) as *mut _;
    }
}

/// Dataset-related OpenThread C data carriers
struct DatasetResources {
    /// The operational dataset
    dataset: otOperationalDataset,
    dataset_tlv: otOperationalDatasetTlvs,
}

impl DatasetResources {
    /// Create a new `DatasetResources` instance.
    #[allow(unused)]
    const fn new() -> Self {
        unsafe {
            Self {
                dataset: MaybeUninit::zeroed().assume_init(),
                dataset_tlv: MaybeUninit::zeroed().assume_init(),
            }
        }
    }
}

/// Convert an `otIp6Address`, port and network interface ID to a `SocketAddrV6`.
#[allow(unused)]
fn to_sock_addr(addr: &otIp6Address, port: u16, netif: u32) -> SocketAddrV6 {
    SocketAddrV6::new(Ipv6Addr::from(unsafe { addr.mFields.m8 }), port, 0, netif)
}

/// Convert a `SocketAddrV6` to an `otSockAddr`.
#[cfg(any(feature = "udp", feature = "srp"))]
fn to_ot_addr(addr: &SocketAddrV6) -> crate::sys::otSockAddr {
    crate::sys::otSockAddr {
        mAddress: otIp6Address {
            mFields: sys::otIp6Address__bindgen_ty_1 {
                m8: addr.ip().octets(),
            },
        },
        mPort: addr.port(),
    }
}
