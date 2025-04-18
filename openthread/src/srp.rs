use core::cell::RefCell;
use core::ffi::CStr;
use core::fmt::{self, Display};
use core::future::poll_fn;
use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::net::{Ipv6Addr, SocketAddrV6};

use crate::signal::Signal;
use crate::sys::{
    otDnsTxtEntry, otError_OT_ERROR_INVALID_ARGS, otError_OT_ERROR_INVALID_STATE,
    otError_OT_ERROR_NO_BUFS, otIp6Address, otSrpClientAddService, otSrpClientClearHostAndServices,
    otSrpClientClearService, otSrpClientEnableAutoHostAddress, otSrpClientEnableAutoStartMode,
    otSrpClientGetHostInfo, otSrpClientGetKeyLeaseInterval, otSrpClientGetLeaseInterval,
    otSrpClientGetServerAddress, otSrpClientGetServices, otSrpClientGetTtl, otSrpClientHostInfo,
    otSrpClientIsAutoStartModeEnabled, otSrpClientIsRunning, otSrpClientItemState,
    otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_ADDING,
    otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REFRESHING,
    otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REGISTERED,
    otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REMOVED,
    otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REMOVING,
    otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_TO_ADD,
    otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_TO_REFRESH,
    otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_TO_REMOVE, otSrpClientRemoveHostAndServices,
    otSrpClientRemoveService, otSrpClientService, otSrpClientSetHostAddresses,
    otSrpClientSetHostName, otSrpClientSetKeyLeaseInterval, otSrpClientSetLeaseInterval,
    otSrpClientSetTtl, otSrpClientStart, otSrpClientStop,
};
use crate::{ot, to_ot_addr, to_sock_addr, OpenThread, OtContext, OtError};

/// The unique ID of a registered SRP service
pub type SrpServiceSlot = usize;

/// The resources (data) that is necessary for the OpenThread stack to operate with SRP services.
///
/// A separate type so that it can be allocated outside of the OpenThread futures,
/// thus avoiding expensive mem-moves.
///
/// Can also be statically-allocated.
pub struct OtSrpResources<const SRP_SVCS: usize = 3, const SRP_BUF_SZ: usize = 300> {
    /// Memory for up to `SRP_SVCS` SRP services
    services: MaybeUninit<[otSrpClientService; SRP_SVCS]>,
    /// Whether a service slot in the above memory is taken
    taken: MaybeUninit<[bool; SRP_SVCS]>,
    /// Memory for the SRP configuration (host name and IP addresses)
    conf: MaybeUninit<[u8; SRP_BUF_SZ]>,
    /// Memory for the SRP service data
    buffers: MaybeUninit<[[u8; SRP_BUF_SZ]; SRP_SVCS]>,
    /// The state of the SRP services, from Rust POV
    state: MaybeUninit<RefCell<OtSrpState<'static>>>,
}

impl<const SRP_SVCS: usize, const SRP_BUF_SZ: usize> OtSrpResources<SRP_SVCS, SRP_BUF_SZ> {
    #[allow(clippy::declare_interior_mutable_const)]
    const INIT_SERVICE: otSrpClientService = otSrpClientService {
        mName: core::ptr::null(),
        mInstanceName: core::ptr::null(),
        mSubTypeLabels: core::ptr::null(),
        mTxtEntries: core::ptr::null(),
        mPort: 0,
        mPriority: 0,
        mWeight: 0,
        mNumTxtEntries: 0,
        mState: 0,
        mData: 0,
        mNext: core::ptr::null_mut(),
        mLease: 0,
        mKeyLease: 0,
    };
    #[allow(clippy::declare_interior_mutable_const)]
    const INIT_BUFFERS: [u8; SRP_BUF_SZ] = [0; SRP_BUF_SZ];

    /// Create a new `OtSrpResources` instance.
    pub const fn new() -> Self {
        Self {
            services: MaybeUninit::uninit(),
            taken: MaybeUninit::uninit(),
            conf: MaybeUninit::uninit(),
            buffers: MaybeUninit::uninit(),
            state: MaybeUninit::uninit(),
        }
    }

    /// Initialize the resources, as they start their life as `MaybeUninit` so as to avoid mem-moves.
    ///
    /// Returns:
    /// - A reference to a `RefCell<OtSrpState>` value that represents the initialized OpenThread SRP state.
    pub(crate) fn init(&mut self) -> &RefCell<OtSrpState<'static>> {
        self.services.write([Self::INIT_SERVICE; SRP_SVCS]);
        self.taken.write([false; SRP_SVCS]);
        self.conf.write(Self::INIT_BUFFERS);
        self.buffers.write([Self::INIT_BUFFERS; SRP_SVCS]);

        let services = unsafe { self.services.assume_init_mut() };
        let services = unsafe {
            core::mem::transmute::<
                &mut [otSrpClientService; SRP_SVCS],
                &'static mut [otSrpClientService; SRP_SVCS],
            >(services)
        };

        let taken = unsafe { self.taken.assume_init_mut() };
        let taken = unsafe {
            core::mem::transmute::<&mut [bool; SRP_SVCS], &'static mut [bool; SRP_SVCS]>(taken)
        };

        let conf = unsafe { self.conf.assume_init_mut() };
        let conf = unsafe {
            core::mem::transmute::<&mut [u8; SRP_BUF_SZ], &'static mut [u8; SRP_BUF_SZ]>(conf)
        };

        let buffers: &mut [[u8; SRP_BUF_SZ]; SRP_SVCS] = unsafe { self.buffers.assume_init_mut() };
        let buffers: &'static mut [u8] = unsafe {
            core::slice::from_raw_parts_mut(buffers.as_mut_ptr() as *mut _, SRP_BUF_SZ * SRP_SVCS)
        };

        self.state.write(RefCell::new(OtSrpState {
            services,
            taken,
            conf,
            conf_taken: false,
            buffers,
            buf_len: SRP_BUF_SZ,
            changes: Signal::new(),
        }));

        info!("OpenThread SRP resources initialized");

        unsafe { self.state.assume_init_mut() }
    }
}

impl<const SRP_SVCS: usize, const SRP_BUF_SZ: usize> Default
    for OtSrpResources<SRP_SVCS, SRP_BUF_SZ>
{
    fn default() -> Self {
        Self::new()
    }
}

/// The SRP state of the OpenThread stack, from Rust POV.
///
/// This data lives behind a `RefCell` and is mutably borrowed each time
/// the OpenThread stack is activated, by creating an `OtContext` instance.
pub(crate) struct OtSrpState<'a> {
    /// Slots for SRP services
    services: &'a mut [otSrpClientService],
    /// Whether a service slot is taken
    taken: &'a mut [bool],
    /// Memory for the SRP configuration (host name and IP addresses)
    conf: &'a mut [u8],
    /// Whether the host is currently configured (i.e., the `conf` field is used)
    conf_taken: bool,
    /// Memory for the data of all SRP services
    buffers: &'a mut [u8],
    /// The length of the data buffer for one SRP service
    buf_len: usize,
    /// Changed signal
    pub(crate) changes: Signal<()>,
}

/// An enum describing the status of either a concrete SRP service, or the SRP host.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum SrpState {
    /// The service/host is to be added/registered.
    ToAdd,
    /// The service/host is being added/registered.
    Adding,
    /// The service/host is to be refreshed (re-register to renew lease).
    ToRefresh,
    /// The service/host is being refreshed.
    Refreshing,
    /// The service/host is to be removed/unregistered.
    ToRemove,
    /// The service/host is being removed/unregistered.
    Removing,
    /// The service/host has been removed/unregistered.
    Removed,
    /// The service/host is registered.
    Registered,
    /// Any other state.
    Other(otSrpClientItemState),
}

impl Display for SrpState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ToAdd => write!(f, "To add"),
            Self::Adding => write!(f, "Adding"),
            Self::ToRefresh => write!(f, "To refresh"),
            Self::Refreshing => write!(f, "Refreshing"),
            Self::ToRemove => write!(f, "To remove"),
            Self::Removing => write!(f, "Removing"),
            Self::Removed => write!(f, "Removed"),
            Self::Registered => write!(f, "Registered"),
            Self::Other(state) => write!(f, "Other ({})", state),
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for SrpState {
    fn format(&self, f: defmt::Formatter<'_>) {
        match self {
            Self::ToAdd => defmt::write!(f, "To add"),
            Self::Adding => defmt::write!(f, "Adding"),
            Self::ToRefresh => defmt::write!(f, "To refresh"),
            Self::Refreshing => defmt::write!(f, "Refreshing"),
            Self::ToRemove => defmt::write!(f, "To remove"),
            Self::Removing => defmt::write!(f, "Removing"),
            Self::Removed => defmt::write!(f, "Removed"),
            Self::Registered => defmt::write!(f, "Registered"),
            Self::Other(state) => defmt::write!(f, "Other ({})", state),
        }
    }
}

#[allow(non_upper_case_globals)]
#[allow(non_snake_case)]
impl From<otSrpClientItemState> for SrpState {
    fn from(value: otSrpClientItemState) -> Self {
        match value {
            otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_TO_ADD => Self::ToAdd,
            otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_ADDING => Self::Adding,
            otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_TO_REFRESH => Self::ToRefresh,
            otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REFRESHING => Self::Refreshing,
            otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_TO_REMOVE => Self::ToRemove,
            otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REMOVING => Self::Removing,
            otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REMOVED => Self::Removed,
            otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REGISTERED => Self::Registered,
            other => Self::Other(other),
        }
    }
}

/// The SRP configuration of the OpenThread stack.
#[derive(Debug, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SrpConf<'a> {
    /// SRP hostname
    pub host_name: &'a str,
    /// SRP host Ipv6 addresses.
    /// If empty, the SRP implementation will automatically set the host addresses
    /// by itself, using non-link-local addresses, once these become available.
    pub host_addrs: &'a [Ipv6Addr],
    /// SRP TTL (Time To Live) value.
    pub ttl: u32,
    /// Default lease time for SRP services if they specify 0 for their lease time.
    /// Set to 0 to use the OpenThread default value.
    pub default_lease_secs: u32,
    /// Default key lease time for SRP services' keys if they specify 0 for their key lease time.
    /// Set to 0 to use the OpenThread default value.
    pub default_key_lease_secs: u32,
}

impl SrpConf<'_> {
    /// Create a new `SrpConf` instance, wuth a host named "ot-device",
    /// no explicit host addresses, a TTL of 60 seconds, and default lease times.
    pub const fn new() -> Self {
        Self {
            host_name: "ot-device",
            host_addrs: &[],
            ttl: 60,
            default_lease_secs: 0,
            default_key_lease_secs: 0,
        }
    }

    fn store(&self, ot_srp: &mut otSrpClientHostInfo, buf: &mut [u8]) -> Result<(), OtError> {
        let (addrs, buf) = align_min::<otIp6Address>(buf, self.host_addrs.len())?;

        ot_srp.mName = store_str(self.host_name, buf)?.0.as_ptr();

        for (index, ip) in self.host_addrs.iter().enumerate() {
            let addr = &mut addrs[index];
            addr.mFields.m8 = ip.octets();
        }

        ot_srp.mAddresses = if addrs.is_empty() {
            core::ptr::null_mut()
        } else {
            addrs.as_ptr()
        };
        ot_srp.mNumAddresses = addrs.len() as _;
        ot_srp.mAutoAddress = addrs.is_empty();

        Ok(())
    }
}

impl Default for SrpConf<'_> {
    fn default() -> Self {
        Self::new()
    }
}

/// An SRP service that can be registered with the OpenThread stack.
#[derive(Debug, Clone, Eq, PartialEq, Hash)]
pub struct SrpService<'a, SI, TI> {
    /// The service name.
    pub name: &'a str,
    /// The instance name.
    pub instance_name: &'a str,
    /// The subtype labels.
    pub subtype_labels: SI,
    /// The TXT entries.
    pub txt_entries: TI,
    /// The service port.
    pub port: u16,
    /// The service priority.
    pub priority: u16,
    /// The service weight.
    pub weight: u16,
    /// The service lease time in seconds.
    /// Set to 0 to use the default value as specified in `SrpConf`.
    pub lease_secs: u32,
    /// The service key lease time in seconds.
    /// Set to 0 to use the default value as specified in `SrpConf`.
    pub key_lease_secs: u32,
}

impl<'a, SI, TI> SrpService<'a, SI, TI>
where
    SI: Iterator<Item = &'a str> + Clone + 'a,
    TI: Iterator<Item = (&'a str, &'a [u8])> + Clone + 'a,
{
    fn store(&self, ot_srp: &mut otSrpClientService, buf: &mut [u8]) -> Result<(), OtError> {
        let subtype_labels_len = self.subtype_labels.clone().count();
        let txt_entries_len = self.txt_entries.clone().count();

        let (txt_entries, buf) = align_min::<otDnsTxtEntry>(buf, txt_entries_len)?;
        let (subtype_labels, buf) = align_min::<*const char>(buf, subtype_labels_len + 1)?;

        let (name, buf) = store_str(self.name, buf)?;
        let (instance_name, buf) = store_str(self.instance_name, buf)?;

        ot_srp.mName = name.as_ptr();
        ot_srp.mInstanceName = instance_name.as_ptr();

        let mut index = 0;
        let mut buf = buf;

        for subtype_label in self.subtype_labels.clone() {
            let (subtype_label, rem_buf) = store_str(subtype_label, buf)?;

            subtype_labels[index] = subtype_label.as_ptr() as *const _;

            buf = rem_buf;
            index += 1;
        }

        subtype_labels[index] = core::ptr::null();

        index = 0;

        for (key, value) in self.txt_entries.clone() {
            let txt_entry = &mut txt_entries[index];

            let (key, rem_buf) = store_str(key, buf)?;
            let (value, rem_buf) = store_data(value, rem_buf)?;

            txt_entry.mKey = key.as_ptr();
            txt_entry.mValue = value.as_ptr();
            txt_entry.mValueLength = value.len() as _;

            buf = rem_buf;
            index += 1;
        }

        ot_srp.mSubTypeLabels = subtype_labels.as_ptr() as *const _;
        ot_srp.mTxtEntries = txt_entries.as_ptr();
        ot_srp.mNumTxtEntries = txt_entries_len as _;
        ot_srp.mPort = self.port;
        ot_srp.mPriority = self.priority;
        ot_srp.mWeight = self.weight;
        ot_srp.mLease = self.lease_secs;
        ot_srp.mKeyLease = self.key_lease_secs;
        ot_srp.mState = 0;
        ot_srp.mNext = core::ptr::null_mut();

        Ok(())
    }
}

impl<'a, SI, TI> Display for SrpService<'a, SI, TI>
where
    SI: Iterator<Item = &'a str> + Clone,
    TI: Iterator<Item = (&'a str, &'a [u8])> + Clone,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "SRP Service {{name: {}, instance: {}, port: {}, priority: {}, weight: {}, lease: {}, keylease: {}, labels: [",
            self.name,
            self.instance_name,
            self.port,
            self.priority,
            self.weight,
            self.lease_secs,
            self.key_lease_secs
        )?;

        for (index, label) in self.subtype_labels.clone().enumerate() {
            if index > 0 {
                write!(f, ", {}", label)?;
            } else {
                write!(f, "{}", label)?;
            }
        }

        write!(f, "], txt: [")?;

        for (index, value) in self.txt_entries.clone().enumerate() {
            if index > 0 {
                write!(f, ", {}: {:?}", value.0, value.1)?;
            } else {
                write!(f, "{}: {:?}", value.0, value.1)?;
            }
        }

        write!(f, "]}}")
    }
}

#[cfg(feature = "defmt")]
impl<'a, SI, TI> defmt::Format for SrpService<'a, SI, TI>
where
    SI: Iterator<Item = &'a str> + Clone,
    TI: Iterator<Item = (&'a str, &'a [u8])> + Clone,
{
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "SRP Service {{name: {}, instance: {}, port: {}, priority: {}, weight: {}, lease: {}, keylease: {}, labels: [",
            self.name,
            self.instance_name,
            self.port,
            self.priority,
            self.weight,
            self.lease_secs,
            self.key_lease_secs
        );

        for (index, label) in self.subtype_labels.clone().enumerate() {
            if index > 0 {
                defmt::write!(f, ", {}", label);
            } else {
                defmt::write!(f, "{}", label);
            }
        }

        defmt::write!(f, "], txt: [");

        for (index, value) in self.txt_entries.clone().enumerate() {
            if index > 0 {
                defmt::write!(f, ", {}: {:?}", value.0, value.1);
            } else {
                defmt::write!(f, "{}: {:?}", value.0, value.1);
            }
        }

        defmt::write!(f, "]}}")
    }
}

/// Type alias for an SRP service as returned by
/// `OpenThread::srp_services`.
pub type OutSrpService<'a> = SrpService<'a, OutSrpSubtypeLabelsIter<'a>, OutSrpTxtEntriesIter<'a>>;

/// An iterator over the subtype labels of an SRP service
/// as returned by `OpenThread::srp_services`.
#[derive(Clone)]
pub struct OutSrpSubtypeLabelsIter<'a> {
    ptr: *const *const u8,
    index: usize,
    _phantom: PhantomData<&'a ()>,
}

impl<'a> Iterator for OutSrpSubtypeLabelsIter<'a> {
    type Item = &'a str;

    fn next(&mut self) -> Option<Self::Item> {
        if self.ptr.is_null() {
            return None;
        }

        let label = unsafe { *self.ptr.add(self.index) };

        if label.is_null() {
            None
        } else {
            self.index += 1;
            Some(unwrap!(
                unsafe { CStr::from_ptr(label as _) }.to_str(),
                "Invalid UTF-8 in SRP subtype label"
            ))
        }
    }
}

/// An iterator over the TXT entries of an SRP service
/// as returned by `OpenThread::srp_services`.
#[derive(Clone)]
pub struct OutSrpTxtEntriesIter<'a> {
    ptr: *const otDnsTxtEntry,
    size: usize,
    index: usize,
    _phantom: PhantomData<&'a ()>,
}

impl<'a> Iterator for OutSrpTxtEntriesIter<'a> {
    type Item = (&'a str, &'a [u8]);

    fn next(&mut self) -> Option<Self::Item> {
        if self.ptr.is_null() || self.index == self.size {
            return None;
        }

        let entry = unsafe { self.ptr.add(self.index) };

        self.index += 1;

        let entry = unsafe { &*entry };

        Some((
            unwrap!(
                unsafe { CStr::from_ptr(entry.mKey) }.to_str(),
                "Invalid UTF-8 in SRP TXT entry key"
            ),
            unsafe { core::slice::from_raw_parts(entry.mValue, entry.mValueLength as _) },
        ))
    }
}

impl<'a> From<&'a otSrpClientService> for OutSrpService<'a> {
    fn from(ot_srp: &'a otSrpClientService) -> Self {
        Self {
            name: if !ot_srp.mName.is_null() {
                unwrap!(
                    unsafe { CStr::from_ptr(ot_srp.mName) }.to_str(),
                    "Invalid UTF-8 in SRP service name"
                )
            } else {
                ""
            },
            instance_name: if !ot_srp.mInstanceName.is_null() {
                unwrap!(
                    unsafe { CStr::from_ptr(ot_srp.mInstanceName) }.to_str(),
                    "Invalid UTF-8 in SRP instance name"
                )
            } else {
                ""
            },
            subtype_labels: OutSrpSubtypeLabelsIter {
                ptr: ot_srp.mSubTypeLabels as _,
                index: 0,
                _phantom: PhantomData,
            },
            txt_entries: OutSrpTxtEntriesIter {
                ptr: ot_srp.mTxtEntries,
                size: ot_srp.mNumTxtEntries as _,
                index: 0,
                _phantom: PhantomData,
            },
            port: ot_srp.mPort,
            priority: ot_srp.mPriority,
            weight: ot_srp.mWeight,
            lease_secs: ot_srp.mLease,
            key_lease_secs: ot_srp.mKeyLease,
        }
    }
}

impl OpenThread<'_> {
    /// Return the current SRP client configuration and SRP client host state to the provided closure.
    ///
    /// Arguments:
    /// - `f`: A closure that takes the SRP configuration and SRP host state as arguments.
    pub fn srp_conf<F, R>(&self, f: F) -> Result<R, OtError>
    where
        F: FnOnce(&SrpConf, SrpState, bool) -> Result<R, OtError>,
    {
        let mut ot = self.activate();
        let instance = ot.state().ot.instance;
        let srp = ot.state().srp()?;

        let info = unwrap!(unsafe { otSrpClientGetHostInfo(instance).as_ref() });

        let conf = SrpConf {
            host_name: if !info.mName.is_null() {
                unwrap!(
                    unsafe { CStr::from_ptr(info.mName) }.to_str(),
                    "Invalid UTF-8 in SRP host name"
                )
            } else {
                ""
            },
            host_addrs: if info.mNumAddresses > 0 && !info.mAddresses.is_null() {
                unsafe {
                    core::slice::from_raw_parts(
                        info.mAddresses as *const _,
                        info.mNumAddresses as _,
                    )
                }
            } else {
                &[]
            },
            ttl: unsafe { otSrpClientGetTtl(instance) },
            default_lease_secs: unsafe { otSrpClientGetLeaseInterval(instance) },
            default_key_lease_secs: unsafe { otSrpClientGetKeyLeaseInterval(instance) },
        };

        f(&conf, info.mState.into(), !srp.conf_taken)
    }

    /// Return `true` if there is neither host, nor any service currently registered with the SRP client.
    pub fn srp_is_empty(&self) -> Result<bool, OtError> {
        let mut ot = self.activate();
        let state = ot.state();
        let srp = state.srp()?;

        Ok(!srp.conf_taken && srp.taken.iter().all(|&taken| !taken))
    }

    /// Set the SRP client configuration.
    ///
    /// Arguments:
    /// - `conf`: The SRP configuration.
    ///
    /// Returns:
    /// - `Ok(())` if the configuration was set successfully.
    /// - `Err(OtError)` if the configuration could not be set. One reason why the configuration setting
    ///   might fail is if the configuration had already been set and then not removed with `srp_remove_all`.
    pub fn srp_set_conf(&self, conf: &SrpConf) -> Result<(), OtError> {
        let mut ot = self.activate();
        let instance = ot.state().ot.instance;
        let srp = ot.state().srp()?;

        if srp.conf_taken {
            Err(OtError::new(otError_OT_ERROR_INVALID_STATE))?;
        }

        unsafe {
            otSrpClientSetLeaseInterval(instance, conf.default_lease_secs);
        }
        unsafe {
            otSrpClientSetKeyLeaseInterval(instance, conf.default_key_lease_secs);
        }
        unsafe {
            otSrpClientSetTtl(instance, conf.ttl);
        }

        let mut srp_conf = otSrpClientHostInfo {
            mName: core::ptr::null(),
            mAddresses: core::ptr::null(),
            mNumAddresses: 0,
            mAutoAddress: true,
            mState: 0,
        };

        conf.store(&mut srp_conf, srp.conf)?;
        srp.conf_taken = true;

        ot!(unsafe { otSrpClientSetHostName(instance, srp_conf.mName) })?;

        if !conf.host_addrs.is_empty() {
            ot!(unsafe {
                otSrpClientSetHostAddresses(instance, srp_conf.mAddresses, srp_conf.mNumAddresses)
            })?;
        } else {
            ot!(unsafe { otSrpClientEnableAutoHostAddress(instance) })?;
        }

        Ok(())
    }

    /// Return `true` if the SRP client is running, `false` otherwise.
    pub fn srp_running(&self) -> Result<bool, OtError> {
        let mut ot = self.activate();
        let instance = ot.state().ot.instance;
        let _ = ot.state().srp()?;

        Ok(unsafe { otSrpClientIsRunning(instance) })
    }

    /// Return `true` if the SRP client is in auto-start mode, `false` otherwise.
    pub fn srp_autostart_enabled(&self) -> Result<bool, OtError> {
        let mut ot = self.activate();
        let instance = ot.state().ot.instance;
        let _ = ot.state().srp()?;

        Ok(unsafe { otSrpClientIsAutoStartModeEnabled(instance) })
    }

    /// Auto-starts the SRP client.
    pub fn srp_autostart(&self) -> Result<(), OtError> {
        let mut ot = self.activate();
        let instance = ot.state().ot.instance;
        let _ = ot.state().srp()?;

        unsafe {
            otSrpClientEnableAutoStartMode(
                instance,
                Some(OtContext::plat_c_srp_auto_start_callback),
                instance as _,
            );
        }

        Ok(())
    }

    /// Start the SRP client for the given SRP server address.
    ///
    /// Arguments:
    /// - `server_addr`: The SRP server address.
    pub fn srp_start(&self, server_addr: SocketAddrV6) -> Result<(), OtError> {
        let mut ot = self.activate();
        let instance = ot.state().ot.instance;
        let _ = ot.state().srp()?;

        ot!(unsafe { otSrpClientStart(instance, &to_ot_addr(&server_addr)) })
    }

    /// Stop the SRP client.
    pub fn srp_stop(&self) -> Result<(), OtError> {
        let mut ot = self.activate();
        let instance = ot.state().ot.instance;
        let _ = ot.state().srp()?;

        unsafe {
            otSrpClientStop(instance);
        }

        Ok(())
    }

    /// Return the SRP server address, if the SRP client is running and
    /// had connected to a server.
    pub fn srp_server_addr(&self) -> Result<Option<SocketAddrV6>, OtError> {
        let mut ot = self.activate();
        let instance = ot.state().ot.instance;
        let _ = ot.state().srp()?;

        let addr = unwrap!(unsafe { otSrpClientGetServerAddress(instance).as_ref() });
        let addr = to_sock_addr(&addr.mAddress, addr.mPort, 0);

        // OT documentation notes that if the SRP client is not running
        // this will return the unspecified addr (0.0.0.0.0.0.0.0)
        Ok((!addr.ip().is_unspecified()).then_some(addr))
    }

    /// Iterate over the SRP services registered with the SRP client.
    ///
    /// Arguments:
    /// - `f`: A closure that receives a tuple of the next SRP service, SRP service state, and SRP service ID.
    ///   If there are no more SRP services, the closure will receive `None`.
    pub fn srp_services<F>(&self, mut f: F) -> Result<(), OtError>
    where
        F: FnMut(Option<(&OutSrpService<'_>, SrpState, SrpServiceSlot)>),
    {
        let mut ot = self.activate();
        let instance = ot.state().ot.instance;
        let srp = ot.state().srp()?;

        let mut service_ptr: *const otSrpClientService =
            unsafe { otSrpClientGetServices(instance) };

        while !service_ptr.is_null() {
            let service = unsafe { &*service_ptr };

            let slot = unwrap!(srp.services.iter().position(|s| core::ptr::eq(s, service)));

            f(Some((&service.into(), service.mState.into(), slot)));

            service_ptr = service.mNext;
        }

        f(None);

        Ok(())
    }

    /// Add an SRP service to the SRP client.
    ///
    /// Arguments:
    /// - `service`: The SRP service to add.
    ///
    /// Returns:
    /// - The SRP service slot, if the service was added successfully.
    /// - `Err(OtError)` if the service could not be added. One reason why the service addition
    ///   might fail is if there are no more slots available for services. This can happen even if all services
    ///   had been removed, as the slots are not freed until the SRP client propagates the removal info to the SRP server.
    pub fn srp_add_service<'a, SI, TI>(
        &self,
        service: &'a SrpService<'a, SI, TI>,
    ) -> Result<SrpServiceSlot, OtError>
    where
        SI: Iterator<Item = &'a str> + Clone + 'a,
        TI: Iterator<Item = (&'a str, &'a [u8])> + Clone + 'a,
    {
        let mut ot = self.activate();
        let instance = ot.state().ot.instance;
        let srp = ot.state().srp()?;

        let slot = srp
            .taken
            .iter()
            .position(|&taken| !taken)
            .ok_or(OtError::new(otError_OT_ERROR_NO_BUFS))?;

        let service_data = &mut srp.services[slot];
        let buf = &mut srp.buffers[srp.buf_len * slot..srp.buf_len * (slot + 1)];

        service.store(service_data, buf)?;

        ot!(unsafe { otSrpClientAddService(instance, service_data) })?;

        debug!("Service added");

        srp.taken[slot] = true;

        Ok(slot)
    }

    /// Remove an SRP service from the SRP client.
    ///
    /// Arguments:
    /// - `slot`: The SRP service to remove.
    /// - `immediate`: If `true`, the service will be removed immediately, otherwise, the service will be removed gracefully
    ///   by propagating the removal info to the SRP server.
    pub fn srp_remove_service(&self, slot: SrpServiceSlot, immediate: bool) -> Result<(), OtError> {
        let mut ot = self.activate();
        let instance = ot.state().ot.instance;
        let srp = ot.state().srp()?;

        if slot > srp.services.len() || !srp.taken[slot] {
            Err(OtError::new(otError_OT_ERROR_INVALID_ARGS))?;
        }

        if immediate {
            ot!(unsafe { otSrpClientClearService(instance, &mut srp.services[slot]) })?;
            srp.taken[slot] = false;
            debug!("Service {} cleared immeidately", slot);
        } else {
            ot!(unsafe { otSrpClientRemoveService(instance, &mut srp.services[slot]) })?;
            debug!("Service {} scheduled for removal", slot);
        }

        Ok(())
    }

    /// Remove the SRP hostname and all SRP services from the SRP client.
    ///
    /// Arguments:
    /// - `immediate`: If `true`, the hostname and services will be removed immediately, otherwise,
    ///   the hostname and services will be removed gracefully by propagating the removal info to the SRP server.
    pub fn srp_remove_all(&self, immediate: bool) -> Result<(), OtError> {
        let mut ot = self.activate();
        let instance = ot.state().ot.instance;
        let srp = ot.state().srp()?;

        if immediate {
            unsafe {
                otSrpClientClearHostAndServices(instance);
            }

            srp.conf_taken = false;
            srp.taken.fill(false);

            debug!("Hostname and all services cleared immediately");
        } else {
            ot!(unsafe { otSrpClientRemoveHostAndServices(instance, false, true) })?;
            debug!("Hostname and all services scheduled for removal");
        }

        Ok(())
    }

    /// Wait for the SRP state to change.
    ///
    /// This method will wait forever if `OpenThread` is not instantiated with SRP.
    ///
    /// NOTE:
    /// It is not advised to call this method concurrently from multiple async tasks
    /// because it uses a single waker registration. Thus, while the method will not panic,
    /// the tasks will fight with each other by each re-registering its own waker, thus keeping the CPU constantly busy.
    pub async fn srp_wait_changed(&self) {
        if self.activate().state().srp().is_ok() {
            poll_fn(move |cx| {
                unwrap!(self.activate().state().srp.as_mut())
                    .changes
                    .poll_wait(cx)
            })
            .await;
        } else {
            core::future::pending::<()>().await;
        }
    }
}

impl OtContext<'_> {
    /// Reclaims the slots of the SRP host and services that are reported as removed
    fn cleanup(
        &mut self,
        host_info: &otSrpClientHostInfo,
        mut removed_services: Option<&otSrpClientService>,
    ) {
        let state = self.state();

        if let Ok(srp) = state.srp() {
            if host_info.mState == otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REMOVED {
                srp.conf_taken = false;
                info!("SRP host removed");
            }

            while let Some(service) = removed_services {
                let slot = unwrap!(srp.services.iter().position(|s| core::ptr::eq(s, service)));

                removed_services = unsafe { service.mNext.as_ref() };

                srp.taken[slot] = false;
                info!("SRP service at slot {} removed", slot);
            }
        }
    }

    pub(crate) fn plat_srp_auto_started(&mut self) {
        let state = self.state();
        if let Ok(srp) = state.srp() {
            srp.changes.signal(());
            state.ot.changes.signal(());
        }
    }

    pub(crate) fn plat_srp_changed(
        &mut self,
        host_info: &otSrpClientHostInfo,
        _services: Option<&otSrpClientService>,
        removed_services: Option<&otSrpClientService>,
    ) {
        trace!("Plat changed callback");

        self.cleanup(host_info, removed_services);

        let state = self.state();
        if let Ok(srp) = state.srp() {
            srp.changes.signal(());
            state.ot.changes.signal(());
        }
    }
}

fn align_min<T>(buf: &mut [u8], count: usize) -> Result<(&mut [T], &mut [u8]), OtError> {
    if count == 0 || core::mem::size_of::<T>() == 0 {
        return Ok((&mut [], buf));
    }

    let (t_leading_buf0, t_buf, _) = unsafe { buf.align_to_mut::<T>() };
    if t_buf.len() < count {
        Err(OtError::new(otError_OT_ERROR_NO_BUFS))?;
    }

    // Shrink `t_buf` to the number of requested items (count)
    let t_buf = &mut t_buf[..count];
    let t_leading_buf0_len = t_leading_buf0.len();
    let t_buf_size = core::mem::size_of_val(t_buf);

    let (buf0, remaining_buf) = buf.split_at_mut(t_leading_buf0_len + t_buf_size);

    let (t_leading_buf, t_buf, t_remaining_buf) = unsafe { buf0.align_to_mut::<T>() };
    assert_eq!(t_leading_buf0_len, t_leading_buf.len());
    assert_eq!(t_buf.len(), count);
    assert!(t_remaining_buf.is_empty());

    Ok((t_buf, remaining_buf))
}

fn store_str<'t>(str: &str, buf: &'t mut [u8]) -> Result<(&'t CStr, &'t mut [u8]), OtError> {
    let data_len = str.len() + 1;

    if data_len > buf.len() {
        Err(OtError::new(otError_OT_ERROR_NO_BUFS))?;
    }

    let (str_buf, rem_buf) = buf.split_at_mut(data_len);

    str_buf[..str.len()].copy_from_slice(str.as_bytes());
    str_buf[str.len()] = 0;

    Ok((
        unwrap!(
            CStr::from_bytes_with_nul(&str_buf[..data_len]),
            "Invalid Cstr"
        ),
        rem_buf,
    ))
}

fn store_data<'t>(data: &[u8], buf: &'t mut [u8]) -> Result<(&'t [u8], &'t mut [u8]), OtError> {
    if data.len() > buf.len() {
        Err(OtError::new(otError_OT_ERROR_NO_BUFS))?;
    }

    let (data_buf, rem_buf) = buf.split_at_mut(data.len());

    data_buf[..data.len()].copy_from_slice(data);

    Ok((data_buf, rem_buf))
}
