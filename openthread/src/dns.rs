//! A safe, generic wrapper over OpenThread's DNS client API
//! (`OPENTHREAD_CONFIG_DNS_CLIENT_ENABLE` / the `dns-client` feature).
//!
//! Three query primitives are exposed, mirroring OpenThread 1:1:
//! - [`OpenThread::dns_browse`] - DNS-SD browse (service instance enumeration).
//! - [`OpenThread::dns_resolve_service`] /
//!   [`OpenThread::dns_resolve_service_and_host_address`] - service instance
//!   resolution (SRV / TXT, optionally followed by host AAAA resolution).
//! - [`OpenThread::dns_resolve_address`] - host name -> IPv6 address(es).
//!
//! Each query is `async` and completes when OpenThread invokes its response
//! callback (on a response or a time-out). The user passes a closure that is
//! invoked **once**, synchronously, from within that callback, with a borrowed
//! response accessor ([`DnsBrowseResponse`], [`DnsServiceResponse`] or
//! [`DnsAddressResponse`]). The accessors expose OpenThread's index-based getter
//! functions, reading variable-length data (names, TXT) into caller-supplied
//! buffers - because the underlying response is only valid for the duration of
//! the callback and must not be retained.
//!
//! This wrapper is intentionally Matter-agnostic: it surfaces the raw DNS-SD
//! capability so it is reusable for any DNS use case, not only Matter discovery.
//!
//! Only one DNS query may be in flight at a time per `OpenThread` instance;
//! starting another while one is pending returns [`otError`] `BUSY`.

use core::ffi::{c_void, CStr};
use core::future::poll_fn;
use core::net::Ipv6Addr;

use crate::sys::{
    otDnsAddressResponse, otDnsAddressResponseGetAddress, otDnsAddressResponseGetHostName,
    otDnsBrowseResponse, otDnsBrowseResponseGetHostAddress, otDnsBrowseResponseGetServiceInfo,
    otDnsBrowseResponseGetServiceInstance, otDnsBrowseResponseGetServiceName, otDnsClientBrowse,
    otDnsClientResolveAddress, otDnsClientResolveService, otDnsClientResolveServiceAndHostAddress,
    otDnsQueryConfig, otDnsRecursionFlag, otDnsRecursionFlag_OT_DNS_FLAG_NO_RECURSION,
    otDnsRecursionFlag_OT_DNS_FLAG_RECURSION_DESIRED, otDnsRecursionFlag_OT_DNS_FLAG_UNSPECIFIED,
    otDnsServiceInfo, otDnsServiceMode, otDnsServiceMode_OT_DNS_SERVICE_MODE_SRV,
    otDnsServiceMode_OT_DNS_SERVICE_MODE_SRV_TXT,
    otDnsServiceMode_OT_DNS_SERVICE_MODE_SRV_TXT_OPTIMIZE,
    otDnsServiceMode_OT_DNS_SERVICE_MODE_SRV_TXT_SEPARATE,
    otDnsServiceMode_OT_DNS_SERVICE_MODE_TXT, otDnsServiceMode_OT_DNS_SERVICE_MODE_UNSPECIFIED,
    otDnsServiceResponse, otDnsServiceResponseGetHostAddress, otDnsServiceResponseGetServiceInfo,
    otDnsServiceResponseGetServiceName, otError, otError_OT_ERROR_BUSY, otError_OT_ERROR_NONE,
    otError_OT_ERROR_NOT_FOUND, otInstance, otIp6Address,
};
use crate::{ot, to_ot_addr, OpenThread, OtContext, OtError};

use core::net::SocketAddrV6;

/// Which records the DNS client queries during a service resolution.
///
/// Mirrors `otDnsServiceMode`.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DnsServiceMode {
    /// Use the default mode from the DNS client's default config.
    #[default]
    Unspecified,
    /// Query for SRV record only.
    Srv,
    /// Query for TXT record only.
    Txt,
    /// Query for both SRV and TXT records in the same message.
    SrvTxt,
    /// Query for SRV and TXT records in separate, parallel messages.
    SrvTxtSeparate,
    /// Try SRV and TXT together first, then separately if that fails.
    SrvTxtOptimize,
}

impl DnsServiceMode {
    const fn to_ot(self) -> otDnsServiceMode {
        match self {
            Self::Unspecified => otDnsServiceMode_OT_DNS_SERVICE_MODE_UNSPECIFIED,
            Self::Srv => otDnsServiceMode_OT_DNS_SERVICE_MODE_SRV,
            Self::Txt => otDnsServiceMode_OT_DNS_SERVICE_MODE_TXT,
            Self::SrvTxt => otDnsServiceMode_OT_DNS_SERVICE_MODE_SRV_TXT,
            Self::SrvTxtSeparate => otDnsServiceMode_OT_DNS_SERVICE_MODE_SRV_TXT_SEPARATE,
            Self::SrvTxtOptimize => otDnsServiceMode_OT_DNS_SERVICE_MODE_SRV_TXT_OPTIMIZE,
        }
    }
}

/// Whether the DNS server is asked to resolve the query recursively.
///
/// Mirrors `otDnsRecursionFlag`.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DnsRecursion {
    /// Use the default from the DNS client's default config.
    #[default]
    Unspecified,
    /// Ask the server to resolve recursively.
    Desired,
    /// Ask the server not to resolve recursively.
    None,
}

impl DnsRecursion {
    const fn to_ot(self) -> otDnsRecursionFlag {
        match self {
            Self::Unspecified => otDnsRecursionFlag_OT_DNS_FLAG_UNSPECIFIED,
            Self::Desired => otDnsRecursionFlag_OT_DNS_FLAG_RECURSION_DESIRED,
            Self::None => otDnsRecursionFlag_OT_DNS_FLAG_NO_RECURSION,
        }
    }
}

/// Per-query DNS configuration, mirroring `otDnsQueryConfig`.
///
/// Every field is optional: a `None`/`Unspecified`/zero value tells OpenThread
/// to use the corresponding field from its default config (see
/// `otDnsClientGetDefaultConfig`). Passing `None` for the whole config to a
/// query uses the default config for all fields.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Default)]
pub struct DnsQueryConfig {
    /// DNS server to query. `None` uses the default server.
    pub server: Option<SocketAddrV6>,
    /// Response wait time in milliseconds. `None`/zero = default.
    pub response_timeout_ms: Option<u32>,
    /// Maximum transmit attempts before failing. `None`/zero = default.
    pub max_tx_attempts: Option<u8>,
    /// Recursion preference.
    pub recursion: DnsRecursion,
    /// Which records to query during service resolution.
    pub service_mode: DnsServiceMode,
}

impl DnsQueryConfig {
    fn to_ot(self) -> otDnsQueryConfig {
        otDnsQueryConfig {
            mServerSockAddr: self
                .server
                .map(|addr| to_ot_addr(&addr))
                .unwrap_or_else(zeroed_sock_addr),
            mResponseTimeout: self.response_timeout_ms.unwrap_or(0),
            mMaxTxAttempts: self.max_tx_attempts.unwrap_or(0),
            mRecursionFlag: self.recursion.to_ot(),
            // NAT64 and transport selection are left at their default
            // (unspecified = zero); callers needing them can be added later.
            mNat64Mode: 0,
            mServiceMode: self.service_mode.to_ot(),
            mTransportProto: 0,
        }
    }
}

fn zeroed_sock_addr() -> crate::sys::otSockAddr {
    crate::sys::otSockAddr {
        mAddress: otIp6Address {
            mFields: crate::sys::otIp6Address__bindgen_ty_1 { m8: [0; 16] },
        },
        mPort: 0,
    }
}

/// Information about a discovered DNS service instance (from an SRV/TXT lookup).
///
/// Borrows the caller-supplied host-name and TXT buffers that were filled by the
/// originating getter call.
#[derive(Debug)]
pub struct DnsServiceInfo<'a> {
    /// Service record TTL (seconds).
    pub ttl: u32,
    /// Service port (from the SRV record).
    pub port: u16,
    /// Service priority (from the SRV record).
    pub priority: u16,
    /// Service weight (from the SRV record).
    pub weight: u16,
    /// The host name the service resolves to, if requested/available.
    pub host_name: Option<&'a str>,
    /// The host's first IPv6 address, if available (all-zero = unavailable).
    pub host_address: Option<Ipv6Addr>,
    /// TTL of the host address (seconds).
    pub host_address_ttl: u32,
    /// Raw TXT record data, if requested/available.
    pub txt_data: Option<&'a [u8]>,
    /// Whether the TXT data did not fit in the supplied buffer and was truncated.
    pub txt_data_truncated: bool,
    /// TTL of the TXT data (seconds).
    pub txt_data_ttl: u32,
}

/// Build an `otDnsServiceInfo` pointing at the caller's host-name and TXT
/// scratch buffers, run `getter`, then parse the result into a [`DnsServiceInfo`].
///
/// Either buffer may be empty to skip retrieving that piece of info.
fn read_service_info<'a, F>(
    host_buf: &'a mut [u8],
    txt_buf: &'a mut [u8],
    getter: F,
) -> Result<DnsServiceInfo<'a>, OtError>
where
    F: FnOnce(*mut otDnsServiceInfo) -> otError,
{
    let mut info = otDnsServiceInfo {
        mTtl: 0,
        mPort: 0,
        mPriority: 0,
        mWeight: 0,
        mHostNameBuffer: if host_buf.is_empty() {
            core::ptr::null_mut()
        } else {
            host_buf.as_mut_ptr() as *mut _
        },
        mHostNameBufferSize: host_buf.len() as u16,
        mHostAddress: otIp6Address {
            mFields: crate::sys::otIp6Address__bindgen_ty_1 { m8: [0; 16] },
        },
        mHostAddressTtl: 0,
        mTxtData: if txt_buf.is_empty() {
            core::ptr::null_mut()
        } else {
            txt_buf.as_mut_ptr()
        },
        mTxtDataSize: txt_buf.len() as u16,
        mTxtDataTruncated: false,
        mTxtDataTtl: 0,
    };

    ot!(getter(&mut info))?;

    let host_name = (!info.mHostNameBuffer.is_null())
        .then(|| {
            unsafe { CStr::from_ptr(info.mHostNameBuffer) }
                .to_str()
                .ok()
        })
        .flatten();

    let host_octets = unsafe { info.mHostAddress.mFields.m8 };
    let host_address = (host_octets != [0; 16]).then(|| Ipv6Addr::from(host_octets));

    let txt_data = (!info.mTxtData.is_null() && info.mTxtDataSize > 0)
        .then(|| unsafe { core::slice::from_raw_parts(info.mTxtData, info.mTxtDataSize as usize) });

    Ok(DnsServiceInfo {
        ttl: info.mTtl,
        port: info.mPort,
        priority: info.mPriority,
        weight: info.mWeight,
        host_name,
        host_address,
        host_address_ttl: info.mHostAddressTtl,
        txt_data,
        txt_data_truncated: info.mTxtDataTruncated,
        txt_data_ttl: info.mTxtDataTtl,
    })
}

/// Read a name via an OpenThread `...GetName`/`...GetServiceName` getter into
/// `buf`, returning it as a `&str`. The getter must null-terminate.
fn read_name<F>(buf: &mut [u8], getter: F) -> Result<&str, OtError>
where
    F: FnOnce(*mut core::ffi::c_char, u16) -> otError,
{
    ot!(getter(buf.as_mut_ptr() as *mut _, buf.len() as u16))?;

    let cstr = unsafe { CStr::from_ptr(buf.as_ptr() as *const _) };
    cstr.to_str()
        .map_err(|_| OtError::new(crate::sys::otError_OT_ERROR_PARSE))
}

/// Read an index-based IPv6 address getter (`...GetAddress`/`...GetHostAddress`).
///
/// Returns `Ok(Some(addr))` for a present record, `Ok(None)` when the index is
/// past the end (`OT_ERROR_NOT_FOUND`), or an error otherwise.
#[allow(non_upper_case_globals)]
fn read_address<F>(getter: F) -> Result<Option<(Ipv6Addr, u32)>, OtError>
where
    F: FnOnce(*mut otIp6Address, *mut u32) -> otError,
{
    let mut addr = otIp6Address {
        mFields: crate::sys::otIp6Address__bindgen_ty_1 { m8: [0; 16] },
    };
    let mut ttl: u32 = 0;

    match getter(&mut addr, &mut ttl) {
        otError_OT_ERROR_NONE => Ok(Some((Ipv6Addr::from(unsafe { addr.mFields.m8 }), ttl))),
        otError_OT_ERROR_NOT_FOUND => Ok(None),
        err => Err(OtError::new(err)),
    }
}

/// A borrowed accessor over a DNS browse (service instance enumeration) response.
///
/// Valid only within the browse callback closure; do not retain it. Service
/// instances are enumerated by index via [`DnsBrowseResponse::service_instance`].
pub struct DnsBrowseResponse(*const otDnsBrowseResponse);

impl DnsBrowseResponse {
    /// The queried service name (e.g. `_service._udp.domain`), into `buf`.
    pub fn service_name<'a>(&self, buf: &'a mut [u8]) -> Result<&'a str, OtError> {
        read_name(buf, |p, n| unsafe {
            otDnsBrowseResponseGetServiceName(self.0, p, n)
        })
    }

    /// The service instance label at `index` (just the leading label, not the
    /// full name), into `buf`. Returns `Ok(None)` once the index is past the end.
    #[allow(non_upper_case_globals)]
    pub fn service_instance<'a>(
        &self,
        index: u16,
        buf: &'a mut [u8],
    ) -> Result<Option<&'a str>, OtError> {
        match unsafe {
            otDnsBrowseResponseGetServiceInstance(
                self.0,
                index,
                buf.as_mut_ptr() as *mut _,
                buf.len() as u8,
            )
        } {
            otError_OT_ERROR_NONE => {
                let cstr = unsafe { CStr::from_ptr(buf.as_ptr() as *const _) };
                Ok(Some(cstr.to_str().map_err(|_| {
                    OtError::new(crate::sys::otError_OT_ERROR_PARSE)
                })?))
            }
            otError_OT_ERROR_NOT_FOUND => Ok(None),
            err => Err(OtError::new(err)),
        }
    }

    /// Service info (SRV/TXT/AAAA, when present in the response) for the instance
    /// labelled `instance_label` (as returned by
    /// [`DnsBrowseResponse::service_instance`]). `host_buf`/`txt_buf` receive the
    /// host name and TXT data; pass an empty slice to skip either.
    pub fn service_info<'a>(
        &self,
        instance_label: &str,
        host_buf: &'a mut [u8],
        txt_buf: &'a mut [u8],
    ) -> Result<DnsServiceInfo<'a>, OtError> {
        let label = CName::new(instance_label)?;
        read_service_info(host_buf, txt_buf, |info| unsafe {
            otDnsBrowseResponseGetServiceInfo(self.0, label.as_ptr(), info)
        })
    }

    /// The `index`-th IPv6 address advertised for `host_name` in the response.
    /// Returns `Ok(None)` once the index is past the end.
    pub fn host_address(
        &self,
        host_name: &str,
        index: u16,
    ) -> Result<Option<(Ipv6Addr, u32)>, OtError> {
        let host = CName::new(host_name)?;
        read_address(|addr, ttl| unsafe {
            otDnsBrowseResponseGetHostAddress(self.0, host.as_ptr(), index, addr, ttl)
        })
    }
}

/// A borrowed accessor over a DNS service instance resolution response.
///
/// Valid only within the resolve-service callback closure; do not retain it.
pub struct DnsServiceResponse(*const otDnsServiceResponse);

impl DnsServiceResponse {
    /// The resolved service instance name: the leading label into `label_buf`
    /// and the rest of the name into `name_buf` (pass an empty `name_buf` to
    /// skip the latter). Returns the label.
    pub fn service_name<'a>(
        &self,
        label_buf: &'a mut [u8],
        name_buf: &mut [u8],
    ) -> Result<&'a str, OtError> {
        let label_ptr = label_buf.as_mut_ptr() as *mut core::ffi::c_char;
        let label_cap = label_buf.len() as u8;
        let (name_ptr, name_cap) = if name_buf.is_empty() {
            (core::ptr::null_mut(), 0)
        } else {
            (
                name_buf.as_mut_ptr() as *mut core::ffi::c_char,
                name_buf.len() as u16,
            )
        };

        ot!(unsafe {
            otDnsServiceResponseGetServiceName(self.0, label_ptr, label_cap, name_ptr, name_cap)
        })?;

        let cstr = unsafe { CStr::from_ptr(label_buf.as_ptr() as *const _) };
        cstr.to_str()
            .map_err(|_| OtError::new(crate::sys::otError_OT_ERROR_PARSE))
    }

    /// Service info (SRV/TXT, plus AAAA when provided) from the response.
    /// `host_buf`/`txt_buf` receive the host name and TXT data; pass an empty
    /// slice to skip either.
    pub fn service_info<'a>(
        &self,
        host_buf: &'a mut [u8],
        txt_buf: &'a mut [u8],
    ) -> Result<DnsServiceInfo<'a>, OtError> {
        read_service_info(host_buf, txt_buf, |info| unsafe {
            otDnsServiceResponseGetServiceInfo(self.0, info)
        })
    }

    /// The `index`-th IPv6 address advertised for `host_name`. Returns
    /// `Ok(None)` once the index is past the end.
    pub fn host_address(
        &self,
        host_name: &str,
        index: u16,
    ) -> Result<Option<(Ipv6Addr, u32)>, OtError> {
        let host = CName::new(host_name)?;
        read_address(|addr, ttl| unsafe {
            otDnsServiceResponseGetHostAddress(self.0, host.as_ptr(), index, addr, ttl)
        })
    }
}

/// A borrowed accessor over a DNS address (AAAA) resolution response.
///
/// Valid only within the resolve-address callback closure; do not retain it.
pub struct DnsAddressResponse(*const otDnsAddressResponse);

impl DnsAddressResponse {
    /// The queried host name, into `buf`.
    pub fn host_name<'a>(&self, buf: &'a mut [u8]) -> Result<&'a str, OtError> {
        read_name(buf, |p, n| unsafe {
            otDnsAddressResponseGetHostName(self.0, p, n)
        })
    }

    /// The `index`-th resolved IPv6 address. Returns `Ok(None)` once the index
    /// is past the end.
    pub fn address(&self, index: u16) -> Result<Option<(Ipv6Addr, u32)>, OtError> {
        read_address(|addr, ttl| unsafe {
            otDnsAddressResponseGetAddress(self.0, index, addr, ttl)
        })
    }
}

/// The borrowed response handed to a DNS query closure, tagged by query kind.
///
/// A given query only ever yields its matching variant; the closure typically
/// matches on the one it expects.
pub enum DnsResponse {
    /// A browse (service instance enumeration) response.
    Browse(DnsBrowseResponse),
    /// A service instance resolution response.
    Service(DnsServiceResponse),
    /// An address (AAAA) resolution response.
    Address(DnsAddressResponse),
}

impl<'a> OpenThread<'a> {
    /// Send a DNS-SD browse (service instance enumeration) query for
    /// `service_name` (e.g. `_service._udp.domain`).
    ///
    /// On a response or time-out, `f` is invoked once with
    /// [`DnsResponse::Browse`]. Returns the DNS transaction result.
    pub async fn dns_browse<F>(
        &self,
        service_name: &str,
        config: Option<&DnsQueryConfig>,
        f: F,
    ) -> Result<(), OtError>
    where
        F: FnMut(&DnsResponse),
    {
        self.dns_query(config, f, |instance, ctx, ot_config| {
            let name = CName::new(service_name)?;
            ot!(unsafe {
                otDnsClientBrowse(
                    instance,
                    name.as_ptr(),
                    Some(Self::plat_c_dns_browse_callback),
                    ctx,
                    ot_config,
                )
            })
        })
        .await
    }

    /// Resolve a service instance (`<instance_label>.<service_name>`), querying
    /// SRV/TXT records per the config's service mode.
    ///
    /// Does not perform a separate host address resolution; any host AAAA records
    /// are only available if the server included them. Use
    /// [`OpenThread::dns_resolve_service_and_host_address`] to also resolve the
    /// host address. On a response or time-out, `f` is invoked once with
    /// [`DnsResponse::Service`].
    pub async fn dns_resolve_service<F>(
        &self,
        instance_label: &str,
        service_name: &str,
        config: Option<&DnsQueryConfig>,
        f: F,
    ) -> Result<(), OtError>
    where
        F: FnMut(&DnsResponse),
    {
        self.dns_query(config, f, |instance, ctx, ot_config| {
            let label = CName::new(instance_label)?;
            let name = CName::new(service_name)?;
            ot!(unsafe {
                otDnsClientResolveService(
                    instance,
                    label.as_ptr(),
                    name.as_ptr(),
                    Some(Self::plat_c_dns_service_callback),
                    ctx,
                    ot_config,
                )
            })
        })
        .await
    }

    /// Like [`OpenThread::dns_resolve_service`], but additionally resolves the
    /// host name discovered from the SRV record (a follow-up AAAA query) when the
    /// server did not include the host address. The callback fires once both
    /// resolutions complete. Cannot be used with `DnsServiceMode::Txt`.
    pub async fn dns_resolve_service_and_host_address<F>(
        &self,
        instance_label: &str,
        service_name: &str,
        config: Option<&DnsQueryConfig>,
        f: F,
    ) -> Result<(), OtError>
    where
        F: FnMut(&DnsResponse),
    {
        self.dns_query(config, f, |instance, ctx, ot_config| {
            let label = CName::new(instance_label)?;
            let name = CName::new(service_name)?;
            ot!(unsafe {
                otDnsClientResolveServiceAndHostAddress(
                    instance,
                    label.as_ptr(),
                    name.as_ptr(),
                    Some(Self::plat_c_dns_service_callback),
                    ctx,
                    ot_config,
                )
            })
        })
        .await
    }

    /// Resolve a host name to its IPv6 (AAAA) address(es).
    ///
    /// On a response or time-out, `f` is invoked once with
    /// [`DnsResponse::Address`].
    pub async fn dns_resolve_address<F>(
        &self,
        host_name: &str,
        config: Option<&DnsQueryConfig>,
        f: F,
    ) -> Result<(), OtError>
    where
        F: FnMut(&DnsResponse),
    {
        self.dns_query(config, f, |instance, ctx, ot_config| {
            let name = CName::new(host_name)?;
            ot!(unsafe {
                otDnsClientResolveAddress(
                    instance,
                    name.as_ptr(),
                    Some(Self::plat_c_dns_address_callback),
                    ctx,
                    ot_config,
                )
            })
        })
        .await
    }

    /// Shared driver for the three DNS query kinds: install the user closure,
    /// start the query via `start`, await the completion signal, and return the
    /// transaction error. Errors with `BUSY` if a DNS query is already in flight.
    async fn dns_query<F, S>(
        &self,
        config: Option<&DnsQueryConfig>,
        mut f: F,
        start: S,
    ) -> Result<(), OtError>
    where
        F: FnMut(&DnsResponse),
        S: FnOnce(*mut otInstance, *mut c_void, *const otDnsQueryConfig) -> Result<(), OtError>,
    {
        let config_storage;

        {
            let mut ot = self.activate();
            let state = ot.state();

            if state.ot.dns_callback.is_some() {
                warn!("Another DNS query in progress");
                return Err(OtError::new(otError_OT_ERROR_BUSY));
            }

            let instance = state.ot.instance;

            // Install the (lifetime-erased) user closure. Same pattern and the
            // same `mem::forget` caveat as `scan` (see `scan.rs`).
            let f: &mut dyn FnMut(&DnsResponse) = &mut f;
            state.ot.dns_callback = Some(unsafe {
                core::mem::transmute::<&mut dyn FnMut(&DnsResponse), &'a mut dyn FnMut(&DnsResponse)>(
                    f,
                )
            });

            config_storage = config.map(|c| c.to_ot());
            let config_ptr = config_storage
                .as_ref()
                .map_or(core::ptr::null(), |c| c as *const _);

            let res = start(instance, instance as *mut _ as *mut c_void, config_ptr);

            if res.is_err() {
                // Query never started; release the slot.
                state.ot.dns_callback = None;
                res?;
            }
        }

        let _guard = scopeguard::guard((), |_| {
            let mut ot = self.activate();
            ot.state().ot.dns_callback = None;
        });

        let error = poll_fn(move |cx| self.activate().state().ot.dns_done.poll_wait(cx)).await;

        ot!(error)
    }

    unsafe extern "C" fn plat_c_dns_browse_callback(
        error: otError,
        response: *const otDnsBrowseResponse,
        context: *mut c_void,
    ) {
        Self::dns_dispatch(
            error,
            context,
            DnsResponse::Browse(DnsBrowseResponse(response)),
        );
    }

    unsafe extern "C" fn plat_c_dns_service_callback(
        error: otError,
        response: *const otDnsServiceResponse,
        context: *mut c_void,
    ) {
        Self::dns_dispatch(
            error,
            context,
            DnsResponse::Service(DnsServiceResponse(response)),
        );
    }

    unsafe extern "C" fn plat_c_dns_address_callback(
        error: otError,
        response: *const otDnsAddressResponse,
        context: *mut c_void,
    ) {
        Self::dns_dispatch(
            error,
            context,
            DnsResponse::Address(DnsAddressResponse(response)),
        );
    }

    /// Common tail for the three DNS response trampolines: invoke the installed
    /// user closure with the borrowed response, clear the slot, and signal the
    /// awaiting future with the transaction error.
    fn dns_dispatch(error: otError, context: *mut c_void, response: DnsResponse) {
        let instance = context as *mut otInstance;

        let mut ot = OtContext::callback(instance);
        let state = ot.state();

        if let Some(f) = state.ot.dns_callback.as_mut() {
            f(&response);
        }

        state.ot.dns_callback = None;
        state.ot.dns_done.signal(error);
    }
}

/// A stack-allocated, null-terminated copy of a DNS name to hand to the C API.
///
/// DNS names are bounded (<= 255 bytes); a generous fixed buffer avoids needing
/// a caller-provided scratch buffer for the query side.
struct CName {
    buf: [u8; Self::CAP],
    len: usize,
}

impl CName {
    const CAP: usize = 256; // 255 max name + NUL

    fn new(name: &str) -> Result<Self, OtError> {
        if name.len() + 1 > Self::CAP {
            return Err(OtError::new(crate::sys::otError_OT_ERROR_INVALID_ARGS));
        }

        let mut buf = [0u8; Self::CAP];
        buf[..name.len()].copy_from_slice(name.as_bytes());
        // `buf` is zero-initialized, so it is already NUL-terminated.

        Ok(Self {
            buf,
            len: name.len() + 1,
        })
    }

    fn as_ptr(&self) -> *const core::ffi::c_char {
        debug_assert_eq!(self.buf[self.len - 1], 0);
        self.buf.as_ptr() as *const _
    }
}
