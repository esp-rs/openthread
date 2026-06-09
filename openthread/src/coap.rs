use core::cell::RefCell;
use core::ffi::c_void;
use core::future::poll_fn;
use core::mem::MaybeUninit;
use core::net::SocketAddrV6;
use core::ptr;

use crate::signal::Signal;
use crate::sys::{
    otCoapAddResource, otCoapCode, otCoapCode_OT_COAP_CODE_EMPTY, otCoapMessageAppendObserveOption,
    otCoapMessageAppendUriPathOptions, otCoapMessageGenerateToken, otCoapMessageGetCode,
    otCoapMessageGetMessageId, otCoapMessageGetToken, otCoapMessageGetTokenLength,
    otCoapMessageGetType, otCoapMessageInit, otCoapMessageInitResponse,
    otCoapMessageSetPayloadMarker, otCoapMessageSetToken, otCoapNewMessage, otCoapOptionIterator,
    otCoapOptionIteratorGetFirstOptionMatching, otCoapOptionIteratorGetOptionUintValue,
    otCoapOptionIteratorInit, otCoapOptionType_OT_COAP_OPTION_OBSERVE, otCoapRemoveResource,
    otCoapResource, otCoapSendRequestWithParameters, otCoapSendResponseWithParameters, otCoapStart,
    otCoapStop, otCoapType, otCoapType_OT_COAP_TYPE_ACKNOWLEDGMENT,
    otCoapType_OT_COAP_TYPE_CONFIRMABLE, otCoapType_OT_COAP_TYPE_NON_CONFIRMABLE,
    otCoapType_OT_COAP_TYPE_RESET, otError, otError_OT_ERROR_INVALID_ARGS, otError_OT_ERROR_NONE,
    otError_OT_ERROR_NO_BUFS, otIp6Address, otIp6Address__bindgen_ty_1, otMessage, otMessageAppend,
    otMessageFree, otMessageGetLength, otMessageGetOffset, otMessageInfo, otMessageRead,
    OT_COAP_DEFAULT_TOKEN_LENGTH,
};
use crate::{ot, Bytes, OpenThread, OtContext, OtError};

/// Maximum CoAP token length
pub const COAP_MAX_TOKEN_LEN: usize = 8;

/// Maximum URI path length
pub const COAP_MAX_URI_LEN: usize = 32;

/// Default CoAP UDP port
pub const COAP_DEFAULT_PORT: u16 = 5683;

/// CoAP message type.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CoapType {
    Confirmable,
    NonConfirmable,
    Acknowledgment,
    Reset,
}

impl CoapType {
    pub fn as_raw(self) -> otCoapType {
        match self {
            Self::Confirmable => otCoapType_OT_COAP_TYPE_CONFIRMABLE,
            Self::NonConfirmable => otCoapType_OT_COAP_TYPE_NON_CONFIRMABLE,
            Self::Acknowledgment => otCoapType_OT_COAP_TYPE_ACKNOWLEDGMENT,
            Self::Reset => otCoapType_OT_COAP_TYPE_RESET,
        }
    }

    #[allow(non_upper_case_globals)]
    pub fn from_raw(raw: otCoapType) -> Self {
        match raw {
            otCoapType_OT_COAP_TYPE_CONFIRMABLE => Self::Confirmable,
            otCoapType_OT_COAP_TYPE_NON_CONFIRMABLE => Self::NonConfirmable,
            otCoapType_OT_COAP_TYPE_ACKNOWLEDGMENT => Self::Acknowledgment,
            _ => Self::Reset,
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CoapCode(pub u8);

impl CoapCode {
    pub const EMPTY: Self = Self::new(0, 0);
    pub const GET: Self = Self::new(0, 1);
    pub const POST: Self = Self::new(0, 2);
    pub const PUT: Self = Self::new(0, 3);
    pub const DELETE: Self = Self::new(0, 4);

    pub const CREATED: Self = Self::new(2, 1);
    pub const DELETED: Self = Self::new(2, 2);
    pub const VALID: Self = Self::new(2, 3);
    pub const CHANGED: Self = Self::new(2, 4);
    pub const CONTENT: Self = Self::new(2, 5);

    pub const BAD_REQUEST: Self = Self::new(4, 0);
    pub const UNAUTHORIZED: Self = Self::new(4, 1);
    pub const FORBIDDEN: Self = Self::new(4, 3);
    pub const NOT_FOUND: Self = Self::new(4, 4);
    pub const METHOD_NOT_ALLOWED: Self = Self::new(4, 5);

    pub const INTERNAL_ERROR: Self = Self::new(5, 0);
    pub const NOT_IMPLEMENTED: Self = Self::new(5, 1);

    pub const fn new(class: u8, detail: u8) -> Self {
        Self(((class & 0x7) << 5) | (detail & 0x1f))
    }
    pub const fn class(self) -> u8 {
        (self.0 >> 5) & 0x7
    }
    pub const fn detail(self) -> u8 {
        self.0 & 0x1f
    }
    pub const fn as_raw(self) -> otCoapCode {
        self.0 as otCoapCode
    }
    pub const fn from_raw(raw: otCoapCode) -> Self {
        Self(raw as u8)
    }
}

/// Metadata for a CoAP request
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CoapRequest {
    pub coap_type: CoapType,
    pub code: CoapCode,
    pub message_id: u16,
    pub token: heapless::Vec<u8, COAP_MAX_TOKEN_LEN>,
    pub payload_len: usize,
    pub local: SocketAddrV6,
    pub peer: SocketAddrV6,
}

/// Metadata for a CoAP notification
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CoapNotification {
    pub coap_type: CoapType,
    pub code: CoapCode,
    pub observe_seq: Option<u32>,
    pub payload_len: usize,
}

pub struct OtCoapResources<
    const RESOURCES: usize = 4,
    const REQUESTS: usize = 2,
    const RX_BUF_SZ: usize = 512,
> {
    resource_slots: MaybeUninit<[CoapResourceCtx; RESOURCES]>,
    resource_buffers: MaybeUninit<[[u8; RX_BUF_SZ]; RESOURCES]>,
    request_slots: MaybeUninit<[CoapRequestCtx; REQUESTS]>,
    request_buffers: MaybeUninit<[[u8; RX_BUF_SZ]; REQUESTS]>,
    state: MaybeUninit<RefCell<OtCoapState<'static>>>,
}

impl<const RESOURCES: usize, const REQUESTS: usize, const RX_BUF_SZ: usize>
    OtCoapResources<RESOURCES, REQUESTS, RX_BUF_SZ>
{
    #[allow(clippy::declare_interior_mutable_const)]
    const INIT_RESOURCE: CoapResourceCtx = CoapResourceCtx::new();
    #[allow(clippy::declare_interior_mutable_const)]
    const INIT_REQUEST: CoapRequestCtx = CoapRequestCtx::new();
    #[allow(clippy::declare_interior_mutable_const)]
    const INIT_BUFFER: [u8; RX_BUF_SZ] = [0; RX_BUF_SZ];

    pub const fn new() -> Self {
        Self {
            resource_slots: MaybeUninit::uninit(),
            resource_buffers: MaybeUninit::uninit(),
            request_slots: MaybeUninit::uninit(),
            request_buffers: MaybeUninit::uninit(),
            state: MaybeUninit::uninit(),
        }
    }

    pub(crate) fn init(&mut self) -> &RefCell<OtCoapState<'static>> {
        self.resource_slots.write([Self::INIT_RESOURCE; RESOURCES]);
        self.resource_buffers.write([Self::INIT_BUFFER; RESOURCES]);
        self.request_slots.write([Self::INIT_REQUEST; REQUESTS]);
        self.request_buffers.write([Self::INIT_BUFFER; REQUESTS]);

        let resource_slots = unsafe { self.resource_slots.assume_init_mut() };
        let resource_slots: &'static mut [CoapResourceCtx] = unsafe {
            core::mem::transmute::<&mut [CoapResourceCtx], &'static mut [CoapResourceCtx]>(
                resource_slots.as_mut_slice(),
            )
        };

        let resource_buffers: &mut [[u8; RX_BUF_SZ]; RESOURCES] =
            unsafe { self.resource_buffers.assume_init_mut() };
        let resource_buffers: &'static mut [u8] = unsafe {
            core::slice::from_raw_parts_mut(
                resource_buffers.as_mut_ptr() as *mut _,
                RX_BUF_SZ * RESOURCES,
            )
        };

        let request_slots = unsafe { self.request_slots.assume_init_mut() };
        let request_slots: &'static mut [CoapRequestCtx] = unsafe {
            core::mem::transmute::<&mut [CoapRequestCtx], &'static mut [CoapRequestCtx]>(
                request_slots.as_mut_slice(),
            )
        };

        let request_buffers: &mut [[u8; RX_BUF_SZ]; REQUESTS] =
            unsafe { self.request_buffers.assume_init_mut() };
        let request_buffers: &'static mut [u8] = unsafe {
            core::slice::from_raw_parts_mut(
                request_buffers.as_mut_ptr() as *mut _,
                RX_BUF_SZ * REQUESTS,
            )
        };

        self.state.write(RefCell::new(OtCoapState {
            resource_slots,
            resource_buffers,
            request_slots,
            request_buffers,
            buf_len: RX_BUF_SZ,
        }));

        info!("OpenThread CoAP resources initialized");

        unsafe { self.state.assume_init_mut() }
    }
}

impl<const RESOURCES: usize, const REQUESTS: usize, const RX_BUF_SZ: usize> Default
    for OtCoapResources<RESOURCES, REQUESTS, RX_BUF_SZ>
{
    fn default() -> Self {
        Self::new()
    }
}

pub(crate) struct OtCoapState<'a> {
    pub(crate) resource_slots: &'a mut [CoapResourceCtx],
    pub(crate) resource_buffers: &'a mut [u8],
    pub(crate) request_slots: &'a mut [CoapRequestCtx],
    pub(crate) request_buffers: &'a mut [u8],
    pub(crate) buf_len: usize,
}

pub(crate) struct CoapResourceCtx {
    taken: bool,
    ot_resource: otCoapResource,
    uri_path: [u8; COAP_MAX_URI_LEN],
    rx: Signal<CoapRequest>,
}

impl CoapResourceCtx {
    pub(crate) const fn new() -> Self {
        Self {
            taken: false,
            ot_resource: otCoapResource {
                mUriPath: ptr::null(),
                mHandler: None,
                mContext: ptr::null_mut(),
                mNext: ptr::null_mut(),
            },
            uri_path: [0; COAP_MAX_URI_LEN],
            rx: Signal::new(),
        }
    }
}

pub struct CoapResource<'a> {
    ot: OpenThread<'a>,
    slot: usize,
}

impl<'a> CoapResource<'a> {
    pub fn register(ot: OpenThread<'a>, uri_path: &str) -> Result<Self, OtError> {
        if uri_path.len() >= COAP_MAX_URI_LEN || uri_path.as_bytes().contains(&0) {
            return Err(OtError::new(otError_OT_ERROR_INVALID_ARGS));
        }

        let slot = {
            let mut active = ot.activate();
            let state = active.state();
            let instance = state.ot.instance;
            let coap = state.coap()?;

            let slot = coap
                .resource_slots
                .iter()
                .position(|s| !s.taken)
                .ok_or(OtError::new(otError_OT_ERROR_NO_BUFS))?;

            let s = &mut coap.resource_slots[slot];
            s.taken = true;
            s.rx.reset();
            s.uri_path = [0; COAP_MAX_URI_LEN];
            s.uri_path[..uri_path.len()].copy_from_slice(uri_path.as_bytes());

            s.ot_resource = otCoapResource {
                mUriPath: s.uri_path.as_ptr() as *const _,
                mHandler: Some(Self::plat_c_request_handler),
                mContext: slot as *mut c_void,
                mNext: ptr::null_mut(),
            };

            unsafe {
                otCoapAddResource(instance, &mut s.ot_resource);
            }

            slot
        };

        info!("Registered CoAP resource in slot {}", slot);
        Ok(Self { ot, slot })
    }

    pub async fn recv(&self, buf: &mut [u8]) -> Result<(CoapRequest, usize), OtError> {
        let req = poll_fn(|cx| {
            self.ot.activate().state().coap()?.resource_slots[self.slot]
                .rx
                .poll_wait(cx)
                .map(Ok::<_, OtError>)
        })
        .await?;

        let mut active = self.ot.activate();
        let coap = active.state().coap()?;

        let offset = self.slot * coap.buf_len;
        let stored = &coap.resource_buffers[offset..offset + req.payload_len];

        let copy_len = req.payload_len.min(buf.len());
        buf[..copy_len].copy_from_slice(&stored[..copy_len]);

        Ok((req, copy_len))
    }

    extern "C" fn plat_c_request_handler(
        context: *mut c_void,
        message: *mut otMessage,
        message_info: *const otMessageInfo,
    ) {
        let slot: usize = context as usize;

        let mut ctx = OtContext::callback(ptr::null());
        let state = ctx.state();
        let instance = state.ot.instance;
        let Ok(coap) = state.coap() else {
            unreachable!();
        };

        let msg = unsafe { &*message };
        let info = unsafe { &*message_info };

        let coap_type_raw = unsafe { otCoapMessageGetType(msg) };
        let code_raw = unsafe { otCoapMessageGetCode(msg) };
        let message_id = unsafe { otCoapMessageGetMessageId(msg) };

        let token_len = unsafe { otCoapMessageGetTokenLength(msg) } as usize;
        let token_ptr = unsafe { otCoapMessageGetToken(msg) };
        let mut token = heapless::Vec::<u8, COAP_MAX_TOKEN_LEN>::new();
        if token_len <= COAP_MAX_TOKEN_LEN && !token_ptr.is_null() {
            let token_slice = unsafe { core::slice::from_raw_parts(token_ptr, token_len) };
            let _ = token.extend_from_slice(token_slice);
        }

        let total_len = unsafe { otMessageGetLength(msg) } as usize;
        let payload_off = unsafe { otMessageGetOffset(msg) } as usize;
        let payload_len = total_len.saturating_sub(payload_off);

        let slot_buf_len = coap.buf_len;
        let dst_off = slot * slot_buf_len;
        let dst = &mut coap.resource_buffers[dst_off..dst_off + slot_buf_len];

        let read_len = slot_buf_len.min(payload_len);
        let copied = if read_len > 0 {
            unsafe {
                otMessageRead(
                    msg,
                    payload_off as u16,
                    dst.as_mut_ptr() as *mut _,
                    read_len as u16,
                ) as usize
            }
        } else {
            0
        };

        if payload_len > slot_buf_len {
            warn!(
                "CoAP notification truncated. {} bytes were lost",
                payload_len - slot_buf_len
            );
        }

        let req = CoapRequest {
            coap_type: CoapType::from_raw(coap_type_raw),
            code: CoapCode::from_raw(code_raw),
            message_id,
            token,
            payload_len: copied,
            local: crate::to_sock_addr(&info.mSockAddr, info.mSockPort, 0),
            peer: crate::to_sock_addr(&info.mPeerAddr, info.mPeerPort, 0),
        };

        if coap_type_raw == otCoapType_OT_COAP_TYPE_CONFIRMABLE {
            let ack = unsafe { otCoapNewMessage(instance, ptr::null()) };
            if !ack.is_null() {
                let init_err = unsafe {
                    otCoapMessageInitResponse(
                        ack,
                        message,
                        otCoapType_OT_COAP_TYPE_ACKNOWLEDGMENT,
                        otCoapCode_OT_COAP_CODE_EMPTY,
                    )
                };
                if init_err == otError_OT_ERROR_NONE {
                    let send_err = unsafe {
                        otCoapSendResponseWithParameters(instance, ack, message_info, ptr::null())
                    };
                    if send_err != otError_OT_ERROR_NONE {
                        unsafe { otMessageFree(ack) };
                        warn!("Failed to send CoAP empty ACK: {}", send_err);
                    }
                } else {
                    unsafe { otMessageFree(ack) };
                    warn!("Failed to init CoAP empty ACK: {}", init_err);
                }
            } else {
                warn!("Failed to allocate CoAP empty ACK message");
            }
        }

        let slot_entry = &mut coap.resource_slots[slot];
        if !slot_entry.rx.signaled() {
            slot_entry.rx.signal(req);
        } else {
            warn!("Dropping CoAP request: previous req not yet consumed");
        }
    }
}

impl Drop for CoapResource<'_> {
    fn drop(&mut self) {
        let mut active = self.ot.activate();
        let state = active.state();
        let instance = state.ot.instance;
        let Ok(coap) = state.coap() else { return };

        let s = &mut coap.resource_slots[self.slot];
        unsafe {
            otCoapRemoveResource(instance, &mut s.ot_resource);
        }
        s.taken = false;
    }
}

type CoapNotificationResult = Result<CoapNotification, OtError>;

pub(crate) struct CoapRequestCtx {
    taken: bool,
    rx: Signal<CoapNotificationResult>,
}

impl CoapRequestCtx {
    pub(crate) const fn new() -> Self {
        Self {
            taken: false,
            rx: Signal::new(),
        }
    }
}

pub struct CoapObserve<'a> {
    ot: OpenThread<'a>,
    slot: usize,
}

impl<'a> CoapObserve<'a> {
    pub async fn recv(&self, buf: &mut [u8]) -> Result<(CoapNotification, usize), OtError> {
        let result = poll_fn(|cx| {
            self.ot.activate().state().coap()?.request_slots[self.slot]
                .rx
                .poll_wait(cx)
                .map(Ok::<_, OtError>)
        })
        .await?;
        let note = result?;

        let mut active = self.ot.activate();
        let coap = active.state().coap()?;

        let offset = self.slot * coap.buf_len;
        let stored = &coap.request_buffers[offset..offset + note.payload_len];

        let copy_len = note.payload_len.min(buf.len());
        buf[..copy_len].copy_from_slice(&stored[..copy_len]);

        Ok((note, copy_len))
    }

    extern "C" fn plat_c_response_handler(
        context: *mut c_void,
        message: *mut otMessage,
        _message_info: *const otMessageInfo,
        result: otError,
    ) {
        let slot: usize = context as usize;

        let mut ctx = OtContext::callback(ptr::null());
        let state = ctx.state();
        let Ok(coap) = state.coap() else {
            unreachable!();
        };

        if result != otError_OT_ERROR_NONE || message.is_null() {
            let entry = &mut coap.request_slots[slot];
            entry
                .rx
                .signal(Err(OtError::new(if result == otError_OT_ERROR_NONE {
                    crate::sys::otError_OT_ERROR_ABORT
                } else {
                    result
                })));
            return;
        }

        let msg = unsafe { &*message };

        let coap_type_raw = unsafe { otCoapMessageGetType(msg) };
        let code_raw = unsafe { otCoapMessageGetCode(msg) };

        let mut iterator = otCoapOptionIterator::default();
        let mut observe_seq: Option<u32> = None;
        let it_err = unsafe { otCoapOptionIteratorInit(&mut iterator, msg) };
        if it_err == otError_OT_ERROR_NONE {
            let opt = unsafe {
                otCoapOptionIteratorGetFirstOptionMatching(
                    &mut iterator,
                    otCoapOptionType_OT_COAP_OPTION_OBSERVE as u16,
                )
            };
            if !opt.is_null() {
                let mut value: u64 = 0;
                let v_err =
                    unsafe { otCoapOptionIteratorGetOptionUintValue(&mut iterator, &mut value) };
                if v_err == otError_OT_ERROR_NONE {
                    observe_seq = Some(value as u32);
                }
            }
        }

        let total_len = unsafe { otMessageGetLength(msg) } as usize;
        let payload_off = unsafe { otMessageGetOffset(msg) } as usize;
        let payload_len = total_len.saturating_sub(payload_off);

        let slot_buf_len = coap.buf_len;
        let dst_off = slot * slot_buf_len;
        let dst = &mut coap.request_buffers[dst_off..dst_off + slot_buf_len];

        let read_len = slot_buf_len.min(payload_len);
        let copied = if read_len > 0 {
            unsafe {
                otMessageRead(
                    msg,
                    payload_off as u16,
                    dst.as_mut_ptr() as *mut _,
                    read_len as u16,
                ) as usize
            }
        } else {
            0
        };

        if payload_len > slot_buf_len {
            warn!(
                "CoAP notification truncated. {} bytes were lost",
                payload_len - slot_buf_len
            );
        }

        let note = CoapNotification {
            coap_type: CoapType::from_raw(coap_type_raw),
            code: CoapCode::from_raw(code_raw),
            observe_seq,
            payload_len: copied,
        };

        let entry = &mut coap.request_slots[slot];
        entry.rx.signal(Ok(note));
    }
}

impl Drop for CoapObserve<'_> {
    fn drop(&mut self) {
        let mut active = self.ot.activate();
        let Ok(coap) = active.state().coap() else {
            return;
        };
        coap.request_slots[self.slot].taken = false;
        coap.request_slots[self.slot].rx.reset();
    }
}

impl<'a> OpenThread<'a> {
    pub fn coap_start(&self, port: u16) -> Result<(), OtError> {
        let mut active = self.activate();
        let state = active.state();
        let _ = state.coap()?;
        ot!(unsafe { otCoapStart(state.ot.instance, port) })
    }

    pub fn coap_stop(&self) -> Result<(), OtError> {
        let mut active = self.activate();
        let state = active.state();
        let _ = state.coap()?;
        ot!(unsafe { otCoapStop(state.ot.instance) })
    }

    pub fn coap_respond(
        &self,
        request: &CoapRequest,
        code: CoapCode,
        payload: &[u8],
    ) -> Result<(), OtError> {
        let mut active = self.activate();
        let state = active.state();
        let instance = state.ot.instance;
        let _ = state.coap()?;

        let msg = unsafe { otCoapNewMessage(instance, ptr::null()) };
        if msg.is_null() {
            return Err(OtError::new(otError_OT_ERROR_NO_BUFS));
        }

        unsafe {
            otCoapMessageInit(msg, otCoapType_OT_COAP_TYPE_NON_CONFIRMABLE, code.as_raw());
        }

        if !request.token.is_empty() {
            let res = unsafe {
                otCoapMessageSetToken(msg, request.token.as_ptr(), request.token.len() as u8)
            };
            if res != otError_OT_ERROR_NONE {
                unsafe { otMessageFree(msg) };
                return Err(OtError::new(res));
            }
        }

        if !payload.is_empty() {
            let res = unsafe { otCoapMessageSetPayloadMarker(msg) };
            if res != otError_OT_ERROR_NONE {
                unsafe { otMessageFree(msg) };
                return Err(OtError::new(res));
            }
            let res =
                unsafe { otMessageAppend(msg, payload.as_ptr() as *const _, payload.len() as u16) };
            if res != otError_OT_ERROR_NONE {
                unsafe { otMessageFree(msg) };
                return Err(OtError::new(res));
            }
        }

        let mut info = otMessageInfo::default();
        info.mSockAddr = otIp6Address {
            mFields: otIp6Address__bindgen_ty_1 {
                m8: request.local.ip().octets(),
            },
        };
        info.mSockPort = request.local.port();
        info.mPeerAddr = otIp6Address {
            mFields: otIp6Address__bindgen_ty_1 {
                m8: request.peer.ip().octets(),
            },
        };
        info.mPeerPort = request.peer.port();
        info.mHopLimit = 0;

        let send_res =
            unsafe { otCoapSendResponseWithParameters(instance, msg, &info, ptr::null()) };
        if send_res != otError_OT_ERROR_NONE {
            unsafe { otMessageFree(msg) };
            return Err(OtError::new(send_res));
        }

        trace!("Transmitted CoAP response: {}", Bytes(payload));
        Ok(())
    }

    pub fn coap_observe(
        &self,
        server: &SocketAddrV6,
        uri_path: &str,
    ) -> Result<CoapObserve<'a>, OtError> {
        if uri_path.is_empty() || uri_path.as_bytes().contains(&0) {
            return Err(OtError::new(otError_OT_ERROR_INVALID_ARGS));
        }

        let mut path_buf = [0u8; COAP_MAX_URI_LEN];
        if uri_path.len() >= path_buf.len() {
            return Err(OtError::new(otError_OT_ERROR_INVALID_ARGS));
        }
        path_buf[..uri_path.len()].copy_from_slice(uri_path.as_bytes());

        let ot_for_handle = self.clone();
        let mut active = self.activate();
        let state = active.state();
        let instance = state.ot.instance;
        let coap = state.coap()?;

        let slot = coap
            .request_slots
            .iter()
            .position(|s| !s.taken)
            .ok_or(OtError::new(otError_OT_ERROR_NO_BUFS))?;

        coap.request_slots[slot].taken = true;
        coap.request_slots[slot].rx.reset();

        let msg = unsafe { otCoapNewMessage(instance, ptr::null()) };
        if msg.is_null() {
            coap.request_slots[slot].taken = false;
            return Err(OtError::new(otError_OT_ERROR_NO_BUFS));
        }

        unsafe {
            otCoapMessageInit(
                msg,
                otCoapType_OT_COAP_TYPE_CONFIRMABLE,
                CoapCode::GET.as_raw(),
            );
            otCoapMessageGenerateToken(msg, OT_COAP_DEFAULT_TOKEN_LENGTH as u8);
        }

        let err = unsafe { otCoapMessageAppendObserveOption(msg, 0) };
        if err != otError_OT_ERROR_NONE {
            unsafe { otMessageFree(msg) };
            coap.request_slots[slot].taken = false;
            return Err(OtError::new(err));
        }

        let err = unsafe {
            otCoapMessageAppendUriPathOptions(msg, path_buf.as_ptr() as *const core::ffi::c_char)
        };
        if err != otError_OT_ERROR_NONE {
            unsafe { otMessageFree(msg) };
            coap.request_slots[slot].taken = false;
            return Err(OtError::new(err));
        }

        let mut info = otMessageInfo::default();
        info.mPeerAddr = otIp6Address {
            mFields: otIp6Address__bindgen_ty_1 {
                m8: server.ip().octets(),
            },
        };
        info.mPeerPort = server.port();
        info.mHopLimit = 0;

        let send_err = unsafe {
            otCoapSendRequestWithParameters(
                instance,
                msg,
                &info,
                Some(CoapObserve::plat_c_response_handler),
                slot as *mut c_void,
                ptr::null(),
            )
        };
        if send_err != otError_OT_ERROR_NONE {
            unsafe { otMessageFree(msg) };
            coap.request_slots[slot].taken = false;
            return Err(OtError::new(send_err));
        }

        info!("Registered CoAP observe in slot {} for {}", slot, uri_path);

        Ok(CoapObserve {
            ot: ot_for_handle,
            slot,
        })
    }
}
