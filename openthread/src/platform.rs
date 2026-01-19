//! An internal module that does the plumbing from the OpenThread C "Platform" API callbacks to Rust

use core::cell::UnsafeCell;
use core::ffi::{c_char, CStr};

use portable_atomic::AtomicUsize;

use openthread_sys::otError_OT_ERROR_NONE;

use crate::sys::{otError, otInstance, otLogLevel, otLogRegion, otRadioFrame};
use crate::{IntoOtCode, OtActiveState, OtContext};

/// A hack so that we can store a mutable reference to the active state in a global static variable
/// without any explicit synchronization
pub(crate) struct SyncUnsafeCell<T>(pub UnsafeCell<T>);

unsafe impl<T> Sync for SyncUnsafeCell<T> {}

/// A global reference counter for OpenThread instances
pub(crate) static OT_REFCNT: AtomicUsize = AtomicUsize::new(0);

/// A static, mutable global state that allows OpenThnread to call us back via its `otPlat*` functions
/// Look at `OtActiveState` and `OpenThread` for more information as to when this variable is set and unset
pub(crate) static OT_ACTIVE_STATE: SyncUnsafeCell<Option<OtActiveState<'static>>> =
    SyncUnsafeCell(UnsafeCell::new(None));

#[no_mangle]
extern "C" fn otPlatReset(instance: *const u8) -> otError {
    OtContext::callback(instance as *const _)
        .plat_reset()
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatEntropyGet(output: *mut u8, len: u16) -> otError {
    OtContext::callback(core::ptr::null_mut())
        .plat_entropy_get(unsafe { core::slice::from_raw_parts_mut(output, len as usize) })
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otTaskletsSignalPending(instance: *mut otInstance) {
    OtContext::callback(instance).plat_tasklets_signal_pending();
}

#[no_mangle]
extern "C" fn otPlatAlarmMilliGetNow(instance: *const otInstance) -> u32 {
    OtContext::callback(instance).plat_now()
}

#[no_mangle]
extern "C" fn otPlatAlarmMilliStartAt(instance: *mut otInstance, at0: u32, adt: u32) -> otError {
    OtContext::callback(instance)
        .plat_alarm_set(at0, adt)
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatAlarmMilliStop(instance: *const otInstance) -> otError {
    OtContext::callback(instance)
        .plat_alarm_clear()
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatRadioGetIeeeEui64(instance: *const otInstance, mac: *mut u8) {
    let mac = unwrap!(unsafe { core::ptr::slice_from_raw_parts_mut(mac, 8).as_mut() });

    OtContext::callback(instance).plat_radio_ieee_eui64(unwrap!(mac.try_into()));
}

#[no_mangle]
extern "C" fn otPlatRadioGetCaps(instance: *const otInstance) -> u8 {
    OtContext::callback(instance).plat_radio_caps()
}

#[no_mangle]
extern "C" fn otPlatRadioGetTransmitBuffer(instance: *const otInstance) -> *mut otRadioFrame {
    OtContext::callback(instance).plat_radio_transmit_buffer()
}

#[no_mangle]
extern "C" fn otPlatRadioEnable(instance: *const otInstance) -> otError {
    OtContext::callback(instance)
        .plat_radio_enable()
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatRadioSleep(instance: *const otInstance) -> otError {
    OtContext::callback(instance)
        .plat_radio_sleep()
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatRadioDisable(instance: *const otInstance) -> otError {
    OtContext::callback(instance)
        .plat_radio_disable()
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatRadioSetPromiscuous(instance: *const otInstance, enable: bool) {
    OtContext::callback(instance).plat_radio_set_promiscuous(enable)
}

#[no_mangle]
extern "C" fn otPlatRadioGetRssi(instance: *const otInstance) -> i8 {
    OtContext::callback(instance).plat_radio_get_rssi()
}

#[no_mangle]
extern "C" fn otPlatRadioGetReceiveSensitivity(instance: *const otInstance) -> i8 {
    OtContext::callback(instance).plat_radio_receive_sensititivy()
}

#[no_mangle]
extern "C" fn otPlatRadioIsEnabled(instance: *mut otInstance) -> bool {
    OtContext::callback(instance).plat_radio_is_enabled()
}

#[no_mangle]
extern "C" fn otPlatRadioEnergyScan(
    instance: *const otInstance,
    channel: u8,
    duration: u16,
) -> otError {
    OtContext::callback(instance)
        .plat_radio_energy_scan(channel, duration)
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatRadioGetPromiscuous(instance: *const otInstance) -> bool {
    OtContext::callback(instance).plat_radio_get_promiscuous()
}

#[no_mangle]
extern "C" fn otPlatRadioSetExtendedAddress(instance: *const otInstance, address: *const u8) {
    OtContext::callback(instance).plat_radio_set_extended_address(u64::from_be_bytes(unwrap!(
        unsafe { core::slice::from_raw_parts(address, 8) }.try_into()
    )));
}

#[no_mangle]
extern "C" fn otPlatRadioSetShortAddress(instance: *const otInstance, address: u16) {
    OtContext::callback(instance).plat_radio_set_short_address(address);
}

#[no_mangle]
extern "C" fn otPlatRadioSetPanId(instance: *const otInstance, pan_id: u16) {
    OtContext::callback(instance).plat_radio_set_pan_id(pan_id);
}

#[no_mangle]
extern "C" fn otPlatRadioTransmit(
    instance: *const otInstance,
    frame: *const otRadioFrame,
) -> otError {
    OtContext::callback(instance)
        .plat_radio_transmit(unsafe { &*frame })
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatRadioReceive(instance: *mut otInstance, channel: u8) -> otError {
    OtContext::callback(instance)
        .plat_radio_receive(channel)
        .into_ot_code()
}

#[no_mangle]
#[cfg(feature = "full-thread-device")]
extern "C" fn otPlatRadioEnableSrcMatch(instance: *mut otInstance, enable: bool) {
    OtContext::callback(instance).plat_radio_enable_src_match(enable);
}

#[no_mangle]
extern "C" fn otPlatSettingsInit(
    instance: *mut otInstance,
    sensitive_keys: *const u16,
    sensitive_keys_length: u16,
) {
    OtContext::callback(instance).plat_settings_init(unsafe {
        core::slice::from_raw_parts(sensitive_keys, sensitive_keys_length as _)
    })
}

#[no_mangle]
extern "C" fn otPlatSettingsDeinit(instance: *mut otInstance) {
    OtContext::callback(instance).plat_settings_deinit()
}

#[no_mangle]
extern "C" fn otPlatSettingsGet(
    instance: *mut otInstance,
    key: u16,
    index: core::ffi::c_int,
    value: *mut u8,
    value_length: *mut u16,
) -> otError {
    let value_length = unsafe { &mut *value_length };

    match OtContext::callback(instance).plat_settings_get(key, index, unsafe {
        core::slice::from_raw_parts_mut(value, *value_length as _)
    }) {
        Ok(len) => {
            *value_length = len as _;
            otError_OT_ERROR_NONE
        }
        Err(e) => e.into_inner(),
    }
}

#[no_mangle]
extern "C" fn otPlatSettingsSet(
    instance: *mut otInstance,
    key: u16,
    value: *const u8,
    value_length: u16,
) -> otError {
    OtContext::callback(instance)
        .plat_settings_set(key, unsafe {
            core::slice::from_raw_parts(value, value_length as _)
        })
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatSettingsAdd(
    instance: *mut otInstance,
    key: u16,
    value: *const u8,
    value_length: u16,
) -> otError {
    OtContext::callback(instance)
        .plat_settings_add(key, unsafe {
            core::slice::from_raw_parts(value, value_length as _)
        })
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatSettingsDelete(
    instance: *mut otInstance,
    key: u16,
    index: core::ffi::c_int,
) -> otError {
    OtContext::callback(instance)
        .plat_settings_delete(key, index)
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatSettingsWipe(instance: *mut otInstance) {
    OtContext::callback(instance).plat_settings_wipe()
}

/// NOTE:
/// While the correct signature should be something like:
/// ```ignore
/// extern "C" fn otPlatLog(
///     _level: otLogLevel,
///     _region: otLogRegion,
///     _format: *const c_char,
///     _args: ...
/// ) -> otError {
///     todo!()
/// }
/// ```
///
/// ... varargs are not yet stable in Rust, so we cannot express this.
///
/// Fortunately, looking here: https://github.com/openthread/openthread/blob/31f2897951c9dfd89364121f0581622416e77a7b/src/core/common/log.cpp#L131
/// ... it seems (at least for now) that the "varargs" aspect of `otPlatLog` is not used on the OpenThread C++ side.
///
/// So - while risky - until the above OpenThread C++ code stays unchanged - we can get away with the function signature below.
#[no_mangle]
extern "C" fn otPlatLog(
    level: otLogLevel,
    _region: otLogRegion,
    _format: *const c_char,
    str: *const c_char,
) -> otError {
    if level > 0 {
        if let Ok(str) = unsafe { CStr::from_ptr(str) }.to_str() {
            match level {
                1 /*CRIT*/ => {
                    info!("[OpenThread] {}", str);
                }
                2 /*WARN*/ => {
                    warn!("[OpenThread] {}", str);
                }
                3 /*NOTE*/ => {
                    info!("[OpenThread] {}", str);
                }
                4 /*INFO*/ => {
                    debug!("[OpenThread] {}", str);
                }
                _ /*DEBG*/ => {
                    trace!("[OpenThread] {}", str);
                }
            }
        }
    }

    otError_OT_ERROR_NONE
}

// Other C functions which might generally not be supported by MCU ROMs or by - say - `tinyrlibc`

#[no_mangle]
extern "C" fn iscntrl(v: u32) -> bool {
    v < ' ' as u32
}

#[no_mangle]
extern "C" fn isprint(v: u32) -> bool {
    v >= ' ' as u32 && v <= 127
}

#[cfg(feature = "isupper")]
#[no_mangle]
extern "C" fn isupper(v: u32) -> bool {
    v >= 'A' as u32 && v <= 'Z' as u32
}
