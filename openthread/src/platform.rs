//! An internal module that does the plumbing from the OpenThread C "Platform" API callbacks to Rust

use core::cell::{Cell, UnsafeCell};
use core::ffi::{c_char, CStr};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;

use openthread_sys::otError_OT_ERROR_NONE;

#[cfg(feature = "_csl")]
use crate::radio::MacKeys;
#[cfg(feature = "csl-receiver")]
use crate::sys::otExtAddress;
use crate::sys::{otError, otInstance, otLogLevel, otLogRegion, otRadioCaps, otRadioFrame};
#[cfg(feature = "_csl")]
use crate::sys::{otMacKeyMaterial, otRadioKeyType, otRadioKeyType_OT_KEY_TYPE_LITERAL_KEY};
use crate::{IntoOtCode, OtActiveState, OtContext};

/// A hack so that we can store a mutable reference to the active state in a global static variable
/// without any explicit synchronization
pub(crate) struct SyncUnsafeCell<T>(pub UnsafeCell<T>);

unsafe impl<T> Sync for SyncUnsafeCell<T> {}

/// A global reference counter for OpenThread instances
pub(crate) static OT_REFCNT: Mutex<CriticalSectionRawMutex, Cell<usize>> = Mutex::new(Cell::new(0));

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

#[cfg(feature = "csl-receiver")]
#[no_mangle]
extern "C" fn otPlatAlarmMicroGetNow() -> u32 {
    OtContext::callback(core::ptr::null()).plat_now_micros()
}

#[cfg(feature = "csl-receiver")]
#[no_mangle]
extern "C" fn otPlatAlarmMicroStartAt(instance: *mut otInstance, at0: u32, adt: u32) {
    OtContext::callback(instance).plat_alarm_micro_set(at0, adt);
}

#[cfg(feature = "csl-receiver")]
#[no_mangle]
extern "C" fn otPlatAlarmMicroStop(instance: *const otInstance) {
    OtContext::callback(instance).plat_alarm_micro_clear();
}

// --- CSL (Synchronized Sleepy End Device) and enhanced-ACK security ---

#[cfg(feature = "_csl")]
#[no_mangle]
extern "C" fn otPlatRadioGetNow(instance: *const otInstance) -> u64 {
    OtContext::callback(instance).plat_radio_now()
}

#[cfg(feature = "csl-receiver")]
#[no_mangle]
extern "C" fn otPlatRadioReceiveAt(
    instance: *const otInstance,
    channel: u8,
    start: u32,
    duration: u32,
) -> otError {
    OtContext::callback(instance)
        .plat_radio_receive_at(channel, start, duration)
        .into_ot_code()
}

#[cfg(feature = "csl-receiver")]
#[no_mangle]
extern "C" fn otPlatRadioEnableCsl(
    instance: *const otInstance,
    csl_period: u32,
    short_addr: u16,
    ext_addr: *const otExtAddress,
) -> otError {
    // Unlike the source-match entries, OpenThread hands this one out in
    // big-endian byte order: the numeric EUI-64, as `ext_address()` reads it.
    let ext_addr = unsafe { ext_addr.as_ref() }.map(|addr| u64::from_be_bytes(addr.m8));

    OtContext::callback(instance)
        .plat_radio_enable_csl(csl_period, short_addr, ext_addr)
        .into_ot_code()
}

#[cfg(feature = "csl-receiver")]
#[no_mangle]
extern "C" fn otPlatRadioUpdateCslSampleTime(instance: *const otInstance, csl_sample_time: u32) {
    OtContext::callback(instance).plat_radio_update_csl_sample_time(csl_sample_time);
}

#[cfg(feature = "ftd")]
#[no_mangle]
extern "C" fn otPlatRadioGetBusSpeed(instance: *const otInstance) -> u32 {
    OtContext::callback(instance).plat_radio_bus_speed()
}

#[cfg(feature = "ftd")]
#[no_mangle]
extern "C" fn otPlatRadioGetBusLatency(instance: *const otInstance) -> u32 {
    OtContext::callback(instance).plat_radio_bus_latency()
}

#[cfg(feature = "_csl")]
#[no_mangle]
extern "C" fn otPlatRadioGetCslAccuracy(instance: *const otInstance) -> u8 {
    OtContext::callback(instance).plat_radio_csl_accuracy()
}

#[cfg(feature = "_csl")]
#[no_mangle]
extern "C" fn otPlatRadioGetCslUncertainty(instance: *const otInstance) -> u8 {
    OtContext::callback(instance).plat_radio_csl_uncertainty()
}

#[cfg(feature = "_csl")]
#[no_mangle]
extern "C" fn otPlatRadioSetMacKey(
    instance: *const otInstance,
    key_id_mode: u8,
    key_id: u8,
    prev_key: *const otMacKeyMaterial,
    curr_key: *const otMacKeyMaterial,
    next_key: *const otMacKeyMaterial,
    key_type: otRadioKeyType,
) {
    // Only literal keys can be handed to a radio; key references (PSA) are not
    // supported by this crate's OpenThread build.
    let literal = |key: *const otMacKeyMaterial| -> Option<[u8; 16]> {
        (key_type == otRadioKeyType_OT_KEY_TYPE_LITERAL_KEY)
            .then(|| unsafe { key.as_ref() }.map(|key| unsafe { key.mKeyMaterial.mKey.m8 }))
            .flatten()
    };

    let keys = match (literal(prev_key), literal(curr_key), literal(next_key)) {
        (Some(prev), Some(curr), Some(next)) => Some(MacKeys {
            // OpenThread passes the mode as it sits in the frame's security
            // control byte (`kKeyIdMode1 == 1 << 3`); the radio wants 0..=3.
            key_id_mode: key_id_mode >> 3,
            key_id,
            prev,
            curr,
            next,
        }),
        _ => None,
    };

    OtContext::callback(instance).plat_radio_set_mac_keys(keys);
}

#[cfg(feature = "_csl")]
#[no_mangle]
extern "C" fn otPlatRadioSetMacFrameCounter(instance: *const otInstance, frame_counter: u32) {
    OtContext::callback(instance).plat_radio_set_mac_frame_counter(frame_counter, false);
}

#[cfg(feature = "_csl")]
#[no_mangle]
extern "C" fn otPlatRadioSetMacFrameCounterIfLarger(
    instance: *const otInstance,
    frame_counter: u32,
) {
    OtContext::callback(instance).plat_radio_set_mac_frame_counter(frame_counter, true);
}

#[no_mangle]
extern "C" fn otPlatRadioGetIeeeEui64(instance: *const otInstance, mac: *mut u8) {
    let mac = unwrap!(unsafe { core::ptr::slice_from_raw_parts_mut(mac, 8).as_mut() });

    OtContext::callback(instance).plat_radio_ieee_eui64(unwrap!(mac.try_into()));
}

#[no_mangle]
extern "C" fn otPlatRadioGetCaps(instance: *const otInstance) -> otRadioCaps {
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
extern "C" fn otPlatRadioGetTransmitPower(instance: *const otInstance, power: *mut i8) -> otError {
    OtContext::callback(instance)
        .plat_radio_get_transmit_power(unsafe { power.as_mut() })
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatRadioSetTransmitPower(instance: *const otInstance, power: i8) -> otError {
    OtContext::callback(instance)
        .plat_radio_set_transmit_power(power)
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatRadioGetCcaEnergyDetectThreshold(
    instance: *const otInstance,
    threshold: *mut i8,
) -> otError {
    OtContext::callback(instance)
        .plat_radio_get_cca_energy_detect_threshold(unsafe { threshold.as_mut() })
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatRadioSetCcaEnergyDetectThreshold(
    instance: *const otInstance,
    threshold: i8,
) -> otError {
    OtContext::callback(instance)
        .plat_radio_set_cca_energy_detect_threshold(threshold)
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatRadioGetRssi(instance: *const otInstance) -> i8 {
    OtContext::callback(instance).plat_radio_get_rssi()
}

#[no_mangle]
extern "C" fn otPlatRadioGetReceiveSensitivity(instance: *const otInstance) -> i8 {
    OtContext::callback(instance).plat_radio_receive_sensitivity()
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
    OtContext::callback(instance).plat_radio_set_extended_address(u64::from_le_bytes(unwrap!(
        unsafe { core::slice::from_raw_parts(address, 8) }.try_into()
    )));
}

#[no_mangle]
extern "C" fn otPlatRadioSetShortAddress(instance: *const otInstance, address: u16) {
    OtContext::callback(instance).plat_radio_set_short_address(address);
}

// Alternate short address (FTD, Thread >= 1.2).
//
// During a child-to-router role transition an FTD is briefly reachable at BOTH
// its old (child) RLOC16 and its new (router) RLOC16. OpenThread hands the radio
// the old address here so frames addressed to it keep being received for a short
// window (`kAlternateRloc16Timeout`, ~8s), after which the stack clears it (calls
// this with `OT_RADIO_INVALID_SHORT_ADDR`). It is invoked unconditionally in the
// FTD `Mac` path (never on MTD, where `--gc-sections` drops it).
#[no_mangle]
extern "C" fn otPlatRadioSetAlternateShortAddress(instance: *const otInstance, address: u16) {
    OtContext::callback(instance).plat_radio_set_alternate_short_address(address);
}

#[no_mangle]
extern "C" fn otPlatRadioSetPanId(instance: *const otInstance, pan_id: u16) {
    OtContext::callback(instance).plat_radio_set_pan_id(pan_id);
}

#[no_mangle]
extern "C" fn otPlatRadioSetRxOnWhenIdle(instance: *const otInstance, enable: bool) {
    OtContext::callback(instance).plat_radio_set_rx_on_when_idle(enable);
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

// --- Source-address match (FTD only) ---
//
// Only called when an FTD `libopenthread-ftd.a` is linked;
// on MTD they are never called and are dropped by `--gc-sections`.

#[no_mangle]
extern "C" fn otPlatRadioEnableSrcMatch(instance: *const otInstance, enable: bool) {
    OtContext::callback(instance).plat_radio_enable_src_match(enable);
}

#[no_mangle]
extern "C" fn otPlatRadioAddSrcMatchShortEntry(
    instance: *const otInstance,
    short_address: u16,
) -> otError {
    OtContext::callback(instance)
        .plat_radio_add_src_match_short(short_address)
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatRadioAddSrcMatchExtEntry(
    instance: *const otInstance,
    ext_address: *const u8,
) -> otError {
    // Byte order as for `otPlatRadioSetExtendedAddress` above.
    let ext_address = u64::from_le_bytes(unwrap!(unsafe {
        core::slice::from_raw_parts(ext_address, 8)
    }
    .try_into()));

    OtContext::callback(instance)
        .plat_radio_add_src_match_ext(ext_address)
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatRadioClearSrcMatchShortEntry(
    instance: *const otInstance,
    short_address: u16,
) -> otError {
    OtContext::callback(instance)
        .plat_radio_clear_src_match_short(short_address)
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatRadioClearSrcMatchExtEntry(
    instance: *const otInstance,
    ext_address: *const u8,
) -> otError {
    let ext_address = u64::from_le_bytes(unwrap!(unsafe {
        core::slice::from_raw_parts(ext_address, 8)
    }
    .try_into()));

    OtContext::callback(instance)
        .plat_radio_clear_src_match_ext(ext_address)
        .into_ot_code()
}

#[no_mangle]
extern "C" fn otPlatRadioClearSrcMatchShortEntries(instance: *const otInstance) {
    OtContext::callback(instance).plat_radio_clear_src_match_short_entries();
}

#[no_mangle]
extern "C" fn otPlatRadioClearSrcMatchExtEntries(instance: *const otInstance) {
    OtContext::callback(instance).plat_radio_clear_src_match_ext_entries();
}

// Factory diagnostics (`OT_DIAGNOSTIC` builds)
//
// The exact minimal surface the upstream simulation platform provides -
// a mode flag, plus no-op acknowledgments of the channel/power hints and of the received-frame extension hook.

static DIAG_MODE: Mutex<CriticalSectionRawMutex, Cell<bool>> = Mutex::new(Cell::new(false));

#[no_mangle]
extern "C" fn otPlatDiagModeSet(mode: bool) {
    DIAG_MODE.lock(|mode_cell| mode_cell.set(mode));
}

#[no_mangle]
extern "C" fn otPlatDiagModeGet() -> bool {
    DIAG_MODE.lock(|mode_cell| mode_cell.get())
}

#[no_mangle]
extern "C" fn otPlatDiagSetOutputCallback(
    _instance: *mut otInstance,
    _callback: *mut core::ffi::c_void,
    _context: *mut core::ffi::c_void,
) {
}

#[no_mangle]
extern "C" fn otPlatDiagChannelSet(_channel: u8) {}

#[no_mangle]
extern "C" fn otPlatDiagTxPowerSet(_power: i8) {}

#[no_mangle]
extern "C" fn otPlatDiagRadioReceived(
    _instance: *mut otInstance,
    _frame: *mut otRadioFrame,
    _error: otError,
) {
}

// NOTE: `otPlatCryptoPbkdf2GenerateKey` (PSKc derivation, FTD commissioning)
// is deliberately NOT defined here: OpenThread's `crypto_platform.cpp` ships
// a working `OT_TOOL_WEAK` implementation for every FTD build under both of
// its crypto-lib variants, and a strong Rust stub would shadow it.

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
    {
        if let Ok(str) = unsafe { CStr::from_ptr(str) }.to_str() {
            match level {
                0 /*NONE*/ => {
                    // Level-"none" records are not ordinary logs:
                    // OpenThread emits them only for content that must always surface - the MeshCoP certification dumps of reference-device
                    // builds (`DumpCert`: `[THCI]` headers plus hex lines). The distinct prefix lets embedders that must expose these on their
                    // console (test DUTs driven by the upstream cert harness) route on it.
                    info!("[OpenThread-OUT] {}", str);
                }
                1 /*CRIT*/ => {
                    error!("[OpenThread] {}", str);
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

// Other C functions which might generally not be supported by MCU ROMs or by - say - `tinyrlibc`.
//
// IMPORTANT: these MUST match the C `<ctype.h>` ABI exactly — `int isXXX(int c)`.
// C callers read a full `int` from the return register, so a narrower Rust
// return type (e.g. `bool`) leaves its upper bytes undefined on targets that
// do not zero-extend narrow returns (x86-64).

#[no_mangle]
extern "C" fn iscntrl(c: core::ffi::c_int) -> core::ffi::c_int {
    // Control chars: 0x00..=0x1F and 0x7F (DEL).
    ((0..0x20).contains(&c) || c == 0x7f) as core::ffi::c_int
}

#[no_mangle]
extern "C" fn isprint(c: core::ffi::c_int) -> core::ffi::c_int {
    // Printable: space (0x20) through '~' (0x7E).
    (0x20..0x7f).contains(&c) as core::ffi::c_int
}

#[cfg(feature = "isupper")]
#[no_mangle]
extern "C" fn isupper(c: core::ffi::c_int) -> core::ffi::c_int {
    (('A' as core::ffi::c_int..='Z' as core::ffi::c_int).contains(&c)) as core::ffi::c_int
}
