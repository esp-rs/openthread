//! Joiner API: commission this device onto a Thread network via the Thread
//! commissioning protocol (MeshCoP), using a pre-shared joiner key (PSKd).
//!
//! This is the *Thread-native* onboarding flow: a Commissioner active on the
//! target network (e.g. a border router's web UI, or `ot-cli`'s `commissioner`
//! with this device's EUI-64 and PSKd) admits the joiner, which then receives
//! the network credentials over a DTLS session. It is unrelated to (and not
//! needed for) Matter commissioning, where the operational dataset is
//! provisioned out-of-band (e.g. over BLE).

use core::ffi::c_void;
use core::future::poll_fn;

use crate::sys::{
    otError, otError_OT_ERROR_INVALID_ARGS, otInstance, otJoinerStart, otJoinerStop,
    OT_JOINER_MAX_PSKD_LENGTH, OT_PROVISIONING_URL_MAX_SIZE,
};
use crate::{ot, OpenThread, OtContext, OtError};

impl OpenThread<'_> {
    /// Join a Thread network using the Thread commissioning protocol
    /// (`otJoinerStart`): discover nearby networks with an active Commissioner
    /// admitting this device, establish the (EC-JPAKE) DTLS session using the
    /// pre-shared joiner key `pskd`, and receive the network credentials.
    ///
    /// On success, the received Active Operational Dataset has been stored in
    /// the stack; attach to the network by enabling Thread
    /// ([`OpenThread::enable_thread`]).
    ///
    /// Prerequisites: the IPv6 interface must be up
    /// ([`OpenThread::enable_ipv6`]) and the Thread protocol must be disabled,
    /// otherwise an `INVALID_STATE` error is reported.
    ///
    /// Arguments:
    /// - `pskd`: the pre-shared joiner key, as printed on the device / its
    ///   packaging: 6 to 32 uppercase alphanumeric characters excluding
    ///   `I`, `O`, `Q` and `Z` (validated by OpenThread).
    /// - `provisioning_url`: an optional provisioning URL (max 64 bytes),
    ///   advertised to the Commissioner.
    ///
    /// Dropping the returned future cancels the join in progress
    /// (`otJoinerStop`).
    pub async fn join(&self, pskd: &str, provisioning_url: Option<&str>) -> Result<(), OtError> {
        // NUL-terminated stack copies of the string arguments
        // (`otJoinerStart` takes C strings).
        let mut pskd_buf = [0u8; OT_JOINER_MAX_PSKD_LENGTH as usize + 1];
        let mut url_buf = [0u8; OT_PROVISIONING_URL_MAX_SIZE as usize + 1];

        if pskd.len() >= pskd_buf.len() {
            return Err(OtError::new(otError_OT_ERROR_INVALID_ARGS));
        }
        pskd_buf[..pskd.len()].copy_from_slice(pskd.as_bytes());

        let url_ptr = if let Some(url) = provisioning_url {
            if url.len() >= url_buf.len() {
                return Err(OtError::new(otError_OT_ERROR_INVALID_ARGS));
            }
            url_buf[..url.len()].copy_from_slice(url.as_bytes());
            url_buf.as_ptr().cast()
        } else {
            core::ptr::null()
        };

        {
            let mut ot = self.activate();
            let state = ot.state();

            // Clear any stale completion left over from a prior join whose
            // future was dropped after the callback signalled but before the
            // wait below consumed it.
            state.ot.join_done.reset();

            ot!(unsafe {
                otJoinerStart(
                    state.ot.instance,
                    pskd_buf.as_ptr().cast(),
                    url_ptr,
                    // Vendor name/model/sw-version/data: optional, not sent.
                    core::ptr::null(),
                    core::ptr::null(),
                    core::ptr::null(),
                    core::ptr::null(),
                    Some(Self::plat_c_joiner_callback),
                    state.ot.instance as *mut _,
                )
            })?;
        }

        // Cancel-safety: if this future is dropped before the joiner finishes,
        // stop the joiner so it does not keep running (and negotiating DTLS)
        // detached from any consumer. Defused on normal completion below.
        let guard = scopeguard::guard((), |_| {
            let mut ot = self.activate();
            let state = ot.state();

            unsafe { otJoinerStop(state.ot.instance) };
        });

        let res = poll_fn(move |cx| self.activate().state().ot.join_done.poll_wait(cx)).await;

        let _ = scopeguard::ScopeGuard::into_inner(guard);

        ot!(res)
    }

    unsafe extern "C" fn plat_c_joiner_callback(error: otError, context: *mut c_void) {
        let instance = context as *mut otInstance;

        let mut ot = OtContext::callback(instance);
        let state = ot.state();

        state.ot.join_done.signal(error);
    }
}
