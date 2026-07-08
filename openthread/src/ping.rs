//! Ping sender API: ICMPv6 Echo diagnostics from inside the OpenThread stack
//! (`otPingSenderPing`) — round-trip times, loss and per-reply details without
//! hand-rolling ICMPv6 over the raw IPv6 API.

use core::ffi::c_void;
use core::future::poll_fn;
use core::net::Ipv6Addr;

use crate::sys::{
    otError_OT_ERROR_BUSY, otInstance, otIp6Address, otIp6Address__bindgen_ty_1, otPingSenderPing,
    otPingSenderReply, otPingSenderStatistics, otPingSenderStop,
};
use crate::{ot, OpenThread, OtContext, OtError};

/// The configuration of a ping run (`otPingSenderConfig`).
///
/// All fields except [`destination`](Self::destination) may be left at their
/// zero/`None` values, in which case OpenThread's defaults apply (one request,
/// 8 data bytes, 1 s interval).
#[derive(Debug, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PingConfig {
    /// Destination address to ping.
    pub destination: Ipv6Addr,
    /// Source address of the ping; `None` to let OpenThread pick one.
    pub source: Option<Ipv6Addr>,
    /// Data size (# of bytes) of the echo payload, excluding the IPv6/ICMPv6
    /// headers. Zero for the default.
    pub size: u16,
    /// Number of ping requests to send. Zero for the default (one).
    pub count: u16,
    /// Interval between requests, in milliseconds. Zero for the default.
    pub interval_millis: u32,
    /// Time to wait for the final reply after sending the final request, in
    /// milliseconds. Zero for the default.
    pub timeout_millis: u16,
    /// Hop limit; zero for the default (unless
    /// [`allow_zero_hop_limit`](Self::allow_zero_hop_limit) is set).
    pub hop_limit: u8,
    /// Interpret a [`hop_limit`](Self::hop_limit) of zero as a literal zero
    /// hop limit rather than "use the default".
    pub allow_zero_hop_limit: bool,
    /// Allow pings to a multicast group the device itself is subscribed to to
    /// be looped back.
    pub multicast_loop: bool,
}

impl PingConfig {
    /// Create a new `PingConfig` for pinging `destination`, with OpenThread's
    /// defaults for everything else.
    pub const fn new(destination: Ipv6Addr) -> Self {
        Self {
            destination,
            source: None,
            size: 0,
            count: 0,
            interval_millis: 0,
            timeout_millis: 0,
            hop_limit: 0,
            allow_zero_hop_limit: false,
            multicast_loop: false,
        }
    }
}

/// A single ping reply (`otPingSenderReply`).
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PingReply {
    /// The address the reply was received from.
    pub sender: Ipv6Addr,
    /// Round-trip time, in milliseconds.
    pub round_trip_time_millis: u16,
    /// Reply data size (# of bytes), excluding the IPv6/ICMPv6 headers.
    pub size: u16,
    /// Sequence number.
    pub sequence_number: u16,
    /// Hop limit of the reply.
    pub hop_limit: u8,
}

impl From<&otPingSenderReply> for PingReply {
    fn from(reply: &otPingSenderReply) -> Self {
        Self {
            sender: Ipv6Addr::from(unsafe { reply.mSenderAddress.mFields.m8 }),
            round_trip_time_millis: reply.mRoundTripTime,
            size: reply.mSize,
            sequence_number: reply.mSequenceNumber,
            hop_limit: reply.mHopLimit,
        }
    }
}

/// The statistics of a completed ping run (`otPingSenderStatistics`), returned
/// by [`OpenThread::ping`].
#[derive(Debug, Copy, Clone, Default, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PingStatistics {
    /// The number of ping requests sent.
    pub sent_count: u16,
    /// The number of ping replies received.
    pub received_count: u16,
    /// The total round-trip time of all replies, in milliseconds.
    pub total_round_trip_time_millis: u32,
    /// The minimum round-trip time among the replies, in milliseconds.
    pub min_round_trip_time_millis: u16,
    /// The maximum round-trip time among the replies, in milliseconds.
    pub max_round_trip_time_millis: u16,
    /// Whether the destination was a multicast address.
    pub is_multicast: bool,
}

impl From<&otPingSenderStatistics> for PingStatistics {
    fn from(stats: &otPingSenderStatistics) -> Self {
        Self {
            sent_count: stats.mSentCount,
            received_count: stats.mReceivedCount,
            total_round_trip_time_millis: stats.mTotalRoundTripTime,
            min_round_trip_time_millis: stats.mMinRoundTripTime,
            max_round_trip_time_millis: stats.mMaxRoundTripTime,
            is_multicast: stats.mIsMulticast,
        }
    }
}

fn to_ot_ip6_addr(addr: Ipv6Addr) -> otIp6Address {
    otIp6Address {
        mFields: otIp6Address__bindgen_ty_1 { m8: addr.octets() },
    }
}

impl<'a> OpenThread<'a> {
    /// Ping the destination described by `config`, invoking `f` for each
    /// received reply, and return the run's statistics once it completes
    /// (`otPingSenderPing`).
    ///
    /// The device must be attached to a Thread network for anything beyond
    /// pinging its own addresses to succeed.
    ///
    /// Only one ping run can be in flight at a time; a concurrent call is
    /// reported as a `BUSY` error. Dropping the returned future cancels the
    /// ping in progress (`otPingSenderStop`).
    ///
    /// NOTE: The future returned by this method is currently NOT
    /// `core::mem::forget` safe. Its constructor MUST run, so don't call
    /// `core::mem::forget` on it.
    pub async fn ping<F>(&self, config: &PingConfig, mut f: F) -> Result<PingStatistics, OtError>
    where
        F: FnMut(&PingReply),
    {
        {
            let mut ot = self.activate();
            let state = ot.state();

            if state.ot.ping_callback.is_some() {
                warn!("Another ping in progress");
                return Err(OtError::new(otError_OT_ERROR_BUSY));
            }

            // Clear any stale completion left over from a prior ping whose
            // future was dropped after the callback signalled but before
            // `poll_wait` consumed it.
            state.ot.ping_done.reset();

            {
                let f: &mut dyn FnMut(&PingReply) = &mut f;

                state.ot.ping_callback = Some(unsafe {
                    core::mem::transmute::<&mut dyn FnMut(&PingReply), &'a mut dyn FnMut(&PingReply)>(
                        f,
                    )
                });

                let raw_config = crate::sys::otPingSenderConfig {
                    mSource: to_ot_ip6_addr(config.source.unwrap_or(Ipv6Addr::UNSPECIFIED)),
                    mDestination: to_ot_ip6_addr(config.destination),
                    mReplyCallback: Some(Self::plat_c_ping_reply_callback),
                    mStatisticsCallback: Some(Self::plat_c_ping_statistics_callback),
                    mCallbackContext: state.ot.instance as *mut _,
                    mSize: config.size,
                    mCount: config.count,
                    mInterval: config.interval_millis,
                    mTimeout: config.timeout_millis,
                    mHopLimit: config.hop_limit,
                    mAllowZeroHopLimit: config.allow_zero_hop_limit,
                    mMulticastLoop: config.multicast_loop,
                };

                let res = ot!(unsafe { otPingSenderPing(state.ot.instance, &raw_config) });
                if res.is_err() {
                    // Failed to start - drop the stashed closure reference.
                    state.ot.ping_callback = None;
                    res?;
                }
            }
        }

        // Cancel-safety: if this future is dropped before the run completes,
        // stop the sender and drop the stashed (lifetime-erased) `F` reference.
        // See the forget-safety note in `OpenThread::scan` - the same caveat
        // applies to the closure reference stashed here.
        let _guard = scopeguard::guard((), |_| {
            let mut ot = self.activate();
            let state = ot.state();

            unsafe { otPingSenderStop(state.ot.instance) };

            state.ot.ping_callback = None;
        });

        let statistics =
            poll_fn(move |cx| self.activate().state().ot.ping_done.poll_wait(cx)).await;

        Ok(statistics)
    }

    unsafe extern "C" fn plat_c_ping_reply_callback(
        reply: *const otPingSenderReply,
        context: *mut c_void,
    ) {
        let instance = context as *mut otInstance;

        let mut ot = OtContext::callback(instance);
        let state = ot.state();

        let Some(reply) = (unsafe { reply.as_ref() }) else {
            return;
        };

        if let Some(f) = state.ot.ping_callback.as_mut() {
            f(&reply.into());
        }
    }

    unsafe extern "C" fn plat_c_ping_statistics_callback(
        statistics: *const otPingSenderStatistics,
        context: *mut c_void,
    ) {
        let instance = context as *mut otInstance;

        let mut ot = OtContext::callback(instance);
        let state = ot.state();

        let Some(statistics) = (unsafe { statistics.as_ref() }) else {
            return;
        };

        state.ot.ping_callback = None;
        state.ot.ping_done.signal(statistics.into());
    }
}
