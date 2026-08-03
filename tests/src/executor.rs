//! The executor and embassy-time driver of the simulation-node binaries -
//! one code path with two clocks:
//!
//! - **Real time** ([`Mode::RealTime`]): `embassy_time::Instant` follows the
//!   wall clock; when every task is blocked, the run loop sleeps in `poll(2)`
//!   until the earliest timer deadline or an external wake (a task woken from
//!   another thread, e.g. stdin input or `async-io` readiness).
//!
//! - **Virtual time** ([`Mode::Virtual`]): the clock is a counter that only
//!   advances when the upstream simulator says so, implementing the lockstep
//!   protocol of OpenThread's virtual-time simulation (the C
//!   `virtual_time/platform-sim.c` in Rust terms): when the node goes fully
//!   idle, send the simulator an `ALARM_FIRED` event carrying the delay to
//!   the earliest timer ("asleep until then, unless something arrives"),
//!   then block on the event socket; every received event advances the
//!   virtual clock by its relative delay, and a radio event additionally
//!   carries a frame for [`VtRadio`](crate::vt::VtRadio). A wake by another
//!   thread (CLI input) just re-runs the loop - the fresh sleep event sent on
//!   the next idle supersedes the previous one, exactly as the C node's
//!   `select` on stdin does.
//!
//! The lockstep contract is why this is an executor concern and not a time
//! driver alone: the sleep event must be sent at the moment the *whole node*
//! is quiescent, and only the executor's idle point knows that. The embassy
//! arch executors keep that point internal, so the binaries run on
//! `embassy_executor::raw` with this loop (and this crate's `__pender`)
//! instead of an `arch`/`platform` feature.

use std::net::UdpSocket;
use std::os::fd::AsFd;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Mutex, OnceLock};
use std::task::Waker;
use std::time::Instant;

use embassy_executor::{raw, Spawner};

use embassy_time_driver::Driver;

use nix::poll::{poll, PollFd, PollFlags, PollTimeout};

use crate::vt::{self, VtEventKind, VtLink};

/// How the executor passes time when idle.
pub enum Mode {
    /// Wall-clock time; idle = sleep until the earliest timer deadline.
    RealTime,
    /// Simulator-controlled time; idle = the lockstep sleep-event protocol
    /// over the given link.
    Virtual(VtLink),
}

/// Run the executor forever. Call once, from `main`.
pub fn run(mode: Mode, init: impl FnOnce(Spawner)) -> ! {
    let wake: &'static WakePipe = Box::leak(Box::new(WakePipe::new()));
    let executor: &'static raw::Executor = Box::leak(Box::new(raw::Executor::new(
        core::ptr::from_ref(wake).cast_mut().cast(),
    )));

    if let Mode::Virtual(_) = &mode {
        DRIVER.virt.store(true, Ordering::Relaxed);
    }

    init(executor.spawner());

    loop {
        unsafe { executor.poll() };

        // `raw::Executor::poll` runs ONE round: tasks woken during the round
        // (e.g. by a signal from another task) are queued for the next one,
        // announced through the pender. The node is only quiescent once a
        // round completes with no such wake - going idle earlier would
        // advertise a sleep deadline that pending task rounds are about to
        // change (in virtual time: a stale alarm the simulator then schedules
        // a whole `go()` window too late).
        if wake.drain() {
            continue;
        }

        // Wake every timer that is due; if any was, the tasks it unblocked
        // must run before the node can be considered idle.
        let now = DRIVER.now();
        if DRIVER.fire_due(now) {
            continue;
        }

        match &mode {
            Mode::RealTime => {
                // Block until the earliest deadline or an external wake. The
                // timeout clamp is harmless: expiring early just re-evaluates.
                let timeout = match DRIVER.next_deadline() {
                    Some(at) => {
                        let millis = (at.saturating_sub(now)).div_ceil(1000);
                        PollTimeout::from(millis.min(u16::MAX as u64) as u16)
                    }
                    None => PollTimeout::NONE,
                };

                let mut fds = [PollFd::new(wake.sock.as_fd(), PollFlags::POLLIN)];
                let _ = poll(&mut fds, timeout);

                wake.drain();
            }
            Mode::Virtual(link) => {
                // Fully idle: report the time to our earliest alarm (or
                // "forever", as the C platform does when none is scheduled)
                // and block until the simulator hands us the next event -
                // or another thread wakes a task (CLI input).
                let delay = DRIVER
                    .next_deadline()
                    .map(|at| at - now)
                    .unwrap_or(i64::MAX as u64);

                log::trace!("VT idle: sleep(delay={delay})");
                link.send_sleep(delay);

                let mut fds = [
                    PollFd::new(link.fd(), PollFlags::POLLIN),
                    PollFd::new(wake.sock.as_fd(), PollFlags::POLLIN),
                ];
                let _ = poll(&mut fds, PollTimeout::NONE);

                let link_ready = fds[0]
                    .revents()
                    .is_some_and(|r| r.contains(PollFlags::POLLIN));

                if link_ready {
                    // Exactly one event: the simulator delivers the next one
                    // only after our next sleep event acknowledges this one.
                    match link.recv() {
                        Ok((delay, kind)) => {
                            let now = DRIVER.vt_now.fetch_add(delay, Ordering::Relaxed) + delay;

                            match kind {
                                VtEventKind::Alarm => {
                                    log::trace!("VT event: alarm(+{delay}) now={now}")
                                }
                                VtEventKind::RadioFrame(frame) => {
                                    log::trace!(
                                        "VT event: frame(+{delay}) now={now} len={} seq={}",
                                        frame.len(),
                                        frame.seq(),
                                    );
                                    vt::deliver_frame(frame)
                                }
                                VtEventKind::Other(kind) => {
                                    log::warn!("Unsupported simulator event {kind}, ignored")
                                }
                            }
                        }
                        Err(err) => panic!("simulator link receive failed: {err}"),
                    }
                }

                wake.drain();
            }
        }
    }
}

/// Cross-thread wake channel: the `__pender` writes, the idle loop `poll`s
/// and drains. A loopback UDP socket connected to itself, so that the wake
/// can participate in the same `poll(2)` as the simulator event socket.
struct WakePipe {
    sock: UdpSocket,
}

impl WakePipe {
    fn new() -> Self {
        let sock = UdpSocket::bind("127.0.0.1:0").expect("bind wake pipe");
        sock.connect(sock.local_addr().expect("wake pipe addr"))
            .expect("connect wake pipe");
        sock.set_nonblocking(true).expect("wake pipe nonblocking");

        Self { sock }
    }

    fn notify(&self) {
        // A full socket buffer means wakes are already pending - fine.
        let _ = self.sock.send(&[0]);
    }

    /// Drain pending wake bytes; reports whether there were any.
    fn drain(&self) -> bool {
        let mut buf = [0; 16];
        let mut any = false;
        while self.sock.recv(&mut buf).is_ok() {
            any = true;
        }
        any
    }
}

#[export_name = "__pender"]
fn pender(context: *mut ()) {
    let wake = unsafe { &*(context as *const WakePipe) };
    wake.notify();
}

/// The embassy-time driver: virtual or wall-clock "now" (see the module
/// docs), plus the timer queue the run loop serves in both modes.
struct SimDriver {
    timers: Mutex<Vec<(u64, Waker)>>,
    virt: AtomicBool,
    vt_now: AtomicU64,
    rt_epoch: OnceLock<Instant>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: SimDriver = SimDriver {
    timers: Mutex::new(Vec::new()),
    virt: AtomicBool::new(false),
    vt_now: AtomicU64::new(0),
    rt_epoch: OnceLock::new(),
});

impl SimDriver {
    /// Earliest scheduled deadline, in ticks (µs).
    fn next_deadline(&self) -> Option<u64> {
        let timers = self.timers.lock().unwrap();
        timers.iter().map(|(at, _)| *at).min()
    }

    /// Wake and remove every timer due at `now`; report whether any was.
    fn fire_due(&self, now: u64) -> bool {
        let mut due = Vec::new();

        {
            let mut timers = self.timers.lock().unwrap();
            let mut index = 0;
            while index < timers.len() {
                if timers[index].0 <= now {
                    due.push(timers.swap_remove(index).1);
                } else {
                    index += 1;
                }
            }
        }

        // Outside the lock: a woken task may re-schedule immediately.
        let fired = !due.is_empty();
        for waker in due {
            waker.wake();
        }

        fired
    }
}

impl embassy_time_driver::Driver for SimDriver {
    fn now(&self) -> u64 {
        if self.virt.load(Ordering::Relaxed) {
            self.vt_now.load(Ordering::Relaxed)
        } else {
            self.rt_epoch.get_or_init(Instant::now).elapsed().as_micros() as u64
        }
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        let mut timers = self.timers.lock().unwrap();

        // One entry per task, at the EARLIEST requested deadline: a single
        // task can hold several live timers (e.g. an alarm and an ACK
        // timeout selected together), all reporting the same waker - taking
        // the latest would sleep past the earlier one, which virtual time
        // never forgives (nothing else wakes the node). A too-early wake is
        // harmless: the woken task simply re-schedules what remains.
        if let Some(entry) = timers.iter_mut().find(|(_, w)| w.will_wake(waker)) {
            entry.0 = entry.0.min(at);
        } else {
            timers.push((at, waker.clone()));
        }
    }
}
