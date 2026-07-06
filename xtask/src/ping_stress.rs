//! `ping-stress`: an on-demand load/recovery test for a live OpenThread device.
//!
//! Sweeps ICMPv6 echo load (a size × interval matrix, via the system `ping`)
//! against the device and checks the two properties a healthy RX path must
//! have:
//!
//! - **Graceful degradation**: as the offered load exceeds what the 802.15.4
//!   link can carry, loss should grow roughly monotonically with the load —
//!   not fall off a cliff into total deafness.
//! - **Prompt recovery**: right after each burst ends, the device must answer
//!   a clean probe again within a bounded time. A device that stays deaf
//!   after the load is gone indicates an RX-path queue that filled and never
//!   drained (a bug class this crate's radio loop and the spinel driver's RX
//!   queue are specifically designed against).
//!
//! The tool only needs an IPv6 address that routes to the device (typically
//! its OMR address, reachable from the host LAN via a Thread border router),
//! so it runs unchanged against any OpenThread node built on this crate: the
//! host RCP driver, or an MCU (nRF52/ESP32XX) with its native radio.

use std::process::Command;
use std::time::Instant;

use anyhow::{bail, Context, Result};

use log::info;

/// Arguments of the `ping-stress` xtask subcommand.
#[derive(clap::Args, Debug)]
pub struct PingStressArgs {
    /// IPv6 address of the device — must be routable from this host
    /// (typically the device's OMR address, via a Thread border router).
    addr: String,

    /// Network interface to ping from (`ping -I`); required for link-local
    /// addresses.
    #[arg(short = 'I', long)]
    interface: Option<String>,

    /// ICMP payload sizes (bytes) to sweep. The default covers a
    /// single-frame payload and one that 6LoWPAN-fragments into several
    /// 802.15.4 frames.
    #[arg(long, value_delimiter = ',', default_value = "56,512")]
    sizes: Vec<u16>,

    /// Ping intervals (milliseconds) to sweep, heaviest last. The system
    /// `ping` refuses intervals below 2 ms without root.
    #[arg(long, value_delimiter = ',', default_value = "1000,200,100,50,20,10")]
    intervals_ms: Vec<u32>,

    /// Duration of each burst, in seconds.
    #[arg(long, default_value_t = 15)]
    burst_secs: u32,

    /// Maximum time the device may need to answer a fully clean probe after
    /// each burst, in seconds. Exceeding it fails the run.
    #[arg(long, default_value_t = 30)]
    recovery_secs: u32,
}

/// The parsed summary of one `ping` invocation.
#[derive(Debug, Clone, Copy)]
struct PingStats {
    transmitted: u32,
    received: u32,
    /// Average RTT in milliseconds, when at least one reply arrived.
    rtt_avg_ms: Option<f64>,
}

impl PingStats {
    fn loss_pct(&self) -> f64 {
        if self.transmitted == 0 {
            return 0.0;
        }
        (self.transmitted - self.received.min(self.transmitted)) as f64 * 100.0
            / self.transmitted as f64
    }
}

/// One row of the sweep report.
struct BurstOutcome {
    size: u16,
    interval_ms: u32,
    stats: PingStats,
    /// Time until the first fully clean recovery probe, or `None` if the
    /// device did not recover within the limit.
    recovery: Option<f64>,
}

pub fn run(args: &PingStressArgs) -> Result<()> {
    // Baseline: the device must be cleanly reachable before it makes sense to
    // load it.
    info!("Baseline: probing {} ...", args.addr);
    let baseline = ping(args, PingRun::count(10, 500, 56))?;
    if baseline.loss_pct() > 10.0 {
        bail!(
            "baseline probe lost {:.0}% ({}/{} answered) — device unreachable or unhealthy \
             before any load; not proceeding",
            baseline.loss_pct(),
            baseline.received,
            baseline.transmitted,
        );
    }
    info!(
        "Baseline OK: {}/{} answered, avg rtt {}",
        baseline.received,
        baseline.transmitted,
        fmt_rtt(baseline.rtt_avg_ms),
    );

    let mut outcomes = Vec::new();

    for &size in &args.sizes {
        for &interval_ms in &args.intervals_ms {
            info!(
                "Burst: {} B payload every {} ms ({} pps) for {} s ...",
                size,
                interval_ms,
                1000 / interval_ms.max(1),
                args.burst_secs,
            );
            let stats = ping(args, PingRun::deadline(args.burst_secs, interval_ms, size))?;
            info!(
                "  -> {}/{} answered ({:.1}% loss), avg rtt {}",
                stats.received,
                stats.transmitted,
                stats.loss_pct(),
                fmt_rtt(stats.rtt_avg_ms),
            );

            let recovery = measure_recovery(args)?;
            match recovery {
                Some(secs) => info!("  -> recovered in {secs:.1} s"),
                None => info!(
                    "  -> FAILED to recover within {} s (device deaf after load)",
                    args.recovery_secs
                ),
            }

            outcomes.push(BurstOutcome {
                size,
                interval_ms,
                stats,
                recovery,
            });
        }
    }

    report(&outcomes);

    let unrecovered = outcomes.iter().filter(|o| o.recovery.is_none()).count();
    if unrecovered > 0 {
        bail!("{unrecovered} burst(s) left the device deaf beyond the recovery limit");
    }

    Ok(())
}

/// After a burst, probe repeatedly until one probe comes back fully clean
/// (every echo answered), returning the elapsed time — or `None` once the
/// recovery limit is exceeded.
fn measure_recovery(args: &PingStressArgs) -> Result<Option<f64>> {
    let started = Instant::now();

    loop {
        let probe = ping(args, PingRun::count(5, 300, 56))?;
        if probe.received == probe.transmitted && probe.transmitted > 0 {
            return Ok(Some(started.elapsed().as_secs_f64()));
        }

        if started.elapsed().as_secs() > args.recovery_secs as u64 {
            return Ok(None);
        }
    }
}

/// How to bound one `ping` invocation: by echo count or by wall-clock
/// deadline. Interval and payload size apply to both.
struct PingRun {
    count: Option<u32>,
    deadline_secs: Option<u32>,
    interval_ms: u32,
    size: u16,
}

impl PingRun {
    fn count(count: u32, interval_ms: u32, size: u16) -> Self {
        Self {
            count: Some(count),
            deadline_secs: None,
            interval_ms,
            size,
        }
    }

    fn deadline(deadline_secs: u32, interval_ms: u32, size: u16) -> Self {
        Self {
            count: None,
            deadline_secs: Some(deadline_secs),
            interval_ms,
            size,
        }
    }
}

/// Run the system `ping` once and parse its summary.
fn ping(args: &PingStressArgs, run: PingRun) -> Result<PingStats> {
    let mut cmd = Command::new("ping");
    cmd.arg("-6").arg("-q");
    cmd.arg("-i").arg(format!("{}", run.interval_ms as f64 / 1000.0));
    cmd.arg("-s").arg(run.size.to_string());
    if let Some(count) = run.count {
        cmd.arg("-c").arg(count.to_string());
    }
    if let Some(deadline) = run.deadline_secs {
        cmd.arg("-w").arg(deadline.to_string());
    }
    if let Some(iface) = &args.interface {
        cmd.arg("-I").arg(iface);
    }
    cmd.arg(&args.addr);

    let output = cmd
        .output()
        .context("spawning `ping` (is iputils installed?)")?;

    // `ping` exits 0 on success and 1 when not all (or no) replies arrived —
    // both are valid measurements here. Anything else is a usage/network
    // error worth surfacing verbatim.
    let code = output.status.code().unwrap_or(-1);
    if code != 0 && code != 1 {
        bail!(
            "ping failed (exit {code}): {}",
            String::from_utf8_lossy(&output.stderr).trim(),
        );
    }

    parse_ping_output(&String::from_utf8_lossy(&output.stdout))
}

/// Parse the two summary lines of iputils `ping -q`:
///
/// ```text
/// 750 packets transmitted, 748 received, 0.266667% packet loss, time 15150ms
/// rtt min/avg/max/mdev = 12.3/45.6/234.5/12.1 ms
/// ```
///
/// The loss percentage is recomputed from the counts rather than parsed (the
/// line may carry extra clauses such as `+2 duplicates,`).
fn parse_ping_output(out: &str) -> Result<PingStats> {
    let summary = out
        .lines()
        .find(|l| l.contains("packets transmitted"))
        .with_context(|| format!("no summary line in ping output: {out:?}"))?;

    let mut fields = summary.split(',');
    let transmitted = leading_int(fields.next().unwrap_or(""))
        .with_context(|| format!("unparseable transmit count in {summary:?}"))?;
    let received = fields
        .next()
        .and_then(leading_int)
        .with_context(|| format!("unparseable receive count in {summary:?}"))?;

    let rtt_avg_ms = out
        .lines()
        .find(|l| l.starts_with("rtt ") || l.starts_with("round-trip "))
        .and_then(|l| l.split('=').nth(1))
        .and_then(|vals| vals.split('/').nth(1))
        .and_then(|avg| avg.trim().parse::<f64>().ok());

    Ok(PingStats {
        transmitted,
        received,
        rtt_avg_ms,
    })
}

/// The integer at the start of a (whitespace-trimmed) string slice.
fn leading_int(s: &str) -> Option<u32> {
    let s = s.trim_start();
    let end = s.find(|c: char| !c.is_ascii_digit()).unwrap_or(s.len());
    s[..end].parse().ok()
}

fn fmt_rtt(rtt: Option<f64>) -> String {
    match rtt {
        Some(ms) => format!("{ms:.1} ms"),
        None => "n/a".into(),
    }
}

/// Print the sweep as a table, plus a per-size monotonicity note: loss is
/// expected to be non-decreasing as the interval shrinks (load grows); a
/// significant inversion is worth eyeballing, though it is not failed on
/// (radio environments are noisy).
fn report(outcomes: &[BurstOutcome]) {
    println!();
    println!("size (B) | interval (ms) |  pps | tx    | rx    | loss %  | avg rtt (ms) | recovery (s)");
    println!("---------|---------------|------|-------|-------|---------|--------------|-------------");
    for o in outcomes {
        println!(
            "{:>8} | {:>13} | {:>4} | {:>5} | {:>5} | {:>7.2} | {:>12} | {}",
            o.size,
            o.interval_ms,
            1000 / o.interval_ms.max(1),
            o.stats.transmitted,
            o.stats.received,
            o.stats.loss_pct(),
            match o.stats.rtt_avg_ms {
                Some(ms) => format!("{ms:.1}"),
                None => "n/a".into(),
            },
            match o.recovery {
                Some(secs) => format!("{secs:.1}"),
                None => "FAILED".into(),
            },
        );
    }
    println!();

    for size in outcomes
        .iter()
        .map(|o| o.size)
        .collect::<std::collections::BTreeSet<_>>()
    {
        let series: Vec<&BurstOutcome> = outcomes.iter().filter(|o| o.size == size).collect();
        for pair in series.windows(2) {
            let (lighter, heavier) = (pair[0], pair[1]);
            if lighter.stats.loss_pct() > heavier.stats.loss_pct() + 25.0 {
                println!(
                    "NOTE: non-monotonic loss at {size} B: {:.0}% at {} ms vs {:.0}% at {} ms \
                     — worth a second look",
                    lighter.stats.loss_pct(),
                    lighter.interval_ms,
                    heavier.stats.loss_pct(),
                    heavier.interval_ms,
                );
            }
        }
    }
}
