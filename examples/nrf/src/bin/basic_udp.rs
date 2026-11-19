//! Basic example demonstrating the usage of OpenThread native UDP sockets
//! on top of the nrf-802154 radio driver.
//!
//! The example provisions an MTD device with fixed Thread network settings, waits for the device to connect,
//! and then sends and receives Ipv6 UDP packets over the `IEEE 802.15.4` radio.
//!
//! See README.md for instructions on how to configure the other Thread peer (a FTD), using an Esp device.

#![no_std]
#![no_main]

use core::net::{Ipv6Addr, SocketAddrV6};

use defmt::info;

use embassy_executor::Spawner;

use embassy_time::{Duration, Timer};

use embedded_alloc::LlffHeap as Heap;

use nrf_802154::Radio;
use nrf_mpsl::raw::mpsl_clock_lfclk_cfg_t;
use nrf_mpsl::MultiprotocolServiceLayer;

use openthread::{
    BytesFmt, OpenThread, OtResources, OtUdpResources, Radio as _, SimpleRamSettings, UdpSocket,
};
use openthread_driver_nrf::NrfRadio;
use openthread_examples_nrf::{Irqs, Rng};

use rand_core::RngCore;

use static_cell::StaticCell;

use tinyrlibc as _;

use {defmt_rtt as _, panic_probe as _};

macro_rules! mk_static {
    ($t:ty) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit();
        x
    }};
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

const BOUND_PORT: u16 = 1212;

const UDP_SOCKETS_BUF: usize = 1280;
const UDP_MAX_SOCKETS: usize = 2;

const THREAD_DATASET: &str = if let Some(dataset) = option_env!("THREAD_DATASET") {
    dataset
} else {
    "000300001901020fd80208b566147d38e384200e080000639c5d67a3bd0510c490f58d4be0d5eaeb0f09b395d1ae17030d4e4553542d50414e2d304644380708fd7d4f8232cb00000410a7e08419ae47c177fb91bcfcec789aa50c0402a0f77835060004001fffe0"
};

static MPSL: StaticCell<MultiprotocolServiceLayer<'static>> = StaticCell::new();

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

// Only needed for tinyrlibc's alloc functions which won't be called at runtime.
//
// If the firmware would not use or need heap allocation for other purposes, this could be replaced
// with stub impls of `calloc` and `free` that panic with `unimplemented!()`,
// and the `#[global_allocator]` attribute could be removed.
#[global_allocator]
static HEAP: Heap = Heap::empty();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = openthread_examples_nrf::init();

    info!("Starting...");

    let lfclk_cfg = mpsl_clock_lfclk_cfg_t {
        source: nrf_mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: nrf_mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: nrf_mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };

    let mpsl_p = openthread_examples_nrf::mpsl_peripherals!(p);
    let mpsl = MPSL.init(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    spawner.spawn(mpsl_task(mpsl).unwrap());

    let rng = mk_static!(Rng<'static>, openthread_examples_nrf::rng!(p));

    let mut ieee_eui64 = [0; 8];
    RngCore::fill_bytes(rng, &mut ieee_eui64);

    let ot_resources = mk_static!(OtResources, OtResources::new());
    let ot_udp_resources =
        mk_static!(OtUdpResources<UDP_MAX_SOCKETS, UDP_SOCKETS_BUF>, OtUdpResources::new());
    let ot_settings_buf = mk_static!([u8; 1024], [0; 1024]);

    let ot_settings = mk_static!(SimpleRamSettings, SimpleRamSettings::new(ot_settings_buf));

    let radio = Radio::new(
        p.RADIO,
        openthread_examples_nrf::radio_peripherals!(p),
        Irqs,
        mpsl,
    );
    let mut radio = NrfRadio::new(radio);
    let radio_caps = radio.init().await.unwrap();

    let ot = OpenThread::new_with_udp(
        ieee_eui64,
        &radio_caps,
        rng,
        ot_settings,
        ot_resources,
        ot_udp_resources,
    )
    .unwrap();

    info!("About to spawn OT runner");

    spawner.spawn(run_ot(ot.clone(), radio).unwrap());

    info!("About to spawn OT IP info");

    spawner.spawn(run_ot_ip_info(ot.clone()).unwrap());

    spawner.spawn(heartbeat(ot.clone()).unwrap());

    info!("Dataset: {}", THREAD_DATASET);

    ot.set_active_dataset_tlv_hexstr(THREAD_DATASET).unwrap();
    ot.enable_ipv6(true).unwrap();
    ot.enable_thread(true).unwrap();

    let socket = UdpSocket::bind(
        ot,
        &SocketAddrV6::new(Ipv6Addr::UNSPECIFIED, BOUND_PORT, 0, 0),
    )
    .unwrap();

    info!(
        "Opened socket on port {} and waiting for packets...",
        BOUND_PORT
    );

    let buf: &mut [u8] = unsafe { mk_static!([u8; UDP_SOCKETS_BUF]).assume_init_mut() };

    loop {
        let (len, local, remote) = socket.recv(buf).await.unwrap();

        info!("Got {} from {} on {}", BytesFmt(&buf[..len]), remote, local);

        socket.send(b"Hello", Some(&local), &remote).await.unwrap();
        info!("Sent `b\"Hello\"`");
    }
}

#[embassy_executor::task]
async fn run_ot(ot: OpenThread<'static>, radio: NrfRadio<'static>) -> ! {
    ot.run(radio).await
}

/// Once-a-second diagnostic heartbeat: device role, OpenThread message-buffer
/// pool occupancy (the suspected congestion bottleneck), and the radio-layer RX
/// counters. Watch `free` collapse toward 0 and `reasm` climb under a flood, and
/// `radio rx` keep climbing throughout (proving the radio stays alive).
#[embassy_executor::task]
async fn heartbeat(ot: OpenThread<'static>) -> ! {
    loop {
        Timer::after(Duration::from_secs(1)).await;

        let status = ot.net_status();
        let bi = ot.buffer_info();
        let d = nrf_802154::rx_stats();

        info!(
            "HB role={:?} | buf free={}/{} reasm={} | radio rx={} drop={} qlen={} st={} | rxE={} txE={} txS={} txD={}",
            status.role,
            bi.free,
            bi.total,
            bi.reassembly_buffers,
            d.received,
            d.dropped,
            d.queue_len,
            d.status,
            d.rx_enter,
            d.tx_enter,
            d.tx_sched,
            d.tx_done,
        );
    }
}

#[embassy_executor::task]
async fn run_ot_ip_info(ot: OpenThread<'static>) -> ! {
    let mut cur_addrs = heapless::Vec::<(Ipv6Addr, u8), 4>::new();

    loop {
        let mut addrs = heapless::Vec::<(Ipv6Addr, u8), 4>::new();
        ot.ipv6_addrs(|addr| {
            if let Some(addr) = addr {
                let _ = addrs.push(addr);
            }

            Ok(())
        })
        .unwrap();

        if cur_addrs != addrs {
            info!("Got new IPv6 address(es) from OpenThread: {:?}", addrs);

            cur_addrs = addrs;

            info!("Waiting for OpenThread changes signal...");
        }

        ot.wait_changed().await;
    }
}
