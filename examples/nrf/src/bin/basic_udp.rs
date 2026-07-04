//! Basic example for NRF, demonstrating the usage of OpenThread native UDP sockets.
//!
//! The example provisions an MTD device with fixed Thread network settings, waits for the device to connect,
//! and then sends and receives Ipv6 UDP packets over the `IEEE 802.15.4` radio.

#![no_std]
#![no_main]

use core::net::{Ipv6Addr, SocketAddrV6};

use defmt::info;

use embedded_alloc::LlffHeap as Heap;

use embassy_executor::InterruptExecutor;
use embassy_executor::Spawner;

use embassy_time::{Duration, Timer};

use embassy_nrf::interrupt;
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::mode::Blocking;
use embassy_nrf::rng::Rng;
use embassy_nrf::{bind_interrupts, peripherals, radio};

use openthread::nrf::{Ieee802154, NrfRadio};
use openthread::{
    BytesFmt, EmbassyTimeTimer, OpenThread, OtResources, OtUdpResources, PhyRadioRunner,
    ProxyRadio, ProxyRadioResources, Radio, SimpleRamSettings, UdpSocket,
};

use panic_rtt_target as _;

use rand_core::RngCore;

use tinyrlibc as _;

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

bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
});

#[interrupt]
unsafe fn EGU0_SWI0() {
    EXECUTOR_HIGH.on_interrupt()
}

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

const BOUND_PORT: u16 = 1212;

const UDP_SOCKETS_BUF: usize = 1280;
const UDP_MAX_SOCKETS: usize = 2;

const THREAD_DATASET: &str = if let Some(dataset) = option_env!("THREAD_DATASET") {
    dataset
} else {
    "000300001901020fd80208b566147d38e384200e080000639c5d67a3bd0510c490f58d4be0d5eaeb0f09b395d1ae17030d4e4553542d50414e2d304644380708fd7d4f8232cb00000410a7e08419ae47c177fb91bcfcec789aa50c0402a0f77835060004001fffe0"
};

// Only needed for tinyrlibc's alloc functions which won't be called at runtime.
//
// If the firmware would not use or need heap allocation for other purposes, this could be replaced
// with stub impls of `calloc` and `free` that panic with `unimplemented!()`,
// and the `#[global_allocator]` attribute could be removed.
#[global_allocator]
static HEAP: Heap = Heap::empty();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;

    let p = embassy_nrf::init(config);

    rtt_target::rtt_init_defmt!();

    info!("Starting...");

    let rng = mk_static!(Rng<'static, Blocking>, Rng::new_blocking(p.RNG));

    let mut ieee_eui64 = [0; 8];
    RngCore::fill_bytes(rng, &mut ieee_eui64);

    let ot_resources = mk_static!(OtResources, OtResources::new());
    let ot_udp_resources =
        mk_static!(OtUdpResources<UDP_MAX_SOCKETS, UDP_SOCKETS_BUF>, OtUdpResources::new());
    let ot_settings_buf = mk_static!([u8; 1024], [0; 1024]);

    let ot_settings = mk_static!(SimpleRamSettings, SimpleRamSettings::new(ot_settings_buf));

    let ot = OpenThread::new_with_udp(ieee_eui64, rng, ot_settings, ot_resources, ot_udp_resources)
        .unwrap();

    info!("About to spawn OT runner");

    let radio = NrfRadio::new(Ieee802154::new(p.RADIO, Irqs));

    let proxy_radio_resources = mk_static!(ProxyRadioResources, ProxyRadioResources::new());
    let (proxy_radio, phy_radio_runner) = ProxyRadio::new(proxy_radio_resources);

    // High-priority executor: EGU0_SWI0, priority level 7
    interrupt::EGU0_SWI0.set_priority(Priority::P7);

    let spawner_high = EXECUTOR_HIGH.start(interrupt::EGU0_SWI0);
    spawner_high.spawn(run_radio(phy_radio_runner, radio).unwrap());

    info!("Radio created");

    spawner.spawn(run_ot(ot.clone(), proxy_radio).unwrap());

    info!("About to spawn OT IP info");

    spawner.spawn(run_ot_ip_info(ot.clone()).unwrap());

    info!("About to spawn OT diagnostics");

    spawner.spawn(run_ot_diagnostics(ot.clone()).unwrap());

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
async fn run_ot(ot: OpenThread<'static>, radio: ProxyRadio<'static>) -> ! {
    ot.run(radio).await
}

#[embassy_executor::task]
async fn run_radio(mut runner: PhyRadioRunner<'static>, radio: NrfRadio<'static>) -> ! {
    runner
        .run(
            radio,
            EmbassyTimeTimer, /*TODO: Likely not precise enough*/
        )
        .await
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

/// Periodically dump the Thread network diagnostics and the message-buffer
/// statistics gathered via the `openthread` crate's diagnostic getters.
#[embassy_executor::task]
async fn run_ot_diagnostics(ot: OpenThread<'static>) -> ! {
    loop {
        Timer::after(Duration::from_secs(1)).await;

        info!("=== OpenThread diagnostics ===");
        info!(
            "commissioned={} role={} channel={} pan_id={=u16:#06x} rloc16={=u16:#06x}",
            ot.is_commissioned(),
            ot.device_role(),
            ot.channel(),
            ot.pan_id(),
            ot.rloc16(),
        );
        info!(
            "ext_address={=u64:#018x} partition_id={=u32:#x} leader_weight={} leader_router_id={}",
            ot.ext_address(),
            ot.partition_id(),
            ot.leader_weight(),
            ot.leader_router_id(),
        );
        info!(
            "net_data_version={} stable_data_version={} mesh_local_prefix={=[u8]:#04x}",
            ot.net_data_version(),
            ot.net_data_stable_version(),
            ot.mesh_local_prefix(),
        );
        ot.network_name(|name| info!("network_name={}", name));

        let bi = ot.buffer_info();
        info!(
            "buffers: total={} free={} max_used={} reassembly_msgs={} reassembly_bufs={}",
            bi.total, bi.free, bi.max_used, bi.reassembly_messages, bi.reassembly_buffers,
        );

        info!("-- neighbor table --");
        ot.neighbor_table(|n| info!("  neighbor {}", n));

        info!("-- route table --");
        ot.route_table(|r| info!("  route {}", r));
    }
}
