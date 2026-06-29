//! Basic example for esp32-c6 and esp32-h2, demonstrating the usage of OpenThread native UDP sockets.
//!
//! The example provisions an MTD device with fixed Thread network settings, waits for the device to connect,
//! and then sends and receives Ipv6 UDP packets over the `IEEE 802.15.4` radio.

#![no_std]
#![no_main]

use core::net::{Ipv6Addr, SocketAddrV6};

use log::info;

use embassy_executor::Spawner;

use embassy_time::{Duration, Timer};

use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ieee802154::Ieee802154;
use {esp_backtrace as _, esp_println as _};

use openthread::esp::EspRadio;
use openthread::{BytesFmt, OpenThread, OtResources, OtUdpResources, SimpleRamSettings, UdpSocket};

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

const BOUND_PORT: u16 = 1212;

const UDP_SOCKETS_BUF: usize = 1280;
const UDP_MAX_SOCKETS: usize = 2;

const THREAD_DATASET: &str = if let Some(dataset) = option_env!("THREAD_DATASET") {
    dataset
} else {
    "000300001901020fd80208b566147d38e384200e080000639c5d67a3bd0510c490f58d4be0d5eaeb0f09b395d1ae17030d4e4553542d50414e2d304644380708fd7d4f8232cb00000410a7e08419ae47c177fb91bcfcec789aa50c0402a0f77835060004001fffe0"
};

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_alloc::heap_allocator!(size:1024);

    esp_println::logger::init_logger_from_env();

    info!("Starting...");

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT)
            .software_interrupt0,
    );

    // TODO: Use TRNG?
    let rng = mk_static!(Rng, Rng::new());

    let mut ieee_eui64 = [0; 8];
    rng.fill_bytes(&mut ieee_eui64);

    let ot_resources = mk_static!(OtResources, OtResources::new());
    let ot_udp_resources =
        mk_static!(OtUdpResources<UDP_MAX_SOCKETS, UDP_SOCKETS_BUF>, OtUdpResources::new());
    let ot_settings_buf = mk_static!([u8; 1024], [0; 1024]);

    let ot_settings = mk_static!(SimpleRamSettings, SimpleRamSettings::new(ot_settings_buf));

    let ot = OpenThread::new_with_udp(ieee_eui64, rng, ot_settings, ot_resources, ot_udp_resources)
        .unwrap();

    spawner.spawn(
        run_ot(
            ot.clone(),
            EspRadio::new(Ieee802154::new(peripherals.IEEE802154)),
        )
        .unwrap(),
    );

    spawner.spawn(run_ot_ip_info(ot.clone()).unwrap());

    spawner.spawn(run_ot_diagnostics(ot.clone()).unwrap());

    info!("Dataset: {THREAD_DATASET}");

    ot.set_active_dataset_tlv_hexstr(THREAD_DATASET).unwrap();
    ot.enable_ipv6(true).unwrap();
    ot.enable_thread(true).unwrap();

    let socket = UdpSocket::bind(
        ot,
        &SocketAddrV6::new(Ipv6Addr::UNSPECIFIED, BOUND_PORT, 0, 0),
    )
    .unwrap();

    info!("Opened socket on port {BOUND_PORT} and waiting for packets...");

    let buf: &mut [u8] = unsafe { mk_static!([u8; UDP_SOCKETS_BUF]).assume_init_mut() };

    loop {
        let (len, local, remote) = socket.recv(buf).await.unwrap();

        info!("Got {} from {} on {}", BytesFmt(&buf[..len]), remote, local);

        socket.send(b"Hello", Some(&local), &remote).await.unwrap();
        info!("Sent `b\"Hello\"`");
    }
}

#[embassy_executor::task]
async fn run_ot(ot: OpenThread<'static>, radio: EspRadio<'static>) -> ! {
    ot.run(radio).await
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
            info!("Got new IPv6 address(es) from OpenThread: {addrs:?}");

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
            "commissioned={} role={:?} channel={} pan_id={:#06x} rloc16={:#06x}",
            ot.is_commissioned(),
            ot.device_role(),
            ot.channel(),
            ot.pan_id(),
            ot.rloc16(),
        );
        info!(
            "ext_address={:#018x} partition_id={:#x} leader_weight={} leader_router_id={}",
            ot.ext_address(),
            ot.partition_id(),
            ot.leader_weight(),
            ot.leader_router_id(),
        );
        info!(
            "net_data_version={} stable_data_version={} mesh_local_prefix={:02x?}",
            ot.net_data_version(),
            ot.net_data_stable_version(),
            ot.mesh_local_prefix(),
        );
        ot.network_name(|name| info!("network_name={name}"));

        let bi = ot.buffer_info();
        info!(
            "buffers: total={} free={} max_used={} reassembly_msgs={} reassembly_bufs={}",
            bi.total, bi.free, bi.max_used, bi.reassembly_messages, bi.reassembly_buffers,
        );

        info!("-- neighbor table --");
        ot.neighbor_table(|n| info!("  neighbor {n:?}"));

        info!("-- route table --");
        ot.route_table(|r| info!("  route {r:?}"));
    }
}
