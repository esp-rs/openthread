//! Basic example for esp32-c6 and esp32-h2, demonstrating the integration of `openthread` with `embassy-net`.
//!
//! The example provisions an MTD device with fixed Thread network settings, waits for the device to connect,
//! and then sends and receives Ipv6 UDP packets over the `IEEE 802.15.4` radio.
//!
//! See README.md for instructions on how to configure the other Thread peer (a FTD), using another Esp device.

#![no_std]
#![no_main]

use core::net::Ipv6Addr;

use log::info;

use embassy_executor::Spawner;
use embassy_net::udp::{PacketMetadata, UdpMetadata, UdpSocket};
use embassy_net::{Config, ConfigV6, Ipv6Cidr, Runner, StackResources, StaticConfigV6};

use esp_hal::rng::Rng;
use esp_hal::timer::systimer::SystemTimer;
use esp_ieee802154::Ieee802154;
use {esp_backtrace as _, esp_println as _};

use heapless::Vec;

use openthread::enet::{self, EnetDriver, EnetRunner};
use openthread::esp::EspRadio;
use openthread::{BytesFmt, OpenThread, OtResources, SimpleRamSettings};

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
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

const BOUND_PORT: u16 = 1212;

const IPV6_PACKET_SIZE: usize = 1280;
const ENET_MAX_SOCKETS: usize = 2;

const THREAD_DATASET: &str = if let Some(dataset) = option_env!("THREAD_DATASET") {
    dataset
} else {
    "0e080000000000010000000300000b35060004001fffe002083a90e3a319a904940708fd1fa298dbd1e3290510fe0458f7db96354eaa6041b880ea9c0f030f4f70656e5468726561642d35386431010258d10410888f813c61972446ab616ee3c556a5910c0402a0f7f8"
};

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    info!("Starting...");

    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_hal_embassy::init(SystemTimer::new(peripherals.SYSTIMER).alarm0);

    let rng = mk_static!(Rng, Rng::new(peripherals.RNG));

    let enet_seed = rng.next_u64();

    let mut ieee_eui64 = [0; 8];
    rng.fill_bytes(&mut ieee_eui64);

    let ot_resources = mk_static!(OtResources, OtResources::new());
    let ot_settings_buf = mk_static!([u8; 1024], [0; 1024]);
    let enet_driver_state =
        mk_static!(enet::EnetDriverState<IPV6_PACKET_SIZE, 1, 1>, enet::EnetDriverState::new());

    let ot_settings = mk_static!(SimpleRamSettings, SimpleRamSettings::new(ot_settings_buf));

    let ot = OpenThread::new(ieee_eui64, rng, ot_settings, ot_resources).unwrap();

    let (_enet_controller, enet_driver_runner, enet_driver) =
        enet::new(ot.clone(), enet_driver_state);

    spawner
        .spawn(run_enet_driver(
            enet_driver_runner,
            EspRadio::new(Ieee802154::new(
                peripherals.IEEE802154,
                peripherals.RADIO_CLK,
            )),
        ))
        .unwrap();

    let enet_resources = mk_static!(StackResources<ENET_MAX_SOCKETS>, StackResources::new());

    let (stack, enet_runner) =
        embassy_net::new(enet_driver, Config::default(), enet_resources, enet_seed);

    spawner.spawn(run_enet(enet_runner)).unwrap();

    info!("Dataset: {THREAD_DATASET}");

    ot.set_active_dataset_tlv_hexstr(THREAD_DATASET).unwrap();
    ot.enable_ipv6(true).unwrap();
    ot.enable_thread(true).unwrap();

    loop {
        info!("Waiting to get an IPv6 address from OpenThread...");

        let mut addrs = heapless::Vec::<(Ipv6Addr, u8), 4>::new();
        ot.ipv6_addrs(|addr| {
            if let Some(addr) = addr {
                let _ = addrs.push(addr);
            }

            Ok(())
        })
        .unwrap();

        if !addrs.is_empty() {
            info!("Got IPv6 address(es) from OpenThread: {addrs:?}");

            // NOTE: Ideally, we should track any changes to the OpenThread Ipv6 conf with `ot_controller.wait_changed()`
            // and re-initialize the embassy-net config with the new Ip and prefix.
            let (linklocal_addr, linklocal_prefix) = addrs
                .iter()
                .find(|(addr, _)| addr.is_unicast_link_local())
                .expect("No link-local address found");

            info!("Will bind to link-local {linklocal_addr} Ipv6 addr");

            stack.set_config_v6(ConfigV6::Static(StaticConfigV6 {
                address: Ipv6Cidr::new(*linklocal_addr, *linklocal_prefix),
                gateway: None,           // TODO
                dns_servers: Vec::new(), // TODO
            }));

            break;
        }
    }

    let (mut rx_meta, mut tx_meta) = ([PacketMetadata::EMPTY; 2], [PacketMetadata::EMPTY; 2]);
    let rx_buf = unsafe { mk_static!([u8; IPV6_PACKET_SIZE]).assume_init_mut() };
    let tx_buf = unsafe { mk_static!([u8; IPV6_PACKET_SIZE]).assume_init_mut() };

    let mut socket = UdpSocket::new(stack, &mut rx_meta, rx_buf, &mut tx_meta, tx_buf);

    socket.bind(BOUND_PORT).unwrap();

    info!("Opened socket on port {BOUND_PORT} and waiting for packets...");

    let buf: &mut [u8] = unsafe { mk_static!([u8; IPV6_PACKET_SIZE]).assume_init_mut() };

    loop {
        let (
            len,
            UdpMetadata {
                endpoint,
                local_address,
                ..
            },
        ) = socket.recv_from(buf).await.unwrap();

        info!(
            "Got {} from {} on {:?}",
            BytesFmt(&buf[..len]),
            endpoint,
            local_address
        );

        socket.send_to(b"Hello", endpoint).await.unwrap();
        info!("Sent `b\"Hello\"`");
    }
}

#[embassy_executor::task]
async fn run_enet_driver(
    mut runner: EnetRunner<'static, IPV6_PACKET_SIZE>,
    radio: EspRadio<'static>,
) -> ! {
    runner.run(radio).await
}

#[embassy_executor::task]
async fn run_enet(mut runner: Runner<'static, EnetDriver<'static, IPV6_PACKET_SIZE>>) -> ! {
    runner.run().await
}
