//! An example for NRF, demonstrating the usage of OpenThread native UDP sockets as well as the SRP API.
//!
//! The example provisions an MTD device with fixed Thread network settings, waits for the device to connect,
//! and then sends and receives Ipv6 UDP packets over the `IEEE 802.15.4` radio.
//!
//! The example also registers the MTD device under the hostname `srp-example` and should thus be pingable as `srp-example.local`
//! from your Wifi/Ethernet network, as long as you are running the other thread peer as a Thread Border Router (see below).
//!
//! See README.md for instructions on how to configure the other Thread peer (a FTD), using another Esp device.

#![no_std]
#![no_main]

use core::fmt::Write;
use core::net::{Ipv6Addr, SocketAddrV6};

use defmt::info;

use embassy_executor::InterruptExecutor;
use embassy_executor::Spawner;

use embassy_nrf::interrupt;
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::mode::Blocking;
use embassy_nrf::peripherals::{RADIO, RNG};
use embassy_nrf::rng::Rng;
use embassy_nrf::{bind_interrupts, peripherals, radio};

use openthread::nrf::{Ieee802154, NrfRadio};
use openthread::{
    BytesFmt, EmbassyTimeTimer, OpenThread, OtResources, OtSrpResources, OtUdpResources,
    PhyRadioRunner, ProxyRadio, ProxyRadioResources, Radio, SimpleRamSettings, SrpConf, UdpSocket,
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
        let x = STATIC_CELL.uninit().write(($val));
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

const SRP_SERVICE_BUF: usize = 300;
const SRP_MAX_SERVICES: usize = 2;

const THREAD_DATASET: &str = if let Some(dataset) = option_env!("THREAD_DATASET") {
    dataset
} else {
    "0e080000000000010000000300000b35060004001fffe002083a90e3a319a904940708fd1fa298dbd1e3290510fe0458f7db96354eaa6041b880ea9c0f030f4f70656e5468726561642d35386431010258d10410888f813c61972446ab616ee3c556a5910c0402a0f7f8"
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;

    let p = embassy_nrf::init(config);

    rtt_target::rtt_init_defmt!();

    info!("Starting...");

    let rng = mk_static!(Rng<'static, RNG, Blocking>, Rng::new_blocking(p.RNG));

    let mut ieee_eui64 = [0; 8];
    RngCore::fill_bytes(rng, &mut ieee_eui64);

    let random_srp_suffix: u32 = rng.next_u32();

    let ot_resources = mk_static!(OtResources, OtResources::new());
    let ot_udp_resources =
        mk_static!(OtUdpResources<UDP_MAX_SOCKETS, UDP_SOCKETS_BUF>, OtUdpResources::new());
    let ot_srp_resources =
        mk_static!(OtSrpResources<SRP_MAX_SERVICES, SRP_SERVICE_BUF>, OtSrpResources::new());
    let ot_settings_buf = mk_static!([u8; 1024], [0; 1024]);

    let ot_settings = mk_static!(SimpleRamSettings, SimpleRamSettings::new(ot_settings_buf));

    let ot = OpenThread::new_with_udp_srp(
        ieee_eui64,
        rng,
        ot_settings,
        ot_resources,
        ot_udp_resources,
        ot_srp_resources,
    )
    .unwrap();

    info!("About to spawn OT runner");

    let mut radio = NrfRadio::new(Ieee802154::new(p.RADIO, Irqs));

    let proxy_radio_resources = mk_static!(ProxyRadioResources, ProxyRadioResources::new());
    let (proxy_radio, phy_radio_runner) = ProxyRadio::new(radio.caps(), proxy_radio_resources);

    // High-priority executor: EGU0_SWI0, priority level 7
    interrupt::EGU0_SWI0.set_priority(Priority::P7);

    let spawner_high = EXECUTOR_HIGH.start(interrupt::EGU0_SWI0);
    spawner_high
        .spawn(run_radio(phy_radio_runner, radio))
        .unwrap();

    info!("Radio created");

    spawner.spawn(run_ot(ot.clone(), proxy_radio)).unwrap();

    info!("About to spawn OT IP info");

    spawner.spawn(run_ot_info(ot.clone())).unwrap();

    info!("Dataset: {}", THREAD_DATASET);

    ot.srp_autostart().unwrap();

    ot.set_active_dataset_tlv_hexstr(THREAD_DATASET).unwrap();
    ot.enable_ipv6(true).unwrap();
    ot.enable_thread(true).unwrap();

    let mut hostname = heapless::String::<32>::new();
    write!(hostname, "srp-example-{random_srp_suffix:04x}").unwrap();

    let _ = ot.srp_remove_all(false);

    while !ot.srp_is_empty().unwrap() {
        info!("Waiting for SRP records to be removed...");
        ot.wait_changed().await;
    }

    ot.srp_set_conf(&SrpConf {
        host_name: hostname.as_str(),
        ..SrpConf::new()
    })
    .unwrap();

    let mut servicename = heapless::String::<32>::new();
    write!(servicename, "srp{random_srp_suffix:04x}").unwrap();

    // NOTE: To get the host registered, we need to add at least one service
    ot.srp_add_service(&openthread::SrpService {
        name: "_foo._tcp",
        instance_name: servicename.as_str(),
        port: 777,
        subtype_labels: ["foo"].into_iter(),
        txt_entries: [("a", "b".as_bytes())].into_iter(),
        priority: 0,
        weight: 0,
        lease_secs: 0,
        key_lease_secs: 0,
    })
    .unwrap();

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
async fn run_radio(mut runner: PhyRadioRunner<'static>, radio: NrfRadio<'static, RADIO>) -> ! {
    runner
        .run(
            radio,
            EmbassyTimeTimer, /*TODO: Likely not precise enough*/
        )
        .await
}

#[embassy_executor::task]
async fn run_ot_info(ot: OpenThread<'static>) -> ! {
    let mut cur_state = None;
    let mut cur_server_addr = None;

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

        let mut state = cur_state;
        let server_addr = ot.srp_server_addr().unwrap();

        ot.srp_conf(|_, new_state, _| {
            state = Some(new_state);
            Ok(())
        })
        .unwrap();

        if cur_addrs != addrs || cur_state != state || cur_server_addr != server_addr {
            info!("Got new IPv6 address(es) and/or SRP state from OpenThread:\nIP addrs: {:?}\nSRP state: {:?}\nSRP server addr: {:?}", addrs, state, server_addr);

            cur_addrs = addrs;
            cur_state = state;
            cur_server_addr = server_addr;

            ot.srp_conf(|conf, state, empty| {
                info!("SRP conf: {:?}, state: {}, empty: {}", conf, state, empty);

                Ok(())
            })
            .unwrap();

            ot.srp_services(|service| {
                if let Some((service, state, slot)) = service {
                    info!("SRP service: {}, state: {}, slot: {}", service, state, slot);
                }
            })
            .unwrap();

            info!("Waiting for OpenThread changes signal...");
        }

        ot.wait_changed().await;
    }
}
