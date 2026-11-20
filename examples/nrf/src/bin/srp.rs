//! An example demonstrating the usage of OpenThread native UDP sockets as well as the SRP API
//! on top of the nrf-802154 radio driver.
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

use embassy_executor::Spawner;

use embedded_alloc::LlffHeap as Heap;

use nrf_802154::Radio;
use nrf_mpsl::raw::mpsl_clock_lfclk_cfg_t;
use nrf_mpsl::MultiprotocolServiceLayer;

use openthread::{
    BytesFmt, OpenThread, OtResources, OtSrpResources, OtUdpResources, Radio as _,
    SimpleRamSettings, SrpConf, UdpSocket,
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

const SRP_SERVICE_BUF: usize = 300;
const SRP_MAX_SERVICES: usize = 2;

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

    let random_srp_suffix: u32 = rng.next_u32();

    let ot_resources = mk_static!(OtResources, OtResources::new());
    let ot_udp_resources =
        mk_static!(OtUdpResources<UDP_MAX_SOCKETS, UDP_SOCKETS_BUF>, OtUdpResources::new());
    let ot_srp_resources =
        mk_static!(OtSrpResources<SRP_MAX_SERVICES, SRP_SERVICE_BUF>, OtSrpResources::new());
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

    let ot = OpenThread::new_with_udp_srp(
        ieee_eui64,
        &radio_caps,
        rng,
        ot_settings,
        ot_resources,
        ot_udp_resources,
        ot_srp_resources,
    )
    .unwrap();

    info!("About to spawn OT runner");

    spawner.spawn(run_ot(ot.clone(), radio).unwrap());

    info!("About to spawn OT IP info");

    spawner.spawn(run_ot_info(ot.clone()).unwrap());

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
async fn run_ot(ot: OpenThread<'static>, radio: NrfRadio<'static>) -> ! {
    ot.run(radio).await
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
