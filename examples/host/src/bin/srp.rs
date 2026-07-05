//! Host example driving a remote OpenThread RCP over serial, demonstrating the
//! OpenThread native UDP sockets together with the SRP (Service Registration
//! Protocol) API.
//!
//! Runs the OpenThread stack on a Linux/macOS host, talks to a stock `ot-rcp`
//! over serial (spinel), registers an SRP host + service, and serves UDP.
//!
//! Set the serial device with `RCP_SERIAL` (default `/dev/ttyUSB0`) and,
//! optionally, `THREAD_DATASET` (hex TLV).

use core::fmt::Write as _;
use core::net::{Ipv6Addr, SocketAddrV6};

use embassy_executor::{Executor, Spawner};

use log::info;

use openthread::spinel::{SerialPort, SpinelRadio, UartSpinelTransport};
use openthread::{
    BytesFmt, OpenThread, OtResources, OtSrpResources, OtUdpResources, SimpleRamSettings, SrpConf,
    SrpService, UdpSocket,
};

use rand::rngs::StdRng;
use rand::{RngCore, SeedableRng};

use static_cell::StaticCell;

// Linked for its `utoa`/`strtoul` C symbols, which OpenThread's C references.
use tinyrlibc as _;

// Provides `otPlatCAlloc`/`otPlatFree` for the `heap-ext-ot` feature.
#[path = "../platform.rs"]
mod platform;

const BOUND_PORT: u16 = 1212;

const UDP_SOCKETS_BUF: usize = 1280;
const UDP_MAX_SOCKETS: usize = 2;

const SRP_SERVICE_BUF: usize = 300;
const SRP_MAX_SERVICES: usize = 2;

const DEFAULT_SERIAL: &str = "/dev/ttyUSB0";
const DEFAULT_BAUD: u32 = 115_200;

const THREAD_DATASET: &str = match option_env!("THREAD_DATASET") {
    Some(dataset) => dataset,
    None => "000300001901020fd80208b566147d38e384200e080000639c5d67a3bd0510c490f58d4be0d5eaeb0f09b395d1ae17030d4e4553542d50414e2d304644380708fd7d4f8232cb00000410a7e08419ae47c177fb91bcfcec789aa50c0402a0f77835060004001fffe0",
};

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .init();

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| spawner.spawn(main_task(spawner).unwrap()));
}

#[embassy_executor::task]
async fn main_task(spawner: Spawner) {
    let serial_path = std::env::var("RCP_SERIAL").unwrap_or_else(|_| DEFAULT_SERIAL.into());

    info!("Starting; opening RCP serial {serial_path} @ {DEFAULT_BAUD} baud");

    static RNG: StaticCell<StdRng> = StaticCell::new();
    let rng = RNG.init(StdRng::from_os_rng());

    let mut ieee_eui64 = [0u8; 8];
    rng.fill_bytes(&mut ieee_eui64);

    let random_srp_suffix: u32 = rng.next_u32();

    static OT_RESOURCES: StaticCell<OtResources> = StaticCell::new();
    static OT_UDP_RESOURCES: StaticCell<OtUdpResources<UDP_MAX_SOCKETS, UDP_SOCKETS_BUF>> =
        StaticCell::new();
    static OT_SRP_RESOURCES: StaticCell<OtSrpResources<SRP_MAX_SERVICES, SRP_SERVICE_BUF>> =
        StaticCell::new();
    static OT_SETTINGS_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    static OT_SETTINGS: StaticCell<SimpleRamSettings> = StaticCell::new();

    let ot_resources = OT_RESOURCES.init(OtResources::new());
    let ot_udp_resources = OT_UDP_RESOURCES.init(OtUdpResources::new());
    let ot_srp_resources = OT_SRP_RESOURCES.init(OtSrpResources::new());
    let ot_settings_buf = OT_SETTINGS_BUF.init([0; 1024]);
    let ot_settings = OT_SETTINGS.init(SimpleRamSettings::new(ot_settings_buf));

    let ot = OpenThread::new_with_udp_srp(
        ieee_eui64,
        rng,
        ot_settings,
        ot_resources,
        ot_udp_resources,
        ot_srp_resources,
    )
    .unwrap();

    let serial = SerialPort::open(&serial_path, DEFAULT_BAUD).expect("open RCP serial");
    let radio = SpinelRadio::new(UartSpinelTransport::new(serial));

    spawner.spawn(run_ot(ot.clone(), radio).unwrap());
    spawner.spawn(run_ot_info(ot.clone()).unwrap());

    info!("Dataset: {THREAD_DATASET}");

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

    // NOTE: To get the host registered, we need to add at least one service.
    ot.srp_add_service(&SrpService {
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

    info!("Opened socket on port {BOUND_PORT} and waiting for packets...");

    let mut buf = [0u8; UDP_SOCKETS_BUF];

    loop {
        let (len, local, remote) = socket.recv(&mut buf).await.unwrap();

        info!("Got {} from {} on {}", BytesFmt(&buf[..len]), remote, local);

        socket.send(b"Hello", Some(&local), &remote).await.unwrap();
        info!("Sent `b\"Hello\"`");
    }
}

#[embassy_executor::task]
async fn run_ot(ot: OpenThread<'static>, radio: SpinelRadio<UartSpinelTransport<SerialPort>>) -> ! {
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
            info!("Got new IPv6 address(es) and/or SRP state from OpenThread:\nIP addrs: {addrs:?}\nSRP state: {state:?}\nSRP server addr: {server_addr:?}");

            cur_addrs = addrs;
            cur_state = state;
            cur_server_addr = server_addr;

            ot.srp_conf(|conf, state, empty| {
                info!("SRP conf: {conf:?}, state: {state}, empty: {empty}");
                Ok(())
            })
            .unwrap();

            ot.srp_services(|service| {
                if let Some((service, state, slot)) = service {
                    info!("SRP service: {service}, state: {state}, slot: {slot}");
                }
            })
            .unwrap();

            info!("Waiting for OpenThread changes signal...");
        }

        ot.wait_changed().await;
    }
}
