//! Host example driving a remote OpenThread RCP over serial, demonstrating the
//! usage of OpenThread native UDP sockets.
//!
//! Runs the full OpenThread stack on a Linux/macOS host and talks to a stock
//! `ot-rcp` radio co-processor over a serial port (the `spinel` protocol). It
//! provisions an MTD device with fixed Thread network settings, waits for it to
//! connect, and then sends and receives IPv6 UDP packets.
//!
//! Set the serial device with `RCP_SERIAL` (default `/dev/ttyACM0`) and,
//! optionally, the network with `THREAD_DATASET` (a hex TLV string).

use core::net::{Ipv6Addr, SocketAddrV6};

use embassy_executor::{Executor, Spawner};

use log::info;

use openthread::spinel::{SerialPort, SpinelRadio, UartSpinelTransport};
use openthread::{BytesFmt, OpenThread, OtResources, OtUdpResources, SimpleRamSettings, UdpSocket};

use rand::rngs::StdRng;
use rand::{RngCore, SeedableRng};

use static_cell::StaticCell;

// Linked for its `utoa`/`strtoul` C symbols, which OpenThread's C references.
use tinyrlibc as _;

const BOUND_PORT: u16 = 1212;

const UDP_SOCKETS_BUF: usize = 1280;
const UDP_MAX_SOCKETS: usize = 2;

const DEFAULT_SERIAL: &str = "/dev/ttyACM0";
const DEFAULT_BAUD: u32 = 115_200;

const THREAD_DATASET: &str = match option_env!("THREAD_DATASET") {
    Some(dataset) => dataset,
    None => "000300001901020fd80208b566147d38e384200e080000639c5d67a3bd0510c490f58d4be0d5eaeb0f09b395d1ae17030d4e4553542d50414e2d304644380708fd7d4f8232cb00000410a7e08419ae47c177fb91bcfcec789aa50c0402a0f77835060004001fffe0",
};

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

fn main() {
    // Default to `info`; `RUST_LOG` overrides (e.g. `RUST_LOG=trace` surfaces
    // the OpenThread C-stack logs when built with a verbose `OT_LOG_LEVEL`).
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .parse_default_env()
        .init();

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| spawner.spawn(main_task(spawner).unwrap()));
}

#[embassy_executor::task]
async fn main_task(spawner: Spawner) {
    let serial_path = std::env::var("RCP_SERIAL").unwrap_or_else(|_| DEFAULT_SERIAL.into());

    info!("Starting; opening RCP serial {serial_path} @ {DEFAULT_BAUD} baud");

    // A `'static` RNG (OpenThread stores the reference), seeded from the OS.
    static RNG: StaticCell<StdRng> = StaticCell::new();
    let rng = RNG.init(StdRng::from_os_rng());

    // A random EUI-64 for this host node.
    let mut ieee_eui64 = [0u8; 8];
    rng.fill_bytes(&mut ieee_eui64);

    static OT_RESOURCES: StaticCell<OtResources> = StaticCell::new();
    static OT_UDP_RESOURCES: StaticCell<OtUdpResources<UDP_MAX_SOCKETS, UDP_SOCKETS_BUF>> =
        StaticCell::new();
    static OT_SETTINGS_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    static OT_SETTINGS: StaticCell<SimpleRamSettings> = StaticCell::new();

    let ot_resources = OT_RESOURCES.init(OtResources::new());
    let ot_udp_resources = OT_UDP_RESOURCES.init(OtUdpResources::new());
    let ot_settings_buf = OT_SETTINGS_BUF.init([0; 1024]);
    let ot_settings = OT_SETTINGS.init(SimpleRamSettings::new(ot_settings_buf));

    let ot = OpenThread::new_with_udp(ieee_eui64, rng, ot_settings, ot_resources, ot_udp_resources)
        .unwrap();

    // The remote radio: a `SpinelRadio` over the host serial port (HDLC-framed).
    // Unlike the embedded examples there is no `ProxyRadio` / high-priority
    // executor split — serial I/O is not latency-critical, so the radio runs
    // directly in `OpenThread::run`.
    let serial = SerialPort::open(&serial_path, DEFAULT_BAUD).expect("open RCP serial");
    let radio = SpinelRadio::new(UartSpinelTransport::new(serial));

    spawner.spawn(run_ot(ot.clone(), radio).unwrap());
    spawner.spawn(run_ot_ip_info(ot.clone()).unwrap());

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
        }

        ot.wait_changed().await;
    }
}
