//! Host example driving a remote OpenThread RCP over serial: attach to a
//! Thread network and **ping** another mesh node — ICMPv6 Echo from inside
//! the OpenThread stack ([`OpenThread::ping`]), reporting per-reply
//! round-trip times and final loss statistics.
//!
//! The destination defaults to the network Leader's Anycast Locator (ALOC) —
//! an address that exists on every attached mesh, so the example works
//! without knowing any peer address up front. Override it with `PING_DEST`.
//!
//! Set the serial device with `RCP_SERIAL` (default `/dev/ttyACM0`), the baud
//! rate with `RCP_BAUD` (default 115200) and, optionally, the network with
//! `THREAD_DATASET` (a hex TLV string).

use core::net::Ipv6Addr;

use embassy_executor::{Executor, Spawner};
use embassy_time::{Duration, Timer};

use log::info;

use openthread::spinel::{
    SerialPort, SpinelRadio, SpinelRadioResources, UartSpinelTransport, UartTransportResources,
};
use openthread::{DeviceRole, OpenThread, OtResources, PingConfig, SimpleRamSettings};

use rand::rngs::StdRng;
use rand::{RngCore, SeedableRng};

use static_cell::{ConstStaticCell, StaticCell};

// Linked for its `utoa`/`strtoul` C symbols, which OpenThread's C references.
use tinyrlibc as _;

const DEFAULT_SERIAL: &str = "/dev/ttyACM0";
const DEFAULT_BAUD: u32 = 115_200;

const THREAD_DATASET: &str = match option_env!("THREAD_DATASET") {
    Some(dataset) => dataset,
    None => "000300001901020fd80208b566147d38e384200e080000639c5d67a3bd0510c490f58d4be0d5eaeb0f09b395d1ae17030d4e4553542d50414e2d304644380708fd7d4f8232cb00000410a7e08419ae47c177fb91bcfcec789aa50c0402a0f77835060004001fffe0",
};

/// Number of echo requests per ping run.
const PING_COUNT: u16 = 3;

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

fn main() {
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
    let baud = std::env::var("RCP_BAUD")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(DEFAULT_BAUD);

    info!("Starting; opening RCP serial {serial_path} @ {baud} baud");

    static RNG: StaticCell<StdRng> = StaticCell::new();
    let rng = RNG.init(StdRng::from_os_rng());

    let mut ieee_eui64 = [0u8; 8];
    rng.fill_bytes(&mut ieee_eui64);

    static OT_RESOURCES: StaticCell<OtResources> = StaticCell::new();
    static OT_SETTINGS_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    static OT_SETTINGS: StaticCell<SimpleRamSettings> = StaticCell::new();

    let ot_resources = OT_RESOURCES.init(OtResources::new());
    let ot_settings_buf = OT_SETTINGS_BUF.init([0; 1024]);
    let ot_settings = OT_SETTINGS.init(SimpleRamSettings::new(ot_settings_buf));

    let ot = OpenThread::new(ieee_eui64, rng, ot_settings, ot_resources).unwrap();

    static RADIO_RESOURCES: ConstStaticCell<SpinelRadioResources> =
        ConstStaticCell::new(SpinelRadioResources::new());
    static UART_RESOURCES: ConstStaticCell<UartTransportResources> =
        ConstStaticCell::new(UartTransportResources::new());

    let serial = SerialPort::open(&serial_path, baud).expect("open RCP serial");
    let radio = SpinelRadio::new(
        UartSpinelTransport::new(serial, UART_RESOURCES.take()),
        RADIO_RESOURCES.take(),
    );

    spawner.spawn(run_ot(ot.clone(), radio).unwrap());

    info!("Dataset: {THREAD_DATASET}");

    ot.set_active_dataset_tlv_hexstr(THREAD_DATASET).unwrap();
    ot.enable_ipv6(true).unwrap();
    ot.enable_thread(true).unwrap();

    // Wait until attached.
    loop {
        let role = ot.device_role();
        if matches!(
            role,
            DeviceRole::Child | DeviceRole::Router | DeviceRole::Leader
        ) {
            info!("Attached to the network as {role:?}");
            break;
        }

        ot.wait_changed().await;
    }

    // The destination: `PING_DEST`, or the Leader ALOC (mesh-local prefix +
    // the well-known Leader anycast IID `0:ff:fe00:fc00`).
    let dest = std::env::var("PING_DEST")
        .ok()
        .and_then(|addr| addr.parse().ok())
        .unwrap_or_else(|| {
            let prefix = ot.mesh_local_prefix();

            let mut aloc = [0u8; 16];
            aloc[..8].copy_from_slice(&prefix);
            aloc[8..].copy_from_slice(&[0x00, 0x00, 0x00, 0xff, 0xfe, 0x00, 0xfc, 0x00]);

            Ipv6Addr::from(aloc)
        });

    let mut config = PingConfig::new(dest);
    config.count = PING_COUNT;

    loop {
        info!("Pinging {dest} ({PING_COUNT} requests)...");

        let statistics = ot
            .ping(&config, |reply| {
                info!(
                    "  reply from {}: seq={} bytes={} hops={} time={} ms",
                    reply.sender,
                    reply.sequence_number,
                    reply.size,
                    reply.hop_limit,
                    reply.round_trip_time_millis,
                );
            })
            .await
            .unwrap();

        info!(
            "{} sent, {} received ({} lost), RTT min/max {}/{} ms",
            statistics.sent_count,
            statistics.received_count,
            statistics.sent_count - statistics.received_count,
            statistics.min_round_trip_time_millis,
            statistics.max_round_trip_time_millis,
        );

        Timer::after(Duration::from_secs(5)).await;
    }
}

#[embassy_executor::task]
async fn run_ot(
    ot: OpenThread<'static>,
    radio: SpinelRadio<'static, UartSpinelTransport<'static, SerialPort>>,
) -> ! {
    ot.run(radio).await
}
