//! Host example driving a remote OpenThread RCP over serial that attaches as a
//! child and then **forces a router upgrade** via [`OpenThread::become_router`],
//! instead of waiting for OpenThread's jittered automatic upgrade.
//!
//! Its purpose is to deterministically exercise the child-to-router role
//! transition — which is when OpenThread calls `otPlatRadioSetAlternateShortAddress`
//! with the node's old (child) RLOC16 (the alternate short address), then clears
//! it ~8 s later. Run with `RUST_LOG=info` and watch for the
//! `Plat radio set alternate short address callback` line.
//!
//! Needs the `ftd` feature (only a Full Thread Device can become a router):
//! `cargo run --features ftd --bin become_router`
//!
//! Set the serial device with `RCP_SERIAL` (default `/dev/ttyACM0`), the baud
//! rate with `RCP_BAUD` (default 115200) and, optionally, `THREAD_DATASET`.

use embassy_executor::{Executor, Spawner};

use log::{info, warn};

use openthread::spinel::{
    SerialPort, SpinelRadio, SpinelRadioResources, UartSpinelTransport, UartTransportResources,
};
use openthread::{DeviceRole, OpenThread, OtResources, SimpleRamSettings};

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

    // Wait until we have attached as a child.
    loop {
        let role = ot.device_role();
        info!("Role: {role:?} (eligible: {})", ot.router_eligible());
        if matches!(
            role,
            DeviceRole::Child | DeviceRole::Router | DeviceRole::Leader
        ) {
            break;
        }
        ot.wait_changed().await;
    }

    // Force the router upgrade. If we are already a router/leader this is a
    // harmless no-op error; if we are a child, it kicks off the child->router
    // transition (which drives `otPlatRadioSetAlternateShortAddress`).
    if matches!(ot.device_role(), DeviceRole::Child) {
        info!("Child attached — forcing router upgrade via become_router()...");
        match ot.become_router() {
            Ok(()) => info!("become_router(): Address Solicit sent"),
            Err(e) => warn!("become_router() failed: {e:?}"),
        }
    }

    // Report every subsequent role change.
    loop {
        ot.wait_changed().await;
        info!("Role now: {:?}", ot.device_role());
    }
}

#[embassy_executor::task]
async fn run_ot(
    ot: OpenThread<'static>,
    radio: SpinelRadio<'static, UartSpinelTransport<'static, SerialPort>>,
) -> ! {
    ot.run(radio).await
}
