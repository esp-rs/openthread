//! Host example driving a remote OpenThread RCP over serial: onboard this
//! device onto a Thread network using the **Thread-native commissioning
//! protocol** ([`OpenThread::join`]) — no out-of-band dataset provisioning.
//!
//! For the join to succeed, a Commissioner must be active on the target
//! network and admitting this device. E.g. on an OTBR:
//!
//! ```text
//! ot-ctl commissioner start
//! ot-ctl commissioner joiner add <this device's EUI-64, printed below> J01NME
//! ```
//!
//! (With no Commissioner around, the joiner performs a real discovery scan
//! and fails with `NotFound` — still a useful bring-up check.)
//!
//! Needs the `joiner` feature: `cargo run --features joiner --bin joiner`
//!
//! Set the serial device with `RCP_SERIAL` (default `/dev/ttyACM0`), the baud
//! rate with `RCP_BAUD` (default 115200), and the pre-shared joiner key with
//! `PSKD` (default `J01NME`).

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
const DEFAULT_PSKD: &str = "J01NME";

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
    let pskd = std::env::var("PSKD").unwrap_or_else(|_| DEFAULT_PSKD.into());

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

    // The joiner needs the interface up (and Thread disabled).
    ot.enable_ipv6(true).unwrap();

    info!(
        "Joining with PSKd {pskd:?}; admit this device on the Commissioner with EUI-64 {:016x}",
        u64::from_be_bytes(ot.ieee_eui64()),
    );

    loop {
        match ot.join(&pskd, None).await {
            Ok(()) => {
                info!("Join successful - the network credentials are stored; attaching...");
                break;
            }
            Err(e) => {
                warn!("Join failed: {e:?}; retrying in 5s");
                embassy_time::Timer::after(embassy_time::Duration::from_secs(5)).await;
            }
        }
    }

    ot.enable_thread(true).unwrap();

    loop {
        let status = ot.net_status();

        info!(
            "Role: {:?}, ext PAN ID: {:x?}",
            status.role, status.ext_pan_id
        );

        if !matches!(status.role, DeviceRole::Detached | DeviceRole::Disabled) {
            ot.ipv6_addrs(|addr| {
                if let Some((addr, prefix)) = addr {
                    info!("  addr: {addr}/{prefix}");
                }
                Ok(())
            })
            .unwrap();
        }

        ot.wait_changed().await;
    }
}

#[embassy_executor::task]
async fn run_ot(
    ot: OpenThread<'static>,
    radio: SpinelRadio<'static, UartSpinelTransport<'static, SerialPort>>,
) -> ! {
    ot.run(radio).await
}
