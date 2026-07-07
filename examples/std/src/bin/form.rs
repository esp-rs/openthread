//! Host example driving a remote OpenThread RCP over serial: survey the RF
//! environment with an **energy scan** and then **form a brand-new Thread
//! network** on the quietest channel, with this node becoming the Leader.
//!
//! Demonstrates:
//! - [`OpenThread::energy_scan`]: per-channel max-RSSI survey;
//! - [`OpenThread::create_new_network_dataset`] (`ftd` only): generate an
//!   Operational Dataset with random security parameters, tweak it (network
//!   name + the channel picked by the scan) and apply it.
//!
//! Needs the `ftd` feature (only a Full Thread Device can become Leader):
//! `cargo run --features ftd --bin form`
//!
//! Set the serial device with `RCP_SERIAL` (default `/dev/ttyACM0`) and the
//! baud rate with `RCP_BAUD` (default 115200).

use embassy_executor::{Executor, Spawner};

use log::{info, warn};

use openthread::spinel::{
    SerialPort, SpinelRadio, SpinelRadioResources, UartSpinelTransport, UartTransportResources,
};
use openthread::{Channels, DeviceRole, OpenThread, OtResources, SimpleRamSettings};

use rand::rngs::StdRng;
use rand::{RngCore, SeedableRng};

use static_cell::{ConstStaticCell, StaticCell};

// Linked for its `utoa`/`strtoul` C symbols, which OpenThread's C references.
use tinyrlibc as _;

const DEFAULT_SERIAL: &str = "/dev/ttyACM0";
const DEFAULT_BAUD: u32 = 115_200;

/// Energy-scan dwell time per channel, in milliseconds.
const SCAN_DURATION_MILLIS: u16 = 200;

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

    // Bring the interface up (Thread itself stays disabled while we scan).
    ot.enable_ipv6(true).unwrap();

    // Survey all channels and pick the quietest one (lowest max RSSI).
    info!("Energy-scanning all channels ({SCAN_DURATION_MILLIS} ms per channel)...");

    let mut quietest: Option<(u8, i8)> = None;
    ot.energy_scan(Channels::all(), SCAN_DURATION_MILLIS, |result| {
        if let Some(result) = result {
            info!(
                "  channel {:2}: max RSSI {} dBm",
                result.channel, result.max_rssi
            );

            if quietest.is_none_or(|(_, rssi)| result.max_rssi < rssi) {
                quietest = Some((result.channel, result.max_rssi));
            }
        }
    })
    .await
    .unwrap();

    // A radio that cannot measure channel energy yields an *empty* scan (see
    // `Radio::energy_scan`); fall back to the random channel picked by
    // `create_new_network_dataset` in that case.
    let channel = quietest.map(|(channel, rssi)| {
        info!("Quietest channel: {channel} (max RSSI {rssi} dBm)");
        channel
    });

    if channel.is_none() {
        warn!("Energy scan produced no measurements; keeping the random channel");
    }

    // Form a new network on that channel: random security parameters from
    // OpenThread, our own network name, the surveyed channel.
    ot.create_new_network_dataset(|dataset| {
        let mut dataset = dataset.clone();
        dataset.network_name = Some("OT-RS-FORM");
        if let Some(channel) = channel {
            dataset.channel = Some(channel as u16);
        }

        info!(
            "Forming network {:?}, PAN ID 0x{:04x}, channel {:?}",
            dataset.network_name,
            dataset.pan_id.unwrap_or(0),
            dataset.channel,
        );

        ot.set_active_dataset(&dataset)
    })
    .unwrap()
    .unwrap();

    ot.enable_thread(true).unwrap();

    // Wait to become the Leader of the new network, then report.
    loop {
        let status = ot.net_status();

        info!(
            "Role: {:?}, ext PAN ID: {:x?}",
            status.role, status.ext_pan_id
        );

        if matches!(status.role, DeviceRole::Leader) {
            info!("This node is now the Leader of the newly-formed network");

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
