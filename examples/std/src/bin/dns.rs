//! Host example driving a remote OpenThread RCP over serial, demonstrating the
//! OpenThread DNS client:
//!
//! 1. A regular DNS `AAAA` resolution of a host name to its IPv6 address(es).
//! 2. A DNS-SD browse for all `_matter._tcp` service instances registered with
//!    the Thread network's SRP/DNS-SD server.
//!
//! NOTE: OpenThread's DNS client is a *unicast* resolver that queries the
//! SRP/DNS-SD server discovered via `srp_autostart()`, in the
//! `default.service.arpa` domain — NOT mDNS's `local`.
//!
//! Set the serial device with `RCP_SERIAL` (default `/dev/ttyACM0`), the baud
//! rate with `RCP_BAUD` (default 115200; e.g. 460800 for ESP32XX RCPs) and,
//! optionally, `THREAD_DATASET` (hex TLV).

use embassy_executor::{Executor, Spawner};
use embassy_time::{Duration, Timer};

use log::info;

use openthread::spinel::{
    SerialPort, SpinelRadio, SpinelRadioResources, UartSpinelTransport, UartTransportResources,
};
use openthread::{DnsResponse, OpenThread, OtResources, OtUdpResources, SimpleRamSettings};

use rand::rngs::StdRng;
use rand::{RngCore, SeedableRng};

use static_cell::{ConstStaticCell, StaticCell};

// Linked for its `utoa`/`strtoul` C symbols, which OpenThread's C references.
use tinyrlibc as _;

const UDP_SOCKETS_BUF: usize = 1280;
const UDP_MAX_SOCKETS: usize = 2;

const DEFAULT_SERIAL: &str = "/dev/ttyACM0";
const DEFAULT_BAUD: u32 = 115_200;

const DNS_HOST_NAME: &str = "google.com";
const DNSSD_SERVICE_TYPE: &str = "_matter._tcp.default.service.arpa";

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
    static OT_UDP_RESOURCES: StaticCell<OtUdpResources<UDP_MAX_SOCKETS, UDP_SOCKETS_BUF>> =
        StaticCell::new();
    static OT_SETTINGS_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    static OT_SETTINGS: StaticCell<SimpleRamSettings> = StaticCell::new();

    let ot_resources = OT_RESOURCES.init(OtResources::new());
    let ot_udp_resources = OT_UDP_RESOURCES.init(OtUdpResources::new());
    let ot_settings_buf = OT_SETTINGS_BUF.init([0; 1024]);
    let ot_settings = OT_SETTINGS.init(SimpleRamSettings::new(ot_settings_buf));

    // Pure DNS *client*: no SRP resources needed. We still `srp_autostart()`
    // below (works without SRP resources) so the DNS client auto-discovers its
    // server.
    let ot = OpenThread::new_with_udp(ieee_eui64, rng, ot_settings, ot_resources, ot_udp_resources)
        .unwrap();

    // The radio/transport buffers, in `const`-constructed statics (`.bss`), so
    // they never travel through the stack.
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

    // Enables the mechanism by which the DNS client auto-discovers the server to
    // query (it adopts the SRP/DNS-SD server address SRP auto-start selects from
    // network data). Without it, queries would need an explicit server.
    ot.srp_autostart().unwrap();

    ot.set_active_dataset_tlv_hexstr(THREAD_DATASET).unwrap();
    ot.enable_ipv6(true).unwrap();
    ot.enable_thread(true).unwrap();

    // Wait until attached (a unicast, non-link-local address).
    info!("Waiting for the device to connect to the Thread network...");
    loop {
        let mut connected = false;
        ot.ipv6_addrs(|addr| {
            if let Some((addr, _prefix)) = addr {
                if !addr.is_unicast_link_local() && !addr.is_loopback() {
                    connected = true;
                }
            }
            Ok(())
        })
        .unwrap();

        if connected {
            break;
        }

        ot.wait_changed().await;
    }

    info!("Connected. Starting DNS queries...");

    loop {
        // --- 1. Regular DNS: resolve a host name to its IPv6 address(es). ---
        info!("Resolving `{DNS_HOST_NAME}` (regular DNS, AAAA)...");

        let result = ot
            .dns_resolve_address(DNS_HOST_NAME, None, |response| {
                let DnsResponse::Address(response) = response else {
                    return;
                };

                let mut index = 0;
                loop {
                    match response.address(index) {
                        Ok(Some((addr, ttl))) => {
                            info!("  {DNS_HOST_NAME} -> {addr} (ttl {ttl})");
                            index += 1;
                        }
                        Ok(None) => break,
                        Err(e) => {
                            info!("  Error reading address {index}: {e:?}");
                            break;
                        }
                    }
                }

                if index == 0 {
                    info!("  No addresses returned");
                }
            })
            .await;

        if let Err(e) = result {
            info!("Address resolution failed: {e:?}");
        }

        // --- 2. DNS-SD: browse for service instances of a given type. ---
        info!("Browsing `{DNSSD_SERVICE_TYPE}` (DNS-SD)...");

        let result = ot
            .dns_browse(DNSSD_SERVICE_TYPE, None, |response| {
                let DnsResponse::Browse(response) = response else {
                    return;
                };

                let mut label_buf = [0u8; 64];
                let mut host_buf = [0u8; 128];
                let mut txt_buf = [0u8; 256];

                let mut index = 0;
                loop {
                    let label = match response.service_instance(index, &mut label_buf) {
                        Ok(Some(label)) => label,
                        Ok(None) => break,
                        Err(e) => {
                            info!("  Error reading instance {index}: {e:?}");
                            break;
                        }
                    };

                    info!("  Instance: {label}");

                    match response.service_info(label, &mut host_buf, &mut txt_buf) {
                        Ok(info) => {
                            info!(
                                "    port {}, host {:?}, addr {:?}, txt {} bytes",
                                info.port,
                                info.host_name,
                                info.host_address,
                                info.txt_data.map(|t| t.len()).unwrap_or(0),
                            );
                        }
                        Err(e) => info!("    (no service info: {e:?})"),
                    }

                    index += 1;
                }

                if index == 0 {
                    info!("  No service instances found");
                }
            })
            .await;

        if let Err(e) = result {
            info!("Browse failed: {e:?}");
        }

        Timer::after(Duration::from_secs(10)).await;
    }
}

#[embassy_executor::task]
async fn run_ot(
    ot: OpenThread<'static>,
    radio: SpinelRadio<'static, UartSpinelTransport<'static, SerialPort>>,
) -> ! {
    ot.run(radio).await
}
