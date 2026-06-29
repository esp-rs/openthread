//! An example for esp32-c6 and esp32-h2, demonstrating the usage of OpenThread's DNS client API.
//!
//! The example provisions an MTD device with fixed Thread network settings, waits for the
//! device to connect, and then - once operational - periodically issues two kinds of DNS
//! queries through OpenThread's (unicast) DNS client, demonstrating that the same API serves
//! both *regular DNS* and *DNS-SD*:
//!
//! 1. A regular DNS address (AAAA) resolution of `google.com` -> IPv6 address(es).
//!    This requires the Thread Border Router to have upstream Internet connectivity and a
//!    usable recursive DNS server configured for the DNS client (see the default-server config
//!    knobs in OpenThread, or set one explicitly via `DnsQueryConfig`).
//!
//! 2. A DNS-SD browse for all `_matter._tcp` service instances registered with the Thread
//!    network's SRP/DNS-SD server.
//!
//!    NOTE: OpenThread's DNS client is a *unicast* resolver that queries the SRP/DNS-SD server
//!    on the Thread Border Router, which serves the Thread network domain
//!    `default.service.arpa` - NOT mDNS's `local` domain. So the browse targets
//!    `_matter._tcp.default.service.arpa`, not `_matter._tcp.local`. (You can register a
//!    matching service from another peer using the `srp` example.)

#![no_std]
#![no_main]

use log::info;

use embassy_executor::Spawner;

use embassy_time::{Duration, Timer};

use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ieee802154::Ieee802154;
use {esp_backtrace as _, esp_println as _};

use openthread::esp::EspRadio;
use openthread::{
    DnsResponse, OpenThread, OtResources, OtRngCore, OtUdpResources, SimpleRamSettings,
};

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
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

const UDP_SOCKETS_BUF: usize = 1280;
const UDP_MAX_SOCKETS: usize = 2;

/// The regular-DNS host name to resolve to an IPv6 address.
const DNS_HOST_NAME: &str = "google.com";

/// The DNS-SD service type to browse for, in the Thread network domain
/// (`default.service.arpa`, served by the Border Router's SRP/DNS-SD server).
const DNSSD_SERVICE_TYPE: &str = "_matter._tcp.default.service.arpa";

const THREAD_DATASET: &str = if let Some(dataset) = option_env!("THREAD_DATASET") {
    dataset
} else {
    "000300001901020fd80208b566147d38e384200e080000639c5d67a3bd0510c490f58d4be0d5eaeb0f09b395d1ae17030d4e4553542d50414e2d304644380708fd7d4f8232cb00000410a7e08419ae47c177fb91bcfcec789aa50c0402a0f77835060004001fffe0"
};

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_alloc::heap_allocator!(size: 1024);

    esp_println::logger::init_logger_from_env();

    info!("Starting...");

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT)
            .software_interrupt0,
    );

    // TODO: Use TRNG?
    let rng = mk_static!(Rng, Rng::new());

    let mut ieee_eui64 = [0; 8];
    rng.fill_bytes(&mut ieee_eui64);

    let ot_resources = mk_static!(OtResources, OtResources::new());
    let ot_udp_resources =
        mk_static!(OtUdpResources<UDP_MAX_SOCKETS, UDP_SOCKETS_BUF>, OtUdpResources::new());
    let ot_settings_buf = mk_static!([u8; 1024], [0; 1024]);

    let ot_settings = mk_static!(SimpleRamSettings, SimpleRamSettings::new(ot_settings_buf));

    // No SRP resources are needed: this example is a pure DNS *client*. We still
    // call `srp_autostart()` below (it works without SRP resources) so the DNS
    // client can auto-discover its server - see the comment there.
    let ot = OpenThread::new_with_udp(ieee_eui64, rng, ot_settings, ot_resources, ot_udp_resources)
        .unwrap();

    spawner.spawn(
        run_ot(
            ot.clone(),
            EspRadio::new(Ieee802154::new(peripherals.IEEE802154)),
        )
        .unwrap(),
    );

    info!("Dataset: {THREAD_DATASET}");

    // This example is a pure DNS *client* - it never registers anything over SRP.
    // We nonetheless enable SRP auto-start because that is the mechanism by which
    // OpenThread's DNS client auto-discovers the server to query: it adopts the
    // SRP/DNS-SD server address that SRP auto-start selects from network data
    // (see `otDnsClientGetDefaultConfig` / OpenThread's `UpdateDefaultConfigAddress`).
    // Without this, DNS queries would have no default server; you would then have
    // to pass an explicit server via `DnsQueryConfig::server`.
    ot.srp_autostart().unwrap();

    ot.set_active_dataset_tlv_hexstr(THREAD_DATASET).unwrap();
    ot.enable_ipv6(true).unwrap();
    ot.enable_thread(true).unwrap();

    // Wait until the device is attached (has a unicast, non-link-local address).
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
async fn run_ot(ot: OpenThread<'static>, radio: EspRadio<'static>) -> ! {
    ot.run(radio).await
}
