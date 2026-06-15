//! An example for NRF, demonstrating the usage of OpenThread's DNS client API.
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
//!
//! See README.md for instructions on how to configure the other Thread peer (a FTD / Border
//! Router).

#![no_std]
#![no_main]

use defmt::info;

use embedded_alloc::LlffHeap as Heap;

use embassy_executor::InterruptExecutor;
use embassy_executor::Spawner;

use embassy_nrf::interrupt;
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::mode::Blocking;
use embassy_nrf::rng::Rng;
use embassy_nrf::{bind_interrupts, peripherals, radio};

use embassy_time::{Duration, Timer};

use openthread::nrf::{Ieee802154, NrfRadio};
use openthread::sys::otRadioCaps;
use openthread::{
    DnsResponse, EmbassyTimeTimer, OpenThread, OtResources, OtUdpResources, PhyRadioRunner,
    ProxyRadio, ProxyRadioResources, Radio, SimpleRamSettings,
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
        let x = STATIC_CELL.uninit().write($val);
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
    "0e080000000000010000000300000b35060004001fffe002083a90e3a319a904940708fd1fa298dbd1e3290510fe0458f7db96354eaa6041b880ea9c0f030f4f70656e5468726561642d35386431010258d10410888f813c61972446ab616ee3c556a5910c0402a0f7f8"
};

const NRF_RADIO_CAPS: otRadioCaps = NrfRadio::CAPS.bits();

// Only needed for tinyrlibc's alloc functions which won't be called at runtime.
#[global_allocator]
static HEAP: Heap = Heap::empty();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;

    let p = embassy_nrf::init(config);

    rtt_target::rtt_init_defmt!();

    info!("Starting...");

    let rng = mk_static!(Rng<'static, Blocking>, Rng::new_blocking(p.RNG));

    let mut ieee_eui64 = [0; 8];
    RngCore::fill_bytes(rng, &mut ieee_eui64);

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

    info!("About to spawn OT runner");

    let radio = NrfRadio::new(Ieee802154::new(p.RADIO, Irqs));

    let proxy_radio_resources = mk_static!(ProxyRadioResources, ProxyRadioResources::new());
    let (proxy_radio, phy_radio_runner) = ProxyRadio::new(proxy_radio_resources);

    // High-priority executor: EGU0_SWI0, priority level 7
    interrupt::EGU0_SWI0.set_priority(Priority::P7);

    let spawner_high = EXECUTOR_HIGH.start(interrupt::EGU0_SWI0);
    spawner_high.spawn(run_radio(phy_radio_runner, radio).unwrap());

    info!("Radio created");

    spawner.spawn(run_ot(ot.clone(), proxy_radio).unwrap());

    info!("Dataset: {}", THREAD_DATASET);

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

    // Wait until the device is attached (has a mesh-local / non-link-local address).
    info!("Waiting for the device to connect to the Thread network...");
    loop {
        let mut connected = false;
        ot.ipv6_addrs(|addr| {
            if let Some((addr, _prefix)) = addr {
                // A unicast, non-link-local address indicates we are attached.
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
        info!("Resolving `{}` (regular DNS, AAAA)...", DNS_HOST_NAME);

        let result = ot
            .dns_resolve_address(DNS_HOST_NAME, None, |response| {
                let DnsResponse::Address(response) = response else {
                    return;
                };

                let mut index = 0;
                loop {
                    match response.address(index) {
                        Ok(Some((addr, ttl))) => {
                            info!("  {} -> {} (ttl {})", DNS_HOST_NAME, addr, ttl);
                            index += 1;
                        }
                        Ok(None) => break,
                        Err(e) => {
                            info!("  Error reading address {}: {:?}", index, e);
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
            info!("Address resolution failed: {:?}", e);
        }

        // --- 2. DNS-SD: browse for service instances of a given type. ---
        info!("Browsing `{}` (DNS-SD)...", DNSSD_SERVICE_TYPE);

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
                    // The instance label getter needs its own buffer; the service-info
                    // getter then takes the label (as a `CStr`) plus the host/TXT buffers.
                    let label = match response.service_instance(index, &mut label_buf) {
                        Ok(Some(label)) => label,
                        Ok(None) => break,
                        Err(e) => {
                            info!("  Error reading instance {}: {:?}", index, e);
                            break;
                        }
                    };

                    info!("  Instance: {}", label);

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
                        Err(e) => info!("    (no service info: {:?})", e),
                    }

                    index += 1;
                }

                if index == 0 {
                    info!("  No service instances found");
                }
            })
            .await;

        if let Err(e) = result {
            info!("Browse failed: {:?}", e);
        }

        Timer::after(Duration::from_secs(10)).await;
    }
}

#[embassy_executor::task]
async fn run_ot(ot: OpenThread<'static>, radio: ProxyRadio<'static, NRF_RADIO_CAPS>) -> ! {
    ot.run(radio).await
}

#[embassy_executor::task]
async fn run_radio(mut runner: PhyRadioRunner<'static>, radio: NrfRadio<'static>) -> ! {
    runner
        .run(
            radio,
            EmbassyTimeTimer, /*TODO: Likely not precise enough*/
        )
        .await
}
