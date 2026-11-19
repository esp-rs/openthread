//! A coexistence example: the `nrf-802154` radio driving OpenThread (attach +
//! SRP) **while a BLE peripheral (SoftDevice Controller + `trouble-host`) is
//! advertising concurrently**, both sharing a single MPSL instance.
//!
//! This is the scenario a Matter-over-Thread device hits: commissioning happens
//! over BLE, and the Thread radio must attach and register SRP while BLE traffic
//! is still contending for radio time. The plain `srp` example runs the radio
//! alone (no BLE); this one exercises the driver under real MPSL BLE/802.15.4
//! time-slice arbitration.
//!
//! Expected healthy behavior: the log shows the device attaching (`Role ...
//! child`), the SRP host reaching `Registered`, AND the BLE peripheral
//! advertising the whole time. If Thread cannot attach here but attaches in the
//! plain `srp` example, the coex path (MPSL arbitration / wiring) is the culprit.
//!
//! Uses the same fixed Thread dataset as `srp.rs`; see that example for how to
//! bring up the peer Thread Border Router.

#![no_std]
#![no_main]

use core::fmt::Write as _;
use core::net::{Ipv6Addr, SocketAddrV6};

use defmt::{info, warn};

use embassy_executor::Spawner;
use embassy_futures::select::select;

use embassy_nrf::mode::Blocking;
use embassy_nrf::rng::Rng;

use embassy_time::{Duration, Timer};

use embedded_alloc::LlffHeap as Heap;

use nrf_802154::Radio;
use nrf_mpsl::raw::mpsl_clock_lfclk_cfg_t;
use nrf_mpsl::MultiprotocolServiceLayer;

use nrf_sdc::{self as sdc, Mem as SdcMem, Peripherals as SdcPeripherals};

use openthread::{
    BytesFmt, OpenThread, OtResources, OtSrpResources, OtUdpResources, Radio as _,
    SimpleRamSettings, SrpConf, UdpSocket,
};
use openthread_driver_nrf::NrfRadio;
use openthread_examples_nrf::Irqs;

use rand_core::RngCore;

use static_cell::StaticCell;

use tinyrlibc as _;

use trouble_host::prelude::*;

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

/// SDC scratch memory (BLE peripheral, single connection).
const SDC_MEM: usize = 3312;

/// trouble-host: 1 connection, small L2CAP.
const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2;

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

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = openthread_examples_nrf::init();

    info!("Starting (802.15.4 + BLE coex)...");

    let lfclk_cfg = mpsl_clock_lfclk_cfg_t {
        source: nrf_mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: nrf_mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: nrf_mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };

    // MPSL owns RTC0, TIMER0, TEMP and PPI 19/30/31.
    let mpsl_p = openthread_examples_nrf::mpsl_peripherals!(p);
    let mpsl = MPSL.init(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    spawner.spawn(mpsl_task(mpsl).unwrap());

    let rng = mk_static!(Rng<'static, Blocking>, Rng::new_blocking(p.RNG));

    let mut ieee_eui64 = [0; 8];
    RngCore::fill_bytes(rng, &mut ieee_eui64);

    let random_srp_suffix: u32 = rng.next_u32();

    // The BLE SoftDevice Controller and OpenThread each hold a mutable RNG for
    // their whole lifetime, but there is a single TRNG peripheral. Seed a small
    // software CSPRNG from the TRNG for the SDC (BLE crypto quality is irrelevant
    // for this coex test), and keep the hardware TRNG for OpenThread.
    let sdc_rng = mk_static!(SoftRng, SoftRng::seed_from(rng));

    // --- Build the BLE SoftDevice Controller on the SAME MPSL ---
    // SDC owns PPI 17/18/20..=29.
    let sdc_p = SdcPeripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
        p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );

    let sdc_mem = mk_static!(SdcMem<SDC_MEM>, SdcMem::new());
    let sdc = sdc::Builder::new()
        .unwrap()
        .support_adv()
        .support_peripheral()
        .build(sdc_p, sdc_rng, mpsl, sdc_mem)
        .unwrap();

    // --- Build the 802.15.4 radio on the SAME MPSL ---
    // Radio owns EGU0, TIMER2, RTC2.
    let radio = Radio::new(
        p.RADIO,
        openthread_examples_nrf::radio_peripherals!(p),
        Irqs,
        mpsl,
    );
    let mut radio = NrfRadio::new(radio);
    let radio_caps = radio.init().await.unwrap();

    // --- OpenThread ---
    let ot_resources = mk_static!(OtResources, OtResources::new());
    let ot_udp_resources =
        mk_static!(OtUdpResources<UDP_MAX_SOCKETS, UDP_SOCKETS_BUF>, OtUdpResources::new());
    let ot_srp_resources =
        mk_static!(OtSrpResources<SRP_MAX_SERVICES, SRP_SERVICE_BUF>, OtSrpResources::new());
    let ot_settings_buf = mk_static!([u8; 1024], [0; 1024]);
    let ot_settings = mk_static!(SimpleRamSettings, SimpleRamSettings::new(ot_settings_buf));

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

    spawner.spawn(run_ot(ot.clone(), radio).unwrap());
    spawner.spawn(run_ot_info(ot.clone()).unwrap());

    info!("Dataset: {}", THREAD_DATASET);

    ot.srp_autostart().unwrap();
    ot.set_active_dataset_tlv_hexstr(THREAD_DATASET).unwrap();
    ot.enable_ipv6(true).unwrap();
    ot.enable_thread(true).unwrap();

    let mut hostname = heapless::String::<32>::new();
    write!(hostname, "coex-example-{random_srp_suffix:04x}").unwrap();

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
    write!(servicename, "coex{random_srp_suffix:04x}").unwrap();

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

    info!("Opened UDP socket on port {}", BOUND_PORT);

    // Run: BLE (advertise forever) concurrently with the Thread UDP echo loop.
    let ble_addr = Address::random([0x41, 0x5a, 0xe3, 0x1e, 0x83, 0xe7]);
    let ble = pin_ble(sdc, ble_addr);
    let udp = udp_echo(&socket);

    select(ble, udp).await;
}

/// Run the BLE peripheral: keep `trouble-host` alive and re-advertise forever,
/// so the SoftDevice Controller is continuously requesting MPSL time alongside
/// the 802.15.4 radio.
async fn pin_ble(controller: sdc::SoftdeviceController<'_>, address: Address) -> ! {
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    // trouble-host 0.7: `build()` returns the `Stack` itself; the peripheral,
    // central and runner are obtained via accessor methods that borrow it
    // (previously `build()` returned a destructurable `Host { .. }`).
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .build();
    let mut peripheral = stack.peripheral();
    let mut runner = stack.runner();

    let ble_bg = async {
        loop {
            if let Err(e) = runner.run().await {
                warn!("BLE runner error: {:?}", defmt::Debug2Format(&e));
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };

    let advertise = async {
        let adv_data = [
            0x02, 0x01, 0x06, // flags
            0x0b, 0x09, b'n', b'r', b'f', b'-', b'c', b'o', b'e', b'x', b'!', b'!',
        ];
        loop {
            info!("BLE: advertising...");
            let advertiser = match peripheral
                .advertise(
                    &Default::default(),
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &adv_data,
                        scan_data: &[],
                    },
                )
                .await
            {
                Ok(a) => a,
                Err(e) => {
                    warn!("BLE: advertise setup error: {:?}", defmt::Debug2Format(&e));
                    Timer::after(Duration::from_secs(1)).await;
                    continue;
                }
            };

            match advertiser.accept().await {
                Ok(conn) => {
                    info!("BLE: central connected");
                    // Hold the connection until it drops, then re-advertise.
                    let _ = conn.next().await;
                    info!("BLE: central disconnected");
                }
                Err(e) => {
                    warn!("BLE: accept error: {:?}", defmt::Debug2Format(&e));
                    Timer::after(Duration::from_millis(500)).await;
                }
            }
        }
    };

    select(ble_bg, advertise).await;
    core::unreachable!()
}

/// A tiny SplitMix64-based software RNG, seeded once from the hardware TRNG.
/// Good enough to satisfy the SoftDevice Controller's `CryptoRng` bound in this
/// coexistence *test* (not for production BLE crypto).
struct SoftRng(u64);

impl SoftRng {
    fn seed_from(src: &mut impl RngCore) -> Self {
        Self(src.next_u64())
    }

    fn next(&mut self) -> u64 {
        // SplitMix64
        self.0 = self.0.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^ (z >> 31)
    }
}

impl RngCore for SoftRng {
    fn next_u32(&mut self) -> u32 {
        self.next() as u32
    }

    fn next_u64(&mut self) -> u64 {
        self.next()
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        for chunk in dest.chunks_mut(8) {
            let v = self.next().to_le_bytes();
            chunk.copy_from_slice(&v[..chunk.len()]);
        }
    }
}

impl rand_core::CryptoRng for SoftRng {}

async fn udp_echo(socket: &UdpSocket<'_>) -> ! {
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
    let mut cur_role = None;

    loop {
        let role = ot.device_role();

        let mut state = cur_state;
        ot.srp_conf(|_, new_state, _| {
            state = Some(new_state);
            Ok(())
        })
        .unwrap();

        if cur_role != Some(role) || cur_state != state {
            info!("Role: {:?}, SRP host state: {:?}", role, state);
            cur_role = Some(role);
            cur_state = state;
        }

        select(
            core::pin::pin!(ot.wait_changed()),
            core::pin::pin!(Timer::after(Duration::from_secs(2))),
        )
        .await;
    }
}
