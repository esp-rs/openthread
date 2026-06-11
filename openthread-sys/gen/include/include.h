// Headers fed to `bindgen` to generate the raw `openthread-sys` bindings.
//
// This list is the *binding* surface, NOT the *link* surface: every public
// OpenThread API header is included here so that Rust bindings exist for it,
// regardless of which cargo features are enabled. The cargo features (and the
// `OT_*` CMake knobs they map to, see `gen/features.rs`) only control which
// implementations are compiled into the static `.a` libraries; a binding for a
// not-compiled-in feature simply has nothing to link against until its feature
// is enabled. Declarations are cheap and `--gc-sections` drops the unused ones,
// so we surface them all rather than hand-maintaining a per-feature subset
// (which previously left most feature flags with no callable Rust API at all).
//
// Keep this in sync with the public headers under `openthread/include/openthread`.

// Core
#include "openthread/instance.h"
#include "openthread/error.h"
#include "openthread/tasklet.h"
#include "openthread/thread.h"
#include "openthread/thread_ftd.h"
#include "openthread/link.h"
#include "openthread/link_raw.h"
#include "openthread/message.h"
#include "openthread/dataset.h"
#include "openthread/dataset_ftd.h"
#include "openthread/dataset_updater.h"
#include "openthread/multi_radio.h"
#include "openthread/radio_stats.h"
#include "openthread/network_time.h"
#include "openthread/verhoeff_checksum.h"
#include "openthread/heap.h"
#include "openthread/logging.h"
#include "openthread/config.h"

// IPv6 / addressing
#include "openthread/ip6.h"
#include "openthread/icmp6.h"
#include "openthread/udp.h"
#include "openthread/tcp.h"
#include "openthread/tcp_ext.h"
#include "openthread/nat64.h"

// Network data / service discovery / naming
#include "openthread/netdata.h"
#include "openthread/netdata_publisher.h"
#include "openthread/server.h"
#include "openthread/srp_client_buffers.h"
#include "openthread/srp_client.h"
#include "openthread/srp_server.h"
#include "openthread/dns.h"
#include "openthread/dns_client.h"
#include "openthread/dnssd_server.h"
#include "openthread/mdns.h"

// Transport / application protocols
#include "openthread/coap.h"
#include "openthread/coap_secure.h"
#include "openthread/sntp.h"

// Commissioning / security
#include "openthread/commissioner.h"
#include "openthread/joiner.h"
#include "openthread/crypto.h"
#include "openthread/random_crypto.h"
#include "openthread/random_noncrypto.h"
#include "openthread/ble_secure.h"
#include "openthread/tcat.h"
#include "openthread/border_agent.h"

// Border router / routing / backbone
#include "openthread/border_router.h"
#include "openthread/border_routing.h"
#include "openthread/backbone_router.h"
#include "openthread/backbone_router_ftd.h"

// Diagnostics / management
#include "openthread/ping_sender.h"
#include "openthread/link_metrics.h"
#include "openthread/jam_detection.h"
#include "openthread/child_supervision.h"
#include "openthread/mesh_diag.h"
#include "openthread/netdiag.h"
#include "openthread/history_tracker.h"
#include "openthread/channel_manager.h"
#include "openthread/channel_monitor.h"

// Misc
#include "openthread/trel.h"

// Platform callbacks implemented by this crate
#include "openthread/platform/alarm-milli.h"
#include "openthread/platform/radio.h"
#include "openthread/platform/misc.h"
#include "openthread/platform/entropy.h"
#include "openthread/platform/settings.h"
#include "openthread/platform/logging.h"

#ifndef OPENTHREAD_CONFIG_SRP_CLIENT_AUTO_START_API_ENABLE
#define OPENTHREAD_CONFIG_SRP_CLIENT_AUTO_START_API_ENABLE 1
#endif
