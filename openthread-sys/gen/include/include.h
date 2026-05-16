#include "openthread/instance.h"
#include "openthread/udp.h"
#include "openthread/thread.h"
#include "openthread/tasklet.h"
#include "openthread/nat64.h"
#include "openthread/netdata.h"
#include "openthread/coap.h"

#include "openthread/platform/alarm-milli.h"
#include "openthread/platform/radio.h"
#include "openthread/platform/misc.h"
#include "openthread/platform/entropy.h"
#include "openthread/platform/settings.h"
#include "openthread/platform/logging.h"

#include "openthread/srp_client_buffers.h"
#include "openthread/srp_client.h"

#ifndef OPENTHREAD_CONFIG_SRP_CLIENT_AUTO_START_API_ENABLE
#define OPENTHREAD_CONFIG_SRP_CLIENT_AUTO_START_API_ENABLE 1
#endif

#ifndef OPENTHREAD_CONFIG_COAP_API_ENABLE
#define OPENTHREAD_CONFIG_COAP_API_ENABLE 1
#endif

#ifndef OPENTHREAD_CONFIG_COAP_OBSERVE_API_ENABLE
#define OPENTHREAD_CONFIG_COAP_OBSERVE_API_ENABLE 1
#endif