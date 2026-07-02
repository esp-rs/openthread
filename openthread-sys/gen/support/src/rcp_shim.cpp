/*
 *  RCP-host spinel bridge shim.
 *
 *  Bridges OpenThread's C++ `RadioSpinel` client to the Rust `SpinelTransport`
 *  supplied by the `openthread` crate, so the full OpenThread stack can run on
 *  this MCU while the 802.15.4 radio lives on a *remote* RCP chip reached over a
 *  UART/SPI link (the spinel protocol).
 *
 *  Compiled only when the `rcp` Cargo feature is enabled (see the support
 *  `CMakeLists.txt`). It provides:
 *
 *    - `RustSpinelInterface`: a `Spinel::SpinelInterface` subclass that carries
 *      spinel frames over an in-memory RX/TX byte path instead of a POSIX file
 *      descriptor. HDLC framing is done here (mirroring the POSIX
 *      `HdlcInterface`); the actual bytes are moved to/from the remote RCP by
 *      the Rust async pump (`OpenThread::run_rcp`).
 *
 *    - The `otPlatRadio*` platform callbacks, implemented by forwarding to the
 *      `RadioSpinel` instance (mirroring OpenThread's POSIX
 *      `src/posix/platform/radio.cpp`). In `rcp` builds the Rust
 *      `otPlatRadio*` implementations in `platform.rs` are `#[cfg]`'d out, so
 *      these are the ones that link.
 *
 *    - `extern "C"` entry points (`otRcp*`) used by the Rust side to construct,
 *      pump, and feed the spinel stack.
 *
 *  The RX/TX design (why not a POSIX-style blocking fd):
 *
 *    - The Rust pump reads bytes from the transport and hands them to
 *      `otRcpReceive()` (which HDLC-decodes into the `RxFrameBuffer`), then calls
 *      `otRcpProcess()` (which runs `RadioSpinel::Process`, consuming decoded
 *      frames). `SendFrame()` HDLC-encodes into a TX buffer (`otRcpHostEnqueueTx`)
 *      that the pump drains and writes to the transport.
 *
 *    - `SpinelInterface::WaitForFrame()` is the synchronous request/response
 *      primitive: `RadioSpinel`/`SpinelDriver` call it (via `WaitResponse`)
 *      whenever they issue a spinel command and must block for its matching
 *      reply frame. This happens both during bring-up (reset + version) AND
 *      during normal operation (per-transmit acks, property get/set) — NOT only
 *      at setup. Received frames that are not the awaited reply (unsolicited RX
 *      packets, transmit-done) are dispatched via the async callbacks while the
 *      wait loops. Our `WaitForFrame` calls back into Rust (`otRcpHostPumpRx`),
 *      which `block_on`s a bounded transport read and feeds the bytes back via
 *      `otRcpReceive()`. This mirrors OpenThread's POSIX `HdlcInterface`, whose
 *      `WaitForFrame` blocks on `select()`; the whole model is single-threaded
 *      and synchronous, so on the single-threaded embassy target this blocking
 *      is the intended design (safe as long as the pump is the sole transport
 *      driver).
 */

#include <stdint.h>
#include <string.h>

#include <openthread/error.h>
#include <openthread/instance.h>
#include <openthread/platform/radio.h>
#include <openthread/platform/time.h>

#include "lib/hdlc/hdlc.hpp"
#include "lib/spinel/radio_spinel.hpp"
#include "lib/spinel/spinel.h"
#include "lib/spinel/spinel_driver.hpp"
#include "lib/spinel/spinel_interface.hpp"

namespace {

using ot::Spinel::FrameBuffer;
using ot::Spinel::RadioSpinel;
using ot::Spinel::RadioSpinelCallbacks;
using ot::Spinel::SpinelDriver;
using ot::Spinel::SpinelInterface;

// Max spinel/HDLC frame size (matches `SpinelInterface::kMaxFrameSize`).
constexpr uint16_t kMaxFrameSize = SpinelInterface::kMaxFrameSize;

// Radio capabilities the RCP is required to support. `0` = accept whatever the
// RCP reports (the Rust side passes `skipRcpCompatibilityCheck` accordingly).
constexpr otRadioCaps kRequiredRadioCaps = static_cast<otRadioCaps>(0);

} // namespace

// ---------------------------------------------------------------------------
// Rust-side hooks (implemented in `openthread/src/rcp.rs`).
// ---------------------------------------------------------------------------
extern "C" {

// Enqueue `aLength` bytes of an outgoing (already HDLC-encoded) spinel frame for
// the Rust pump to write to the transport. Called from `SendFrame`.
void otRcpHostEnqueueTx(const uint8_t *aBuf, uint16_t aLength);

// Synchronously pump the transport for up to `aTimeoutUs`, feeding any received
// bytes back via `otRcpReceive`. Returns `true` if at least one byte was
// received. Called from `WaitForFrame` (i.e. on every synchronous spinel
// request/response — bring-up and per-command acks/gets/sets). Implemented on
// the Rust side with a bounded `block_on(transport.read(..))`.
bool otRcpHostPumpRx(uint64_t aTimeoutUs);

} // extern "C"

// ---------------------------------------------------------------------------
// RustSpinelInterface: a SpinelInterface over the in-memory RX/TX path.
// ---------------------------------------------------------------------------
namespace {

class RustSpinelInterface : public SpinelInterface
{
public:
    RustSpinelInterface(void)
        : mReceiveFrameCallback(nullptr)
        , mReceiveFrameContext(nullptr)
        , mReceiveFrameBuffer(nullptr)
    {
    }

    otError Init(ReceiveFrameCallback aCallback, void *aCallbackContext, RxFrameBuffer &aFrameBuffer) override
    {
        mHdlcDecoder.Init(aFrameBuffer, HandleHdlcFrame, this);
        mReceiveFrameCallback = aCallback;
        mReceiveFrameContext  = aCallbackContext;
        mReceiveFrameBuffer   = &aFrameBuffer;
        return OT_ERROR_NONE;
    }

    void Deinit(void) override
    {
        mReceiveFrameCallback = nullptr;
        mReceiveFrameContext  = nullptr;
        mReceiveFrameBuffer   = nullptr;
    }

    // HDLC-encode `aFrame` and hand the encoded bytes to the Rust pump for
    // transmission to the RCP.
    otError SendFrame(const uint8_t *aFrame, uint16_t aLength) override
    {
        otError                     error = OT_ERROR_NONE;
        FrameBuffer<kMaxFrameSize>  encoderBuffer;
        ot::Hdlc::Encoder           hdlcEncoder(encoderBuffer);

        SuccessOrExit(error = hdlcEncoder.BeginFrame());
        SuccessOrExit(error = hdlcEncoder.Encode(aFrame, aLength));
        SuccessOrExit(error = hdlcEncoder.EndFrame());

        otRcpHostEnqueueTx(encoderBuffer.GetFrame(), encoderBuffer.GetLength());

    exit:
        return error;
    }

    // Called whenever the driver has issued a spinel command and is blocking for
    // its matching reply frame (bring-up + every per-command ack/get/set — see
    // the file header). Pump the Rust transport synchronously for up to the
    // remaining budget; the pumped bytes are decoded via `Receive` (below),
    // which fires the frame-ready callback when a full frame lands.
    otError WaitForFrame(uint64_t aTimeoutUs) override
    {
        return otRcpHostPumpRx(aTimeoutUs) ? OT_ERROR_NONE : OT_ERROR_RESPONSE_TIMEOUT;
    }

    // POSIX mainloop hooks — unused in the async pump model.
    void UpdateFdSet(void *) override {}
    void Process(const void *) override {}

    uint32_t GetBusSpeed(void) const override { return mBusSpeed; }

    otError HardwareReset(void) override { return OT_ERROR_NOT_IMPLEMENTED; }

    const otRcpInterfaceMetrics *GetRcpInterfaceMetrics(void) const override { return &mMetrics; }

    // Feed raw bytes received from the transport into the HDLC decoder. A
    // complete decoded frame triggers `HandleHdlcFrame` -> the receive callback.
    void Receive(const uint8_t *aBuf, uint16_t aLength) { mHdlcDecoder.Decode(aBuf, aLength); }

    void SetBusSpeed(uint32_t aBusSpeed) { mBusSpeed = aBusSpeed; }

private:
    static void HandleHdlcFrame(void *aContext, otError aError)
    {
        static_cast<RustSpinelInterface *>(aContext)->HandleHdlcFrame(aError);
    }

    void HandleHdlcFrame(otError aError)
    {
        if ((mReceiveFrameCallback == nullptr) || (mReceiveFrameBuffer == nullptr))
        {
            return;
        }

        if (aError == OT_ERROR_NONE)
        {
            mReceiveFrameCallback(mReceiveFrameContext);
        }
        else
        {
            mReceiveFrameBuffer->DiscardFrame();
        }
    }

    ReceiveFrameCallback    mReceiveFrameCallback;
    void                   *mReceiveFrameContext;
    RxFrameBuffer          *mReceiveFrameBuffer;
    ot::Hdlc::Decoder       mHdlcDecoder;
    uint32_t                mBusSpeed = 115200;
    otRcpInterfaceMetrics   mMetrics  = {};
};

// Singletons. Constructed once by `otRcpInit`; live for the process lifetime.
static RustSpinelInterface sInterface;
static SpinelDriver        sSpinelDriver;
static RadioSpinel         sRadioSpinel;

} // namespace

// The `otPlatRadio*` forwarders below reach the radio via this accessor,
// mirroring POSIX `GetRadioSpinel()`.
static RadioSpinel &GetRadioSpinel(void) { return sRadioSpinel; }

// ---------------------------------------------------------------------------
// C entry points driven by the Rust side (`OpenThread::run_rcp`).
// ---------------------------------------------------------------------------
extern "C" {

// Construct + initialize the spinel stack (interface -> driver -> radio).
// Must be called BEFORE `otInstanceInitSingle()`, so that the `otPlatRadio*`
// calls made during instance init resolve against a live `RadioSpinel`.
//
// `aBusSpeed`             nominal transport bit/s (spinel timeout sizing).
// `aResetRadio`           request a software reset of the RCP during init.
// `aSkipCompatibilityCheck` skip the RCP spinel-version compatibility check.
otError otRcpInit(uint32_t aBusSpeed, bool aResetRadio, bool aSkipCompatibilityCheck)
{
    RadioSpinelCallbacks callbacks;

    sInterface.SetBusSpeed(aBusSpeed);

    // Wire RadioSpinel's notifications straight to the OpenThread platform
    // radio callbacks (identical to POSIX `radio.cpp`).
    memset(&callbacks, 0, sizeof(callbacks));
    callbacks.mReceiveDone       = otPlatRadioReceiveDone;
    callbacks.mTransmitDone      = otPlatRadioTxDone;
    callbacks.mEnergyScanDone    = otPlatRadioEnergyScanDone;
    callbacks.mTxStarted         = otPlatRadioTxStarted;
    callbacks.mBusLatencyChanged = otPlatRadioBusLatencyChanged;

    // Single interface ID (we are not a multipan host).
    spinel_iid_t iidList[] = {0};

    // `SpinelDriver::Init` calls `sInterface.Init(...)` and performs the reset
    // handshake, which pumps `WaitForFrame` -> `otRcpHostPumpRx`.
    sSpinelDriver.Init(sInterface, aResetRadio, iidList, /* aIidListLength */ 1);

    sRadioSpinel.SetCallbacks(callbacks);
    sRadioSpinel.Init(aSkipCompatibilityCheck, aResetRadio, &sSpinelDriver, kRequiredRadioCaps,
                      /* aEnableRcpTimeSync */ false);

    return OT_ERROR_NONE;
}

// Feed transport bytes received from the RCP into the spinel stack.
void otRcpReceive(const uint8_t *aBuf, uint16_t aLength) { sInterface.Receive(aBuf, aLength); }

// Run one non-blocking iteration of the spinel processing (consumes any
// decoded frames + advances the radio state machine). Called from the pump.
void otRcpProcess(void)
{
    sSpinelDriver.Process(nullptr);
    sRadioSpinel.Process(nullptr);
}

void otRcpDeinit(void)
{
    sRadioSpinel.Deinit();
    sSpinelDriver.Deinit();
    sInterface.Deinit();
}

} // extern "C"

// ---------------------------------------------------------------------------
// otPlatRadio* platform callbacks -> RadioSpinel.
//
// Transcribed from OpenThread's POSIX `src/posix/platform/radio.cpp`; each
// forwards to the `RadioSpinel` client which turns it into spinel traffic to
// the RCP. In non-`rcp` (SoC) builds these are provided in Rust by
// `platform.rs`; here they replace that set.
// ---------------------------------------------------------------------------
extern "C" {

void otPlatRadioGetIeeeEui64(otInstance *, uint8_t *aIeeeEui64) { IgnoreError(GetRadioSpinel().GetIeeeEui64(aIeeeEui64)); }

void otPlatRadioSetPanId(otInstance *, uint16_t aPanId) { IgnoreError(GetRadioSpinel().SetPanId(aPanId)); }

void otPlatRadioSetExtendedAddress(otInstance *, const otExtAddress *aAddress)
{
    otExtAddress addr;

    for (size_t i = 0; i < sizeof(addr); i++)
    {
        addr.m8[i] = aAddress->m8[sizeof(addr) - 1 - i];
    }

    IgnoreError(GetRadioSpinel().SetExtendedAddress(addr));
}

void otPlatRadioSetShortAddress(otInstance *, uint16_t aAddress) { IgnoreError(GetRadioSpinel().SetShortAddress(aAddress)); }

void otPlatRadioSetPromiscuous(otInstance *, bool aEnable) { IgnoreError(GetRadioSpinel().SetPromiscuous(aEnable)); }

bool otPlatRadioIsEnabled(otInstance *) { return GetRadioSpinel().IsEnabled(); }

otError otPlatRadioEnable(otInstance *aInstance) { return GetRadioSpinel().Enable(aInstance); }

otError otPlatRadioDisable(otInstance *) { return GetRadioSpinel().Disable(); }

otError otPlatRadioSleep(otInstance *) { return GetRadioSpinel().Sleep(); }

otError otPlatRadioReceive(otInstance *, uint8_t aChannel) { return GetRadioSpinel().Receive(aChannel); }

otError otPlatRadioTransmit(otInstance *, otRadioFrame *aFrame) { return GetRadioSpinel().Transmit(*aFrame); }

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *) { return &GetRadioSpinel().GetTransmitFrame(); }

int8_t otPlatRadioGetRssi(otInstance *) { return GetRadioSpinel().GetRssi(); }

otRadioCaps otPlatRadioGetCaps(otInstance *) { return GetRadioSpinel().GetRadioCaps(); }

const char *otPlatRadioGetVersionString(otInstance *) { return GetRadioSpinel().GetVersion(); }

bool otPlatRadioGetPromiscuous(otInstance *) { return GetRadioSpinel().IsPromiscuous(); }

void otPlatRadioEnableSrcMatch(otInstance *, bool aEnable) { IgnoreError(GetRadioSpinel().EnableSrcMatch(aEnable)); }

otError otPlatRadioAddSrcMatchShortEntry(otInstance *, uint16_t aShortAddress)
{
    return GetRadioSpinel().AddSrcMatchShortEntry(aShortAddress);
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *, const otExtAddress *aExtAddress)
{
    otExtAddress addr;

    for (size_t i = 0; i < sizeof(addr); i++)
    {
        addr.m8[i] = aExtAddress->m8[sizeof(addr) - 1 - i];
    }

    return GetRadioSpinel().AddSrcMatchExtEntry(addr);
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *, uint16_t aShortAddress)
{
    return GetRadioSpinel().ClearSrcMatchShortEntry(aShortAddress);
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *, const otExtAddress *aExtAddress)
{
    otExtAddress addr;

    for (size_t i = 0; i < sizeof(addr); i++)
    {
        addr.m8[i] = aExtAddress->m8[sizeof(addr) - 1 - i];
    }

    return GetRadioSpinel().ClearSrcMatchExtEntry(addr);
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *) { IgnoreError(GetRadioSpinel().ClearSrcMatchShortEntries()); }

void otPlatRadioClearSrcMatchExtEntries(otInstance *) { IgnoreError(GetRadioSpinel().ClearSrcMatchExtEntries()); }

otError otPlatRadioEnergyScan(otInstance *, uint8_t aScanChannel, uint16_t aScanDuration)
{
    return GetRadioSpinel().EnergyScan(aScanChannel, aScanDuration);
}

otError otPlatRadioGetTransmitPower(otInstance *, int8_t *aPower) { return GetRadioSpinel().GetTransmitPower(*aPower); }

otError otPlatRadioSetTransmitPower(otInstance *, int8_t aPower) { return GetRadioSpinel().SetTransmitPower(aPower); }

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *, int8_t *aThreshold)
{
    return GetRadioSpinel().GetCcaEnergyDetectThreshold(*aThreshold);
}

otError otPlatRadioSetCcaEnergyDetectThreshold(otInstance *, int8_t aThreshold)
{
    return GetRadioSpinel().SetCcaEnergyDetectThreshold(aThreshold);
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *) { return GetRadioSpinel().GetReceiveSensitivity(); }

#if OPENTHREAD_CONFIG_PLATFORM_RADIO_COEX_ENABLE
otError otPlatRadioGetCoexMetrics(otInstance *, otRadioCoexMetrics *aCoexMetrics)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(aCoexMetrics != nullptr, error = OT_ERROR_INVALID_ARGS);
    error = GetRadioSpinel().GetCoexMetrics(*aCoexMetrics);

exit:
    return error;
}

otError otPlatRadioSetCoexEnabled(otInstance *, bool aEnabled) { return GetRadioSpinel().SetCoexEnabled(aEnabled); }

bool otPlatRadioIsCoexEnabled(otInstance *) { return GetRadioSpinel().IsCoexEnabled(); }
#endif // OPENTHREAD_CONFIG_PLATFORM_RADIO_COEX_ENABLE

uint64_t otPlatRadioGetNow(otInstance *) { return GetRadioSpinel().GetNow(); }

uint32_t otPlatRadioGetBusSpeed(otInstance *) { return GetRadioSpinel().GetBusSpeed(); }

uint32_t otPlatRadioGetBusLatency(otInstance *) { return GetRadioSpinel().GetBusLatency(); }

otRadioState otPlatRadioGetState(otInstance *) { return OT_RADIO_STATE_INVALID; }

void otPlatRadioSetMacKey(otInstance             *,
                          uint8_t                 aKeyIdMode,
                          uint8_t                 aKeyId,
                          const otMacKeyMaterial *aPrevKey,
                          const otMacKeyMaterial *aCurrKey,
                          const otMacKeyMaterial *aNextKey,
                          otRadioKeyType          aKeyType)
{
    OT_UNUSED_VARIABLE(aKeyType);
    IgnoreError(GetRadioSpinel().SetMacKey(aKeyIdMode, aKeyId, aPrevKey, aCurrKey, aNextKey));
}

void otPlatRadioSetMacFrameCounter(otInstance *, uint32_t aMacFrameCounter)
{
    IgnoreError(GetRadioSpinel().SetMacFrameCounter(aMacFrameCounter, /* aSetIfLarger */ false));
}

void otPlatRadioSetMacFrameCounterIfLarger(otInstance *, uint32_t aMacFrameCounter)
{
    IgnoreError(GetRadioSpinel().SetMacFrameCounter(aMacFrameCounter, /* aSetIfLarger */ true));
}

otError otPlatRadioSetRegion(otInstance *, uint16_t aRegionCode) { return GetRadioSpinel().SetRadioRegion(aRegionCode); }

otError otPlatRadioGetRegion(otInstance *, uint16_t *aRegionCode) { return GetRadioSpinel().GetRadioRegion(aRegionCode); }

uint32_t otPlatRadioGetSupportedChannelMask(otInstance *) { return GetRadioSpinel().GetRadioChannelMask(false); }

uint32_t otPlatRadioGetPreferredChannelMask(otInstance *) { return GetRadioSpinel().GetRadioChannelMask(true); }

otError otPlatRadioReceiveAt(otInstance *, uint8_t aChannel, uint32_t aStart, uint32_t aDuration)
{
    // Mirrors POSIX `radio.cpp`: receive-at is not wired through the spinel
    // radio here.
    OT_UNUSED_VARIABLE(aChannel);
    OT_UNUSED_VARIABLE(aStart);
    OT_UNUSED_VARIABLE(aDuration);
    return OT_ERROR_NOT_IMPLEMENTED;
}

} // extern "C"
