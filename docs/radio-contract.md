# The radio contract: what OpenThread expects

This document tries to capture our understanding of the contract / expectations of the OpenThread C library towards its radio, i.e. towards
the `otPlatRadio*` callbacks.

## Findings

### C1. `otPlatRadioReceive(channel)` is a state transition

Turns out, this callback is a state transition, NOT a request to receive one single frame.

Basically, OpenThread would invoke this callback with the expectation that the radio will transition to receive mode (if it is not in receive mode yet), and start continuously receiving RX frames in the background, on the specified channel.

Each such received frame should be reported by calling the `otPlatRadioReceiveDone` function, unsolicited.
There is no 1:1 pairing between `Receive` calls and number of delivered frames.

Switching from receive mode on channel X to receive mode on channel Y happens via this callback as well.

### C2. Received frames flow up regardless of the most recent command

The radio **MUST** process and report received frames (via `otPlatRadioReceiveDone` - as per above) **regardless** of the 
last command. 

For example, even if the last command was to transmit a frame (via `otPlatRadioTransmit`) - and - the radio might be doing 
a CSMA backoff prior to switching to transmit mode OR the radio might be back to receive mode waiting for the ACK of its just-transmitted 
frame... all received frames during these periods MUST be reported via `otPlatRadioReceiveDone` - immediately or accumulated in a queue and 
reported after the transmission operation is complete.

The single exception: the ACK frame matching an in-flight transmit must NOT be delivered via `ReceiveDone` - it belongs to that
transmit's completion. Matching the ACK corresponding to the transmitted frame is always the platform's, i.e. the driver's job.
The `ACK_TIMEOUT` capability only selects who runs the timeout timer, never who matches the frame.

This was found by examining `SubMac::HandleReceiveDone` (`ot:src/core/mac/sub_mac.cpp`), which has no state guard: the core accepts and processes frames delivered during CSMA backoff, during the transmit window, and during the ACK wait. Reference platforms exploit this (the simulation radio's `radioProcessFrame` runs during `sTxWait`). 

### C3. The transmit sequence is an atomic platform operation

The operation is completed via `otPlatRadioTxDone(frame, ack_frame, error)`. 

It comprises (per declared capabilities):
- CSMA/CCA first (and if requested);
  - Note that radio is in receive mode during this stage - and see C4;
- The transmission;
  - This is the only stage where the radio is in transmit mode;
- The ACK wait for AR frames;
  - Radio is back to receive mode; and should stay in receive mode after the completion of the transmit operation - see C2, C4 and C6.

Furthermore, and as strange as it sounds - `otPlatRadioTransmit` is legal only from Receive state (see `otPlatRadioReceive(channel)`) and should error with `INVALID_STATE` otherwise.

### C4. After the transmit sequence, the radio MUST return to Receive on its own

I.e. right after the frame is transmitted, it should resume reporting received frames via `otPlatRadioReceiveDone`.
In fact - and as per C2 - the radio should report (or at least accumulate and report later) frames received during the transmission sequence.

So in a way, this statement is even stronger: except for the short period where the radio is in transmission mode to send a frame as requested by OpenThread - it should always stay in receive mode and report all received frames. Even during "weird" stages like CCA and waiting for ACKs - see C2.

Also do note that OpenThread does NOT reliably re-issue `otPlatRadioReceive` after the driver calls `otPlatRadioTxDone`.

Evidence from the OpenThread radio simulation code:
- On the normal data path it does (`Mac::HandleTransmitDone` -> operation postlude -> `UpdateIdleMode` -> unconditional `SubMac::Receive`/`Sleep`;
  neither `SubMac::Receive` nor `Radio::Receive` dedups repeat calls).
- On the **active scan** path it does not: after the Beacon Request's `TxDone`, `Mac::HandleTransmitDone` only starts the scan-duration timer
  (`case kOperationActiveScan`) and listens for beacon responses assuming the radio went back to RX by itself. `SubMac`'s own post-TX 
  `SetState(kStateReceive)` is internal bookkeeping; its explicit platform `Receive()` call is compiled only for RCP builds.
- The `mRxChannelAfterTxDone` frame field tells the platform *which channel* to return to (platforms without `TRANSMIT_RETRIES` must ignore it 
  and use the transmit channel; a subsequent stack `Receive` corrects the channel on the normal path).

### C5. A new radio command during a pending transmit is a sanctioned, silent abort

When the radio lacks `ACK_TIMEOUT`, the core runs its own 16ms timer, and on expiry (`SubMac::HandleTimer`, `kStateTransmit` arm) calls
`Get<Radio>().Receive(...)` *while the platform/driver transmit is still pending*, then synthesizes `NO_ACK` **internally**. 

The platform/driver MUST comply - abandon the ACK wait and "enter Receive" so to say.

Moreover, the driver must NOT report a `TxDone` for the aborted transmit (the core has already completed it; a late `TxDone` is a spurious
second completion).

### C6. Sleep is an explicitly commanded state (for radios incapable of auto-sleep), and a sleeping radio MUST drop frames

Basically, OpenThread calling `otPlatRadioSleep` is the one and only indicator for the radio driver that it should **stop** receiving frames in the background (which is initiated by `otPlatRadioSleep`'s counterpart - `otPlatRadioReceive(channel)` - as per C1).

Evidence:
`Mac::UpdateIdleMode` commands `SubMac::Sleep` -> `otPlatRadioSleep` whenever an rx-off-when-idle device goes idle, gated by `ShouldHandleTransitionToSleep() = mRxOnWhenIdle || !RadioSupportsRxOnWhenIdle()`. 

The C simulation radio in Sleep state discards incoming frames (`radioReceive` requires Receive/Transmit state).
Frames "received while asleep" must not be delivered late; a sleepy child provably missing traffic is protocol-relevant behavior 
(parent indirect queueing + Frame Pending), not an optimization.

### C7. `rx-on-when-idle` is two different things

The first is a **state**, and moreover, a state of *the whole OpenThread stack* (the library).

The second is an offload **capability** of the *radio driver*.

#### The stack state

The stack is actually by default in an `rx-on-when-idle = true` state.
What this state means is simply that - except for the short period where the radio is in transmit mode to transmit a frame - it should otherwise stay in receive mode _and report all received frames to the stack_. Basically everything asserted by C2, C4 and C6.

And that's why this state - and its name - is confusing. Because it is the _default_ state of the stack.

A typical Thread device which is mains-powered - a Full Thread Device / FTD or a Minimal End Device / MED - stays in this mode forever.

OK, but when is `rx-on-when-idle = false` useful then?
For Minimal Thread Devices / MTDs which are battery-powered. Since these want to conserve battery power, they cannot afford to stay in receive mode all the time, because - sometimes contrary to intuition - staying in receive mode also consumes power - both in the radio and for the MCU / SRAM if the MCU is a power-hungry variant.

Switching between the two modes of operation can be done via the `OpenThread::set_link_mode` call.

What happens when `rx-on-when-idle = false` becomes the current state of the stack?
The device becomes a Sleepy End Device / SED. These devices do not receive all the time so as to conserve power, and their parent (an FTD device promoted to a router) is accumulating RX frames on their behalf. The stack is then controlling the radio and the platform as a whole (by calling `otPlatRadioSleep` and then `otPlatRadioReceive` after some time) to wake up the radio and instruct it to check with its parent whether there are RX frames accumulated for it.

Evidence:
MLE device mode `r` bit, `Mac::mRxOnWhenIdle` - reaches the platform only as command cadence - rx-on devices idle in Receive,
rx-off devices get Sleep/Receive command pairs around wake windows.

#### The radio offload capability

The capability is `OT_RADIO_CAPS_RX_ON_WHEN_IDLE`, which corresponds to the `Capabilities::AUTO_SLEEP` Radio enum variant. Note that in the pure-Rust enum variant we "inverted" the name of the capability. Reason being, it is "easy" for the radio to "receive when idle". The real capability is the opposite one - knowing when it is safe to go to sleep.

As the name suggests, by declaring this capability, the radio proclaims that the OpenThread stack does **not** need to "manually" switch
it from receive to sleep and then back to receive via the `otPlatRadioReceive(channel)` and `otPlatRadioSleep`, but rather - that the 
radio can do this automatically by itself.

Now, just because the radio CAN automatically sleep does NOT mean it SHOULD automatically switch between sleep and receive state. After all, if it does so, that would mean it would miss frames, which is OK ONLY for devices which are MTDs which moreover had entered a SED mode (and have proclaimed to their neighbourhood that they become SEDs - so that their parenting FTD can start accumulating RX frames for them while they sleep).

Therefore, the radio should enter/exit auto-sleep mode only when the OpenThread stack asks it explicitly, via `otPlatRadioSetRxOnWhenIdle(true/false)`, which would only be called if the radio proclaims the capability in the first place _and_ the user sets the OpenThread stack to behave as a Sleepy End Device.

Note also that implementing this offloading capability in the radio is non-trivial and completely optional. It is optional because if not present the OpenThread stack itself will emulate it, with explicit `Receive`/`Sleep` calls into the radio at the appropriate moments.

Non-trivial, because the radio must then hit every idle boundary itself, per the long list in the `otPlatRadioSetRxOnWhenIdle` doc comment - including keeping the receiver on after a Data Request whose ACK carried Frame Pending.

This capability exists chiefly for RCPs, where per-boundary commands would cost a UART round-trip each.

### C8. Energy scan is the fourth commanded state

`otPlatRadioEnergyScan` (with the `ENERGY_SCAN` capability) is a bounded operation completed via
`otPlatRadioEnergyScanDone`, after which the core re-commands the radio.

Radios without the capability get the core's software fallback (which needs a synchronous RSSI read - implemented in the otherwise async `openthread` crate by returning the last RSSI). 

Active scan is NOT a distinct radio state: it is an ordinary Receive on the scan channel plus a Beacon Request transmit - which is exactly why it depends on C4.

## The radio state machine

```
            Sleep <-------- otPlatRadioSleep ----------+
              |                                        |
   otPlatRadioReceive(ch)                         (idle, rx-off)
              v                                        |
          Receive(ch) <--- auto-return (C4) ---- TransmitSequence
              |    ^                                   ^
              |    +--- otPlatRadioEnergyScanDone      |
              v                                        |
        EnergyScan(ch, dur)              otPlatRadioTransmit (from Receive)
```

Commands are transitions; **Receive is a state, not an operation**. 

While in Receive the platform streams `ReceiveDone`s. The transmit sequence is a temporary excursion that must land back in Receive by itself.

The crate mirrors this split literally in `OpenThread::run_radio`.

## Obligations for `Radio` implementors (to become trait documentation)

1. `transmit` is the complete transmit sequence per the declared
   capabilities (CSMA/CCA, TX, ACK wait for AR frames), resolved without
   the caller pumping `receive()`. The matching ACK is consumed by
   `transmit` and reported in its result; it never surfaces via `receive`.
2. Frames arriving during the transmit sequence's listening phases are
   neither lost nor consumed: they surface through `receive()` (typically
   via a driver-internal queue). Fixed bounds with drop-on-overflow are
   acceptable - that is saturation, not contract violation.
3. Cancellation (dropping the `transmit` future) is a sanctioned abort: the
   frame may already be on the air; the driver must return to Receive and
   the caller must not report a completion for it (C5). The crate only ever
   cancels on a new *command* - never on a configuration change.
4. After the transmit sequence, the driver returns to Receive on its own
   (C4), on `mRxChannelAfterTxDone` where supported, else the TX channel.
5. `set_sleep()` stops reception; frames arriving while parked
   are dropped, not buffered (C6).
6. Software MAC emulation (ACKs, filtering, FP handling) belongs *below*
   the trait, at an execution layer that can meet the 802.15.4 turnaround
   timing - in hardware, in RCP firmware, or in the radio IRQ (the
   `nrf-802154` model). Hosting `MacRadio` on a high-priority executor was
   the attempted fourth option and is now *measured* to be insufficient on
   real RF (see `the-case-with-nrf-radio.md`): its remaining home is the
   simulation media, whose deadlines are not real.

## Per-driver conformance

| Driver | MAC | Ob.1 (tx=full seq) | Ob.2 (rx continuity) | Ob.4 (auto-RX) | Ob.5 (`set_sleep` drops) |
| --- | --- | --- | --- | --- | --- |
| `nrf-802154` | driver IRQ layer | yes | yes (IRQ queue) | yes (`rx_when_idle`) | yes |
| `EspRadio` | esp-radio HW/blob | yes | yes (driver queue) | yes | needs check |
| `SpinelRadio` | RCP firmware | yes | yes (`rx_queue`) | yes (RCP) | needs check |
| `embassy-nrf` + `MacRadio` | software - **fails hard ACK timing on air; to be replaced by `nrf-802154`** (`the-case-with-nrf-radio.md`) | via wrapper | via parking queue | runner (done) | runner parks |
| `SimRadio`/`VtRadio` + `MacRadio` | software (fine: no real deadlines) | via wrapper | via parking queue | runner (done) | yes (flush-on-wake) |

## Open questions

- Whether to tolerate (ignore) a late `TxDone` after a C5 abort in the
  crate's glue, for robustness against drivers that report one anyway.
- RX timestamps: the glue stamps `mRxInfo.mTimestamp` at *delivery*
  (`Instant::now()` in `plat_radio_receive_done`, already marked "not
  precise"), and `PsduMeta` has no arrival-time field, so frames parked
  during a transmit sequence get stamped up to ~20 ms late. Unused by
  anything the stack does today (MLE-level operation ignores it), but CSL
  sync, Link Metrics and time-sync IEs all consume it - when any of those
  land, `PsduMeta` needs an arrival timestamp captured below the trait,
  independent of (but amplified by) the parked-delivery latency.
