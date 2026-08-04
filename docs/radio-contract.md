# The radio contract: what OpenThread actually expects from `Radio`

Status: DRAFT for review. Nothing in here is implemented yet except where
explicitly marked; the current code's divergences are listed as numbered gaps.

## Provenance

Running the unmodified upstream `thread-cert` suites against this crate's
platform (see `tests/README.md`) exposed a family of defects that all traced
back to one root: the semantics of OpenThread's radio platform API
(`otPlatRadio*`) were never written down for `Radio` implementors, so every
layer re-derived them - each slightly differently. The visible symptom of the
mismatch is that nearly every radio implementation in and around this crate
has independently grown an RX queue:

| Layer | Queue | Stated reason |
| --- | --- | --- |
| `SpinelRadio` | `rx_queue: heapless::Deque<RxFrame, 8>` | frames arriving during command waits |
| `EspRadio` | esp-radio's queue (`rx_queue_size`) | IRQ-fed buffering below the trait |
| `ProxyRadio` | request/response zerocopy channels | bridging to a high-priority executor |
| `MacRadio` | 4-slot `pending_rx` parking queue | frames crossing the TX ACK wait |
| `VtRadio` (tests) | unbounded `VecDeque` | lockstep event bursts |

A queue is not wrong - reception is asynchronous and something must hold
frames until the stack collects them. What is wrong is that the *need* for
it, and the rules it must obey, live in five heads instead of one document.
This note is that document. Every claim below was verified against the
vendored OpenThread sources (`openthread-sys/openthread`, referenced as
`ot:`) - by reading `include/openthread/platform/radio.h` (the normative
comments) and `src/core/mac/sub_mac.cpp` / `mac.cpp` (the only callers).

## The verified contract

**C1. `otPlatRadioReceive(channel)` is a state transition, not a per-frame
request.** "Transition the radio from Sleep to Receive (turn on the radio)"
(`ot:include/openthread/platform/radio.h`, `otPlatRadioReceive`). While the
radio is in Receive state, the platform delivers zero or more frames, one
`otPlatRadioReceiveDone` per frame, unsolicited. There is no pairing between
`Receive` calls and delivered frames.

**C2. Received frames flow up regardless of the most recent command.**
`SubMac::HandleReceiveDone` (`ot:src/core/mac/sub_mac.cpp`) has no state
guard: the core accepts and processes frames delivered during CSMA backoff,
during the transmit window, and during the ACK wait. Reference platforms
exploit this (the simulation radio's `radioProcessFrame` runs during
`sTxWait`). The single exception: the ACK frame matching an in-flight
transmit must NOT be delivered via `ReceiveDone` - it belongs to that
transmit's completion (C3). **Matching the ACK is always the platform's
job**; the `ACK_TIMEOUT` capability only selects who runs the timeout timer,
never who matches the frame.

**C3. The transmit sequence is an atomic platform operation, completed via
`otPlatRadioTxDone(frame, ack_frame, error)`.** It comprises (per declared
capabilities): CSMA/CCA, the transmission, and the ACK wait for AR frames.
`otPlatRadioTransmit` is legal only from Receive state and errors with
`INVALID_STATE` otherwise.

**C4. After the transmit sequence, the radio returns to Receive on its own.**
The stack does NOT reliably re-issue `otPlatRadioReceive` after `TxDone`:

- On the normal data path it does (`Mac::HandleTransmitDone` -> operation
  postlude -> `UpdateIdleMode` -> unconditional `SubMac::Receive`/`Sleep`;
  neither `SubMac::Receive` nor `Radio::Receive` dedups repeat calls).
- On the **active scan** path it does not: after the Beacon Request's
  `TxDone`, `Mac::HandleTransmitDone` only starts the scan-duration timer
  (`case kOperationActiveScan`) and listens for beacon responses assuming
  the radio went back to RX by itself. `SubMac`'s own post-TX
  `SetState(kStateReceive)` is internal bookkeeping; its explicit platform
  `Receive()` call is compiled only for RCP builds.

The `mRxChannelAfterTxDone` frame field tells the platform *which channel*
to return to (platforms without `TRANSMIT_RETRIES` must ignore it and use
the transmit channel; a subsequent stack `Receive` corrects the channel on
the normal path).

**C5. A new radio command during a pending transmit is a sanctioned, silent
abort.** When the radio lacks `ACK_TIMEOUT`, the core runs its own 16ms
timer, and on expiry (`SubMac::HandleTimer`, `kStateTransmit` arm) calls
`Get<Radio>().Receive(...)` *while the platform transmit is still pending*,
then synthesizes `NO_ACK` **internally**. The platform must comply - abandon
the wait, enter Receive - and must NOT report a `TxDone` for the aborted
transmit (the core has already completed it; a late `TxDone` is a spurious
second completion).

**C6. Sleep is an explicitly commanded state (for capability-less radios),
and a sleeping radio drops frames.** `Mac::UpdateIdleMode` commands
`SubMac::Sleep` -> `otPlatRadioSleep` whenever an rx-off-when-idle device
goes idle, gated by `ShouldHandleTransitionToSleep() = mRxOnWhenIdle ||
!RadioSupportsRxOnWhenIdle()`. The C simulation radio in Sleep state
discards incoming frames (`radioReceive` requires Receive/Transmit state).
Frames "received while asleep" must not be delivered late; a sleepy child
provably missing traffic is protocol-relevant behavior (parent indirect
queueing + Frame Pending), not an optimization.

**C7. `rx-on-when-idle` is two different things; keep them apart.**

1. *Stack state* (MLE device mode `r` bit, `Mac::mRxOnWhenIdle`): reaches
   the platform only as command cadence - rx-on devices idle in Receive,
   rx-off devices get Sleep/Receive command pairs around wake windows.
2. *Radio capability + policy toggle* (`OT_RADIO_CAPS_RX_ON_WHEN_IDLE`,
   `otPlatRadioSetRxOnWhenIdle`): an offload negotiation. Capability absent
   does NOT mean "cannot receive when idle" - it means the radio has no
   autonomous idle policy and the core drives it imperatively (and never
   calls `SetRxOnWhenIdle`). Capability present inverts control: the core
   hands the radio a standing policy and stops issuing explicit Sleeps
   (`ShouldHandleTransitionToSleep` returns false); the radio must then hit
   every idle boundary itself, per the long list in the
   `otPlatRadioSetRxOnWhenIdle` doc comment - including keeping the
   receiver on after a Data Request whose ACK carried Frame Pending. This
   capability exists chiefly for RCPs, where per-boundary commands would
   cost a UART round-trip each.

**C8. Energy scan is the fourth commanded state.** `otPlatRadioEnergyScan`
(with the `ENERGY_SCAN` capability) is a bounded operation completed via
`otPlatRadioEnergyScanDone`, after which the core re-commands the radio.
Radios without the capability get the core's software fallback (which needs
a synchronous RSSI read - unavailable through this crate's async trait, so
such scans cleanly return no result; see `Radio::energy_scan`). Active scan
is NOT a distinct radio state: it is ordinary Receive on the scan channel
plus a Beacon Request transmit - which is exactly why it depends on C4.

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

Commands are transitions; **Receive is a state, not an operation**. While in
Receive the platform streams `ReceiveDone`s. The transmit sequence is a
temporary excursion that must land back in Receive by itself.

## Gaps in the current implementation

The runner (`lib.rs::run_radio`) models commands as operations rather than
states. Concretely:

- **G1** - the Tx arm `break`s to the command await after `TxDone`. Safe on
  the normal path only because the `Receive` command is signaled during the
  `process_tasklets()` call before the `break`; on the active-scan path
  (C4) no command comes, and the node is deaf for the scan window. Latent
  today (dataset-based attach never active-scans); real for `scan`/MLE
  Discover flows.
- **G2** - the Tx arm reports `TxDone(ABORT)` when interrupted by a new
  command, violating C5's silent-abort rule (observed benign, still wrong).
- **G3** - there is no `RadioCommand::Sleep`; `otPlatRadioSleep` is a no-op.
  "Sleep" only emerges when the Rx arm's continuation check happens to
  break. Violates C6 twice: hardware drivers get no power-down hook, and
  frames arriving "while asleep" accumulate in driver buffers (`VT_RX`, UDP
  sockets, esp-radio's queue) and are delivered late instead of dropped -
  simulated SEDs currently over-hear, which can mask exactly the
  indirect-messaging bugs the cert suites exist to catch.
- **G4** - the Rx arm's continue-vs-break decision keys on
  `Config::rx_when_idle`, conflating C7's two dimensions. For
  capability-less radios (both sim radios) `SetRxOnWhenIdle` is never
  called, the config stays at its default `true` forever, and the check is
  dead weight; under the state-machine model the decision comes from
  commands alone and the check disappears (along with the deferred
  `pending_rx_when_idle` application hack).
- **G5** - `MacRadio`'s ACK wait consumes the RX stream, requiring the
  4-slot parking queue (frames crossing the wait are screened, ACKed and
  parked for the next `receive()`). This is a correct-but-minimal
  realization of C2 given the current trait shape; the structural answer is
  below.

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
   the caller must not report a completion for it (C5).
4. After the transmit sequence, the driver returns to Receive on its own
   (C4), on `mRxChannelAfterTxDone` where supported, else the TX channel.
5. `sleep()` (new, default no-op) stops reception; frames arriving while
   asleep are dropped, not buffered (C6).
6. Software MAC emulation (ACKs, filtering, FP handling) belongs *below*
   the trait, at an execution layer that can meet the 802.15.4 turnaround
   timing - in hardware, in RCP firmware, in the radio IRQ (the
   `nrf-802154` model), or, for simulation, anywhere. `MacRadio` remains as
   a helper for bare-PHY drivers but must be hosted in a suitable execution
   context (see `embassy-nrf` + `ProxyRadio` + interrupt executor), and its
   use is an integration decision, not something the crate applies
   implicitly.

## Per-driver conformance

| Driver | MAC | Ob.1 (tx=full seq) | Ob.2 (rx continuity) | Ob.4 (auto-RX) | Ob.5 (sleep) |
| --- | --- | --- | --- | --- | --- |
| `nrf-802154` | driver IRQ layer | yes | yes (IRQ queue) | yes (`rx_when_idle`) | yes |
| `EspRadio` | esp-radio HW/blob | yes | yes (driver queue) | yes | needs check |
| `SpinelRadio` | RCP firmware | yes | yes (`rx_queue`) | yes (RCP) | needs check |
| `embassy-nrf` + `MacRadio` | software | via wrapper | via parking queue (G5) | runner concern | no (G3) |
| `SimRadio`/`VtRadio` + `MacRadio` | software | via wrapper | via parking queue | runner concern | no (G3) |

## Plan

Each step lands only with the full real-time and virtual-time suites green
(`cargo test` in `tests/`, `cargo xtask itest` both modes) - the suites that
exposed all of this are the safety net for fixing it.

1. **Runner as state machine** (no trait change): restructure `run_radio`
   around `{Sleep, Receive(conf), EnergyScan, TransmitSequence}`; Tx arm
   returns to Receive instead of the command await (G1); silent abort on
   command interruption (G2); `RadioCommand::Sleep` plumbed from
   `otPlatRadioSleep` (G3); drop the `rx_when_idle` continuation check and
   the deferred-application hack (G4).
2. **Trait contract**: the obligations above as `Radio` documentation, plus
   `async fn sleep(&mut self)` with a default no-op implementation.
3. **Sim fidelity**: `SimRadio`/`VtRadio` honor sleep by discarding (match
   the C simulation radio); verify SED cert tests still pass - and now
   prove the right thing.
4. **`MacRadio` disposition**: keep as the bare-PHY helper with its
   obligations documented (parking queue stays for as long as an ACK wait
   above the trait exists); new drivers are pointed at the below-trait
   model (`nrf-802154`) instead.

## Open questions

- Sleep-drop mechanics for internally-queued drivers: gate at arrival vs
  flush on wake. (The C radio gates at arrival; flushing on wake is
  indistinguishable to the stack but simpler for queue-below drivers.)
- Does `Config::rx_when_idle` remain in `Config` (as the C7-dimension-2
  policy for capability-advertising drivers) or move to a dedicated
  `set_rx_on_when_idle` call, mirroring the platform API more closely?
- `EspRadio`/`SpinelRadio` sleep behavior needs verification against Ob.5
  (esp-radio and RCP firmwares have their own power states).
- Whether to tolerate (ignore) a late `TxDone` after a C5 abort in the
  crate's glue, for robustness against drivers that report one anyway.
