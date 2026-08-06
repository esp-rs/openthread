# The radio contract: what OpenThread actually expects from `Radio`

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
| `MacRadio` | 8-slot `pending_rx` parking queue | frames crossing the TX ACK wait |
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

   The upstream names invert the semantics they carry: every radio can
   *receive* when idle (capability absent = the trivial default, staying
   in RX), so the capability's entire content is in the FALSE direction of
   the toggle - permission to power down, plus the burden of identifying
   "idle", which is a MAC-transaction concept, not a PHY state (upstream's
   own doc opens with "it's hard and costly for the SubMac to identify
   these situations"). The crate therefore surfaces this dimension as
   `Capabilities::AUTO_SLEEP` and `Config::auto_sleep` - polarity inverted
   (`auto_sleep = !rx_on_when_idle`, so `true` is the direction with
   content and the `Config` default is the natural `false`) - and keeps
   the OT vocabulary only at the translation boundaries (the plat glue,
   the spinel `RX_ON_WHEN_IDLE_MODE` property, esp-radio's config field).

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

The crate mirrors this split literally (`lib.rs::run_radio`):

- **Sleep/Receive is state**, carried by [`Config::receive`] like any other
  standing configuration - not a command, and not a trait method. A driver
  powers its receiver down when it turns `false` and must then MISS traffic
  rather than queue it (C6); the simulation radios discard whatever
  accumulated on the `false -> true` edge, which is their wake boundary.
- **Transmit and energy scan are excursions**, the only two `RadioCommand`s
  besides `Interrupt`. They are raced against the arrival of a NEW command:
  a command landing mid-excursion is the stack cancelling it (C5), so the
  excursion future is dropped and no completion is reported. Because the
  command signal holds one value, a cancel also supersedes a not-yet-started
  excursion - the abandoned frame is simply never sent.
- **`Interrupt`** is what `otPlatRadioReceive`/`Sleep`/`Disable` raise. It
  carries no work of its own: it exists to cancel an excursion, or to bring
  the runner back around to re-read the configuration and re-enter Receive.
- **Configuration and source-match updates are NOT commands.** They wake the
  runner through their own signals - which interrupts Receive, since Receive
  owes the stack no completion - but they never abort a transmit or a scan.
  (Getting this wrong hangs the MAC: the stack waits forever for a
  completion callback belonging to an operation that was quietly killed.)
  Source-match in particular must land *during* ongoing Receive: it decides
  the Frame Pending bit of the ACKs the radio sends to data polls, and
  OpenThread updates it without issuing any radio command afterwards.

## Gaps that this closed (historical)

The runner used to model commands as operations rather than states. All of
G1-G4 are fixed; G5 stands by design. Kept because the reasoning is the
justification for the current shape:

- **G1** - the Tx arm `break`s to the command await after `TxDone`. Safe on
  the normal path only because the `Receive` command is signaled during the
  `process_tasklets()` call before the `break`; on the active-scan path
  (C4) no command comes, and the node is deaf for the scan window. Latent
  today (dataset-based attach never active-scans); real for `scan`/MLE
  Discover flows.
- **G2** - the Tx arm reports `TxDone(ABORT)` when interrupted by a new
  command, violating C5's silent-abort rule (observed benign, still wrong).
- **G3** - `otPlatRadioSleep` was a no-op (fixed: it clears
  [`Config::receive`], which is the driver's power-down hook).
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
  bounded parking queue (frames crossing the wait are screened, ACKed and
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
   the caller must not report a completion for it (C5). The crate only ever
   cancels on a new *command* - never on a configuration change.
4. After the transmit sequence, the driver returns to Receive on its own
   (C4), on `mRxChannelAfterTxDone` where supported, else the TX channel.
5. `Config::receive == false` stops reception; frames arriving while parked
   are dropped, not buffered (C6). This is a configuration field rather than
   a `sleep()` method deliberately: Sleep-vs-Receive is the radio's standing
   state, so it travels with the rest of the configuration and a driver
   applies it in `set_config` like everything else.
6. Software MAC emulation (ACKs, filtering, FP handling) belongs *below*
   the trait, at an execution layer that can meet the 802.15.4 turnaround
   timing - in hardware, in RCP firmware, in the radio IRQ (the
   `nrf-802154` model), or, for simulation, anywhere. `MacRadio` remains as
   a helper for bare-PHY drivers but must be hosted in a suitable execution
   context (see `embassy-nrf` + `ProxyRadio` + interrupt executor), and its
   use is an integration decision, not something the crate applies
   implicitly.

The `Radio` trait's "Contract" section now carries these, renumbered:
items 1-3 and 5 above are trait contract points 1-4. Item 4 (auto-return
to Receive) is deliberately NOT a trait obligation - the crate's runner
discharges it (see the runner model above), and a driver owes nothing there beyond item
2's continuity. Item 6 is an architecture principle, not a per-driver
obligation. The trait text also spells out the two personalities hiding
in points 1 and 2: without `TX_ACK`/`RX_ACK` they collapse to pure send /
pure wait, with `MacRadio` polyfilling the full semantics. The table
below keeps THIS list's numbering.

## Per-driver conformance

| Driver | MAC | Ob.1 (tx=full seq) | Ob.2 (rx continuity) | Ob.4 (auto-RX) | Ob.5 (`receive=false` drops) |
| --- | --- | --- | --- | --- | --- |
| `nrf-802154` | driver IRQ layer | yes | yes (IRQ queue) | yes (`rx_when_idle`) | yes |
| `EspRadio` | esp-radio HW/blob | yes | yes (driver queue) | yes | needs check |
| `SpinelRadio` | RCP firmware | yes | yes (`rx_queue`) | yes (RCP) | needs check |
| `embassy-nrf` + `MacRadio` | software | via wrapper | via parking queue (G5) | runner (done) | runner parks; PHY power-down needs `ProxyRadio` plumbing |
| `SimRadio`/`VtRadio` + `MacRadio` | software | via wrapper | via parking queue | runner (done) | yes (flush-on-wake) |

## Open questions

- `EspRadio`/`SpinelRadio` behavior on `Config::receive == false` needs
  verification against Ob.5 (esp-radio and RCP firmwares have their own
  power states, and both currently just store the flag).
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
