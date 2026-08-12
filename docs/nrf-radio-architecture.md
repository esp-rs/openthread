# The nRF radio: why the soft-MAC cannot meet the air, and what replaces it

## Provenance

Bringing the hardware-in-the-loop MCU tier up on an nRF52840 (a XIAO board:
`NrfRadio` under `MacRadio` under `ProxyRadio`, against a known-good
`ot-rcp` peer) turned three upstream suites deterministically red - the
sleepy-child family:

- `Cert_6_1_01_RouterAttach` (SED variant; the MED variant passes)
- `Cert_6_4_01_LinkLocal` (SED variant; the MED variant passes)
- `test_child_supervision` (the child is a 500 ms-poll SED)

The failures presented as MLE `Security` errors on both sides of the attach
handshake, which was a red herring twice over: the frames on the air were
correctly formed and correctly secured. The real defect is below MLE, below
the MAC payload, in pure timing - and it is structural, not a bug to patch.
This note records the evidence, the architecture that actually fits the
silicon, the driver decision taken, and what that retires.

## The measured defect

Ground truth came from an air capture (an idle `ot-rcp` ESP32-C6 on the rig
as promiscuous sniffer, microsecond timestamps) plus RTT instrumentation in
`MacRadio`. The two ends of the same link answer the same obligation - the
IEEE 802.15.4 immediate ACK, due `aTurnaroundTime` = 192 us after the last
symbol of the ACK-requesting frame - like this:

| Sender of the ACK | ACK starts after frame end |
| --- | --- |
| ot-rcp peer (nrf-802154 C driver, hardware-timed) | 193 us, every time |
| this crate's soft-MAC on `NrfRadio` | ~500-650 us |

An nrf-802154-class receiver enforces a strict acceptance window around the
nominal ACK slot; our ACKs fall outside it and are discarded as stray
frames. The capture shows the consequences directly:

- every data poll from the sleepy child is retried 3-4 times, each retry
  answered by another of our (late, ignored) ACKs;
- the child's Child ID Request is retried the same way;
- in the reverse direction, the peer's *perfectly timed* ACKs to our own
  transmissions are missed, because re-arming our receiver after TX goes
  through the same software path - so the parent logs `NoAck` and
  retransmits frames that were in fact acknowledged.

Rx-on-when-idle traffic survives this: retries eventually overlap live
receive windows, MLE dedups the duplicates, and the tests pass - paying a
hidden 3-4x airtime tax on every unicast. A sleepy child cannot survive it:
its receive windows are the narrow post-poll slots, every handshake response
arrives one attach cycle late, and the child's MLE - whose candidate state
has already been reset - rejects each late response with `kErrorSecurity`
(`IsNeighborStateValid`). Hence the misleading `Security` symptoms.

## Why software cannot make the deadline

The 192 us budget decomposes, on this silicon, into a fixed hardware tail -
RX disable plus TX ramp-up (~130 us at the default ramp, ~40 us with fast
ramp-up) - and whatever software adds. Software adds: RADIO IRQ latency,
executor wake, header parse, the ACK-decision (frame-pending lookup), the
driver call. Measured total: 500-650 us. Three things follow:

1. **A higher-resolution timer does not help.** embassy-time's 32 kHz
   granularity is not where the lateness comes from; a timer can only delay
   an action that is ready, never accelerate one that is not. We are past
   the deadline before any timer is consulted. (The current code even
   contains the proof: `MacRadio`'s ACK path has a "wait until send time"
   guard that never triggers.)
2. **Average-case heroics do not help either.** The peer enforces the
   worst case. Any CPU-side path shares the core with USB interrupts, other
   drivers, and critical sections; the jitter tail always lands outside a
   window that the fixed hardware tail has already half consumed.
3. **Half the defect has no software timing component at all.** Catching
   the peer's 193 us ACK after our own TX is a receiver re-arm race; there
   is no wait to make more precise, only ramp latency to eliminate.

The deadline must be met by hardware sequencing, with software out of the
loop at the deadline instant.

## What the silicon offers - and does not

The ESP32-C6's 802.15.4 peripheral contains a true autonomous ACK engine:
hardware parses the frame, consults a pending-bit table, and transmits the
ACK with no CPU involvement (the `EspRadio` path already rides it; the
esp-radio pending-bit saga was about *configuring* that engine, never about
composing ACKs).

The nRF RADIO has no such engine - it is a modem with a CRC checker plus a
hardware *sequencer*: SHORTS (event-to-task wires inside the peripheral),
PPI (the same across peripherals), TIFS (a hardware inter-frame-spacing
countdown for shortcut-driven turnarounds), BCMATCH (a bit counter that
fires mid-reception at a programmed offset), and EasyDMA (header bytes are
in RAM while the payload is still on the air). Out of those parts one
builds a **hardware-timed, software-composed** ACK: an ISR triggered by
BCMATCH parses the header mid-frame and stages the ACK buffer; a
CRC-gated TIMER/PPI (or SHORTS+TIFS) chain launches it at exactly 192 us;
the mirror chain (`PHYEND -> DISABLE -> RXEN -> START`) re-arms reception
~40 us after our own TX so the peer's ACK lands in an already-listening
radio. This is precisely the architecture of Nordic's production C driver
(nrf-802154); there is no fully autonomous alternative on this silicon.

## The architecture that fits

Two designs were considered.

**Extending the current per-call driver** (add the turnaround SHORTS to
`try_send`, keep `async fn receive()` as the only reception window) was
rejected: the radio would still be deaf whenever no `receive()` future is
in flight, which is exactly the class of gap the radio contract exists to
forbid (see `radio-contract.md`, C1/C2/C4: reception is a state, frames
flow unsolicited, the radio returns to Receive on its own). Every layer in
this crate that grew an RX queue grew it against this gap.

**The retained design** is the esp-radio shape, moved into the nRF driver:

- The RADIO ISR owns the protocol edge end-to-end: it parses headers
  mid-frame (BCMATCH), filters, decides and stages ACKs (frame-pending
  from a driver-resident source-match table), commits every CRC-valid
  frame into a queue of driver-owned buffers, and keeps the receiver armed
  through all of it via the hardware chains.
- `async fn receive()` degenerates to "pop the queue, or await the
  queue-nonempty signal". `transmit()` submits to the ISR-driven machine
  and awaits its completion (which includes the hardware-timed ack-wait).
- The driver claims `RX_ACK | TX_ACK | SRC_MATCH` (plus the filters) in
  `MacCapabilities` - honestly, because it actually performs those duties.

The structural dividends, and why they hold **regardless of how the driver
is implemented**:

- **`MacRadio` leaves the nRF path.** A software MAC cannot fix a timing
  problem that lives below it; with a capable driver there is nothing left
  for it to emulate. It remains exactly where its timing model is sound:
  the simulation tiers (`SimRadio`, `VtRadio`), whose media have no real
  deadlines.
- **`ProxyRadio` and the interrupt executor retire on nRF.** Their sole
  reason to exist is giving `MacRadio`'s software deadlines a
  high-priority island. With deadlines in silicon and ISRs, the nRF
  firmware collapses to the ESP firmware's shape: one executor,
  `run_ot(ot, radio)`.
- **The source-match two-copy race disappears.** Today the table lives in
  a stack-side mirror and a runner-side copy, synced asynchronously; the
  ACK decision and the `acked_with_frame_pending` report can disagree.
  With the table at the ACK decision point, the driver reports what it
  actually sent.
- **embassy-time leaves the radio path** (the stack's alarm keeps using
  it); the radio's deadlines live on the RADIO/TIMER/PPI hardware.

## The driver decision

A pure-Rust implementation of the above (an "nrf-802154-lite" over the
PAC: ~1-2k lines of ISR state machine, two-stage BCMATCH parsing,
TIMER/PPI choreography, plus the air-validation burden) was scoped and
**rejected - not on feasibility but on redundancy**. The `nrf-802154`
crate already exists, wraps Nordic's tried-and-tested production C driver
(the same one whose 193 us ACKs this investigation measured on the peer),
and layers on nrf-mpsl for coexistence with BLE - something a from-scratch
driver would not approach. Reimplementing it in Rust would be inventing a
wheel to arrive at timing the C core has had for a decade.

The plan of record therefore:

1. This document captures the architecture and the constraint (done).
2. An `nrf-802154`-backed `Radio` implementation replaces `NrfRadio` as
   the nRF MCU-tier radio, claiming the full MAC capability set.
3. The retirements above land with it: `ProxyRadio` + interrupt executor
   out of the nRF firmware, `MacRadio` scoped to the simulation tiers.

The pure-Rust driver remains a documented fallback should `nrf-802154`
prove unworkable in some environment; its design is the one described
above, and this tier - the sniffer, the SED suites, the manual repro
below - is its ready-made validation rig.

## Consequences until then

The nRF MCU tier runs with the current soft-MAC, which means:

- the SED-dependent tests above stay red on this tier (they pass on the
  ESP MCU tier, whose radio ACKs in hardware) and are excluded from its
  expected-pass list;
- rx-on traffic carries the retry tax - functional, but worth remembering
  when reading airtime-sensitive results from this tier;
- `MacRadio`/`ProxyRadio` stay as-is meanwhile: polishing scaffolding that
  is scheduled for retirement is not worth the churn.

## Reproducing the measurement

- Sniffer: pyspinel against an idle `ot-rcp` (any spinel RCP works),
  `sniffer.py -c <channel> -u <port> -b 460800 --crc --rssi > air.pcap`,
  run unbuffered (`python -u`) if the capture is short.
- Manual repro, no harness: bring the DUT up as leader over its console;
  drive the host `cli_node` + RCP as the child with `mode -`,
  `pollperiod 500`, the leader's dataset, `thread start`; the child stays
  `detached` while the capture fills with poll retry bursts against
  late ACKs.
- The discriminating observations, in the order they excluded hypotheses:
  MED variants pass / SED fail (not a crypto or key problem); the
  source-match/FP dance is correct and indirect delivery follows polls by
  ~2.4 ms (not a queueing or latency problem); poll sequence numbers
  repeat 3-4x while each copy gets ACKed (our ACKs unheard); the air
  timestamps (193 us vs 500-650 us) close the case.
