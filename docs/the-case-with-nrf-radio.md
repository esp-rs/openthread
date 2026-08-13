# The nRF radio: why wrapping the PHY-only `NrfRadio` in the soft-MAC `MacRadio` and executing in a high priority executor is not enough

The `NrfRadio`, which models the NRF52 radio modem is currently a very simple PHY-only radio. In other words, it can:
- Transmit a frame, when `NrfRadio::transmit` is called
- Receive a frame, when `NrfRadio::receive` is called

Notably, it cannot (**and this is crucial**), auto-ACK, i.e.:
- Automatically receive the ACK frame for the just transmitted one via NrfRadio::transmit
- Automatically send an ACK frame for a just-received frame via NrfRadio::receive

Also, it cannot do other MAC offloading operations (and this is less crucial):
- Support filtering by short or extended address or by PAN-ID
- Maintain a queue of received frames; a frame is received when `NrfRadio::receive` is called and that's it; all others are missed

For a long time it was believed that emulating the MAC-offloading capabilities in software by wrapping `NrfRadio` in `MacRadio`, and then
executing `MacRadio` in a high priority executor (so that the strict a few tens of microseconds' timings of ACK-TX and ACK-RX can be kept) would do.

**Unfortunately, this turns out not to be the case.**

While the non-crucial MAC filtering and RX-queue capabilites are successfully emulated with the `MacRadio` wrapper, the `MacRadio(NrfRadio)` nesting cannot keep the deadline necessary for ACK receival and transmission, even with a high priority executor.

Note also that this design was partially driven by the unwillingness to implement the MAC-offloading capabilities directly in the upstream `embassy-nrf` radio.

Note also that I'm not saying that ACKs cannot be received/transmitted in software. This is exactly what the C NRF driver does. But that driver takes advantage of a few additional capabilities of the NRF 802.15.4 hardware which are not currently exposed by the `embassy-nrf` driver - see next sections below.

## Evidence

When testing the  `NrfRadio` under `MacRadio` under `ProxyRadio` triple against a known-good `ot-rcp` peer with the E2E tests,
these three upstream suites deterministically fail all the time - the sleepy-child family:
- `Cert_6_1_01_RouterAttach` (SED variant; the MED variant passes)
- `Cert_6_4_01_LinkLocal` (SED variant; the MED variant passes)
- `test_child_supervision` (the child is a 500 ms-poll SED)

The failures were MLE `Security` errors on both sides of the attach handshake, which was a red herring twice over: the frames on the air were correctly formed and correctly secured. The real defect is below MLE, below the MAC payload, in pure timing.

When using an air capture (an idle `ot-rcp` ESP32-C6 on the rig as promiscuous sniffer, microsecond timestamps) plus RTT instrumentation in
`MacRadio`, the following timings were observed:

| Sender of the ACK | ACK starts after frame end |
| --- | --- |
| ot-rcp peer (nrf-802154 C driver, hardware-timed) | 193 us, every time |
| this crate's soft-MAC on `NrfRadio` | ~500-650 us |

The drama is, an nrf-802154-class receiver enforces a strict acceptance window around the nominal ACK slot;
and since the `NrfRadio` ACKs fall outside it, they are discarded as stray frames. 

The capture shows the consequences directly:
- every data poll from the sleepy child is retried 3-4 times, each retry answered by another of our (late, ignored) ACKs;
- the child's Child ID Request is retried the same way;
- in the reverse direction, the peer's *perfectly timed* ACKs to our own transmissions are missed, because re-arming our receiver 
  after TX goes through a slow software path - so the parent logs `NoAck` and retransmits frames that were in fact acknowledged.

`rx-on-when-idle = true` traffic survives this: retries eventually overlap live receive windows, MLE dedups the duplicates, and the tests pass - paying a hidden 3-4x airtime tax on every unicast. 

A sleepy child cannot survive it though: its receive windows are the narrow post-poll slots, every handshake response arrives one attach cycle
late, and the child's MLE - whose candidate state has already been reset - rejects each late response with `kErrorSecurity`
(`IsNeighborStateValid`). Hence the misleading `Security` symptoms.

## Why NrfRadio + MacRadio + ProxyRadio cannot make the deadlines

### For receiving an ACK frame for a frame we just sent

The reason is the all-in-software switch of the radio from trasnsmission to receival:
- RADIO IRQ latency
- Executor wake
- Execution of `NrfRadio::receive` that "manually" switches from TX to RX

Measured total: ~ 500-650 us. Way too slow.

What would be the alternative? The NRF modem can be instructed - prior to even initiating the transmission - to "automatically" switch to receive mode once the transmission is over, which would take ~130 us at the default ramp, ~40 us with fast ramp-up.

So no matter how fast the interrupt executor is, it would never match an automatic switch of the radio from TX to RX mode.

### For sending an ACK frame for a frame we just received

Here, the problem is that the preparation _and_ the send of the ACK are happening - **all in software** - **after** the frame is received completely. In other words:
- A lot of time (~ 200+ us) is lost by the CPU doing nothing and just waiting for the modem to receive the frame in its full;
  - Contrary to that, the C driver **interleaves** the receival of the frame and the (in-software) preparation of the ACK. This is possible because the ACK only needs the MAC header of the incoming frame, not its payload, so the ACK preparation can start as soon as the header is received
- The switch of the radio from TX to RX is all-software again.

### What the NRF silicon offers

Unlike - say - the ESP32XX 802.15.4 hardware which as a true autonomous ACK engine, 
the nRF RADIO has no such thing - it is a modem with a CRC checker plus a hardware *sequencer*: 
- SHORTS (event-to-task wires inside the peripheral);
- PPI (the same across peripherals);
- TIFS (a hardware inter-frame-spacing countdown for shortcut-driven turnarounds);
- BCMATCH (a bit counter that fires mid-reception at a programmed offset);
- And EasyDMA (header bytes are in RAM while the payload is still on the air).

However with those parts we _can_ build a **hardware-timed, software-composed** ACK: 
- For ACK-TX - an ISR triggered by BCMATCH parses the header mid-frame and stages the ACK buffer; a CRC-gated TIMER/PPI
  (or SHORTS+TIFS) chain launches it at exactly 192 us;
- For ACK-RX - even simpler - (`PHYEND -> DISABLE -> RXEN -> START`) re-arms reception ~40 us after our own TX so the peer's
  ACK lands in an already-listening radio. 
  
This is the architecture of Nordic's production C driver (nrf-802154).

## What shall we do?

### Option 1: Minimal extensions to the upstream `embassy-nrf` 802.15.4 radio driver

What is inescapable is extending the upstream Radio driver in `embassy-nrf` so that it can expose the capabilities discussed in the previous section. At the very minimum, these would be:
- A way to instruct the radio to auto-switch to receive after transmission is over;
- An "I just received a frame MAC header!" hook of sorts, which would allow us to prep the corresponding ACK frame;
- Plus a way to instruct the radio to send this ACK frame exactly at 192 us after the incoming frame is received completely (antena time).

### Option 1a: Significant extensions to the upstream `embassy-nrf` 802.15.4 radio driver

The thing is, if we do the above, and start extending the upstream radio, why not walk the full path and extend it also with:
- The soft-mac capabilities of the `MacRadio` where it could filter by short address, extended address and PAN-ID
- The src-match table
- The queue of received frames

And then why not just push the whole ACK-TX and ACK-RX logic from `MacRadio` upstream? After all - before `SimRadio` and `VtRadio` were introduced, `MacRadio` - as well as the whole `ProxyRadio` executor offloading infra existed **just** to enhance `NrfRadio` with the MAC-offloading caps OpenThread needs it to have in the first place.

The big deal with pushing all MAC-offloading capabilities to upstream `embassy-nrf` is not only or necessarily in the retirement of `MacRadio`, but in the retirement of `ProxyRadio`, `PhyRadioRunner` and the whole notion of "you need to setup a higher prio interrupt executor for the radio driver specifically". Because the upstream `embassy-nrf` radio will just do all timing-sensitive operations in ISRs thus rendering the need for an interrupt executor obsolete (also because it would maintain an internal RX queue).

### Option 2: All bets on `nrf-802154`

The `nrf-802154` crate is a type-safe Rust wrapper of the native NRFXLIB C 802.15.4 driver.
This driver rides on top of `nrf-mpsl` and as such it can co-exist with the NRF BLE controller (i.e. the radio can support BLE and 802.15.4 traffic simultaneously, which is a big deal for `rs-matter`).

This driver offers full mac-offloading capabilities, and as such cannot / should not be used with `MacRadio` + `ProxyRadio` + an interrupt executor.

So this option would _also_ mean we can retire `ProxyRadio`, `PhyRadioRunner`, the interrupt execvutor requirement and the usage of `MacRadio` for anything but simulation purposes.
