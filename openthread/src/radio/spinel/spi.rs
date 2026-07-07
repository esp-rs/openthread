//! [`SpiSpinelTransport`]: spinel frames over SPI (the 5-byte SPI header
//! protocol).
//!
//! Unlike a UART, SPI carries the **raw** spinel frame: each SPI transaction has
//! a 5-byte header whose `data_len` field delimits the payload, so there is no
//! HDLC (see [`super::uart`] for the UART/HDLC path). SPI is also
//! controller-driven and full-duplex-per-transfer, which is why this transport
//! additionally needs an interrupt line — see [`SpiSpinelTransport`].
//!
//! **⚠️ Not yet hardware-tested** — this code path is compile-checked only; it
//! has not been run against a real `ot-rcp` over SPI. See the [`super`] module
//! docs. The interrupt handling is written defensively — the line *level* is
//! polled (via [`InputPin`](embedded_hal::digital::InputPin)) before awaiting an
//! edge, so a missed edge alone will not wedge `recv`, and the asserted polarity
//! is a constructor parameter ([`IntPolarity`]) rather than a baked-in
//! assumption. What remains unvalidated is real-link behaviour: the full-duplex
//! `accept_len`/`data_len` negotiation is implemented to spec but only reasoned
//! about, not observed under a slow / back-pressuring RCP.
//!
//! It also implements the spec's *baseline* only (no optional on-SPI CRC, no
//! receive-alignment allowance, a plain retry-until-accepted loop rather than the
//! reference's backoff) — deliberate scope cuts, not correctness gaps.

use core::mem::MaybeUninit;

use super::{SpinelTransport, MAX_SPINEL_FRAME};

/// SPI framing header size (flag byte + accept-len + data-len, all before the
/// payload). See [`SpiSpinelTransport`] for the layout.
const SPI_HEADER_SIZE: usize = 5;

/// The size of one full SPI transfer buffer (header + max payload).
const SPI_BUF_SIZE: usize = SPI_HEADER_SIZE + MAX_SPINEL_FRAME;

/// Flag-byte constants (from OpenThread's `spi_frame.hpp`).
const SPI_FLAG_RESET: u8 = 1 << 7;
const SPI_FLAG_PATTERN: u8 = 0x02;
const SPI_FLAG_PATTERN_MASK: u8 = 0x03;

/// A minimum SPI transfer size, so a small frame the RCP wants to send us can be
/// picked up in a single transaction even before we know its length.
const SPI_SMALL_PACKET_SIZE: usize = 32;

/// The electrical polarity of the RCP's interrupt (`INT`) line — i.e. which
/// level means "the RCP has a frame for the host". OpenThread's reference RCPs
/// drive it [`ActiveLow`](IntPolarity::ActiveLow), but this is board-dependent,
/// so [`SpiSpinelTransport::new`] takes it explicitly.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IntPolarity {
    /// `INT` is asserted (RCP has data) when the line is **low**. The reference
    /// polarity for a stock `ot-rcp`.
    ActiveLow,
    /// `INT` is asserted (RCP has data) when the line is **high**.
    ActiveHigh,
}

/// A [`SpinelTransport`] over SPI, using OpenThread's SPI framing protocol
/// (`spi_frame.hpp` / the POSIX `spi_interface`), against a stock `ot-rcp` built
/// with the SPI HDLC-less spinel interface.
///
/// # Why SPI needs more than a byte pipe
///
/// SPI is controller-driven and full-duplex-per-transfer: the RCP (a SPI
/// *peripheral*) cannot start a transfer, and every clocked byte shifts one out
/// *and* one in. So two extra mechanisms are layered on the bus:
///
/// - **An interrupt line** (`int`): the RCP asserts it to say "I have a frame for
///   you, please clock a transfer". This is the peripheral-initiated channel raw
///   SPI lacks. This transport [`Wait`](embedded_hal_async::digital::Wait)s on it
///   before reading, and polls its level to avoid missing an edge. Its asserted
///   polarity (active-low for a stock `ot-rcp`, but board-dependent) is passed to
///   [`new`](SpiSpinelTransport::new) as an [`IntPolarity`].
/// - **A 5-byte SPI frame header** on every transfer:
///   `flag(1) | accept_len(2, LE) | data_len(2, LE)`. `accept_len` is how many
///   payload bytes *this* side can receive; `data_len` is how many it is sending.
///   Because the transfer is full-duplex, one transaction simultaneously pushes
///   our frame and pulls the RCP's, each bounded by the other's advertised
///   `accept_len`. The flag byte carries a reset bit and a fixed pattern
///   (`0x02`) that distinguishes a real header from an idle `0x00`/`0xFF` bus.
///
/// The payload inside the SPI frame is the **raw spinel frame** — SPI does its
/// own framing via `data_len`, so (unlike UART) there is no HDLC.
///
/// ```ignore
/// // `spi: embedded_hal_async::spi::SpiDevice`,
/// // `int: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin`
/// static SPI_RESOURCES: ConstStaticCell<SpiTransportResources> =
///     ConstStaticCell::new(SpiTransportResources::new());
/// static RADIO_RESOURCES: ConstStaticCell<SpinelRadioResources> =
///     ConstStaticCell::new(SpinelRadioResources::new());
///
/// let transport =
///     SpiSpinelTransport::new(spi, int, IntPolarity::ActiveLow, SPI_RESOURCES.take());
/// let radio = SpinelRadio::new(transport, RADIO_RESOURCES.take());
/// ```
pub struct SpiSpinelTransport<'a, S, I> {
    spi: S,
    int: I,
    /// Which `int` level means "the RCP has a frame for us".
    int_polarity: IntPolarity,
    /// Full SPI transfer buffers (header + payload), reused every transaction.
    tx_buf: &'a mut [u8; SPI_BUF_SIZE],
    rx_buf: &'a mut [u8; SPI_BUF_SIZE],
    /// A single queued outbound spinel frame waiting to be accepted by the RCP.
    tx_pending: Option<usize>,
    /// A single received spinel frame not yet handed to `recv`, `[.. len]` of
    /// `rx_frame`.
    rx_frame: &'a mut [u8; MAX_SPINEL_FRAME],
    rx_ready: Option<usize>,
    /// The RCP's last-advertised pending data length (learned from a prior header
    /// so the next transfer is sized to fit it).
    rcp_data_len: usize,
    /// Whether we still owe the RCP the reset flag on our next header (set on the
    /// very first transfer so the RCP knows the host came up fresh).
    send_reset: bool,
}

impl<'a, S, I> SpiSpinelTransport<'a, S, I> {
    /// Create an SPI spinel transport over SPI device `spi` and interrupt line
    /// `int`, whose asserted polarity (which level means "the RCP has a frame for
    /// us") is given by `int_polarity` — [`IntPolarity::ActiveLow`] for a stock
    /// `ot-rcp` — with its buffers borrowed from `resources`.
    pub fn new(
        spi: S,
        int: I,
        int_polarity: IntPolarity,
        resources: &'a mut SpiTransportResources,
    ) -> Self {
        let (tx_buf, rx_buf, rx_frame) = resources.init();

        Self {
            spi,
            int,
            int_polarity,
            tx_buf,
            rx_buf,
            tx_pending: None,
            rx_frame,
            rx_ready: None,
            rcp_data_len: 0,
            send_reset: true,
        }
    }
}

impl<S, I> SpiSpinelTransport<'_, S, I>
where
    S: embedded_hal_async::spi::SpiDevice,
    I: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin,
{
    /// Perform one SPI transaction, honoring flow control in both directions:
    /// advertise how much we can receive, send our queued frame (if any), and
    /// pick up the RCP's frame (if it fits). Returns `Ok(true)` if a received
    /// frame was buffered into `rx_frame`.
    async fn push_pull(&mut self) -> Result<bool, SpiTransportError<S::Error, I::Error>> {
        // ---- Build our TX header + payload. ----
        let tx_payload_len = self.tx_pending.unwrap_or(0);

        // How much we're willing to receive this transfer: enough for the RCP's
        // announced pending frame, or at least a small-packet floor.
        let accept_len = if self.rcp_data_len != 0 {
            self.rcp_data_len
        } else {
            SPI_SMALL_PACKET_SIZE
        }
        .max(tx_payload_len);

        // Flag byte: fixed pattern, plus the reset bit on the first transfer.
        self.tx_buf[0] = SPI_FLAG_PATTERN | if self.send_reset { SPI_FLAG_RESET } else { 0 };
        self.tx_buf[1..3].copy_from_slice(&(accept_len as u16).to_le_bytes());
        self.tx_buf[3..5].copy_from_slice(&(tx_payload_len as u16).to_le_bytes());

        // The payload was staged into `tx_buf[SPI_HEADER_SIZE..]` by `send`.
        let transfer_len = SPI_HEADER_SIZE + accept_len.max(tx_payload_len);
        if transfer_len > self.tx_buf.len() {
            return Err(SpiTransportError::FrameTooLarge);
        }

        // Zero the unused RX region so a short/no-op RCP reply reads as zeros.
        for b in self.rx_buf[..transfer_len].iter_mut() {
            *b = 0;
        }

        // ---- Clock the full-duplex transfer. ----
        self.spi
            .transfer(
                &mut self.rx_buf[..transfer_len],
                &self.tx_buf[..transfer_len],
            )
            .await
            .map_err(SpiTransportError::Spi)?;

        // We have now advertised the reset (if any) exactly once.
        self.send_reset = false;

        // ---- Parse the RCP's header. ----
        let flag = self.rx_buf[0];
        if flag == 0x00 || flag == 0xff {
            // Bus idle / RCP not driving MISO — nothing exchanged this transfer.
            return Ok(false);
        }
        if flag & SPI_FLAG_PATTERN_MASK != SPI_FLAG_PATTERN {
            return Err(SpiTransportError::BadHeader);
        }

        let rcp_accept_len = u16::from_le_bytes([self.rx_buf[1], self.rx_buf[2]]) as usize;
        let rcp_data_len = u16::from_le_bytes([self.rx_buf[3], self.rx_buf[4]]) as usize;

        if rcp_accept_len > MAX_SPINEL_FRAME || rcp_data_len > MAX_SPINEL_FRAME {
            return Err(SpiTransportError::BadHeader);
        }

        // Remember the RCP's pending length for sizing the *next* transfer.
        self.rcp_data_len = rcp_data_len;

        // ---- Did our outbound frame get accepted? ----
        if tx_payload_len != 0 && tx_payload_len <= rcp_accept_len {
            self.tx_pending = None;
        }

        // ---- Did we receive a frame (and did it fit in our accept_len)? ----
        let mut got_rx = false;
        if rcp_data_len != 0 && rcp_data_len <= accept_len {
            self.rx_frame[..rcp_data_len]
                .copy_from_slice(&self.rx_buf[SPI_HEADER_SIZE..SPI_HEADER_SIZE + rcp_data_len]);
            self.rx_ready = Some(rcp_data_len);
            // Consumed — the RCP will re-advertise if it has more.
            self.rcp_data_len = 0;
            got_rx = true;
        }

        Ok(got_rx)
    }
}

impl<S, I> SpinelTransport for SpiSpinelTransport<'_, S, I>
where
    S: embedded_hal_async::spi::SpiDevice,
    I: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin,
{
    type Error = SpiTransportError<S::Error, I::Error>;

    async fn send(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
        if frame.len() > MAX_SPINEL_FRAME {
            return Err(SpiTransportError::FrameTooLarge);
        }

        // Stage the payload behind the header region and mark it pending.
        self.tx_buf[SPI_HEADER_SIZE..SPI_HEADER_SIZE + frame.len()].copy_from_slice(frame);
        self.tx_pending = Some(frame.len());

        // Push transfers until the RCP accepts the frame. A transfer may also
        // surface an inbound frame; buffer it for the next `recv`.
        while self.tx_pending.is_some() {
            self.push_pull().await?;
        }

        Ok(())
    }

    async fn recv(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        loop {
            // Hand back an already-buffered frame first.
            if let Some(len) = self.rx_ready.take() {
                let len = len.min(buf.len());
                buf[..len].copy_from_slice(&self.rx_frame[..len]);
                return Ok(len);
            }

            // Decide whether to transfer now or block on the interrupt. We must
            // *poll the INT level* — not just await an edge — to avoid missing a
            // wakeup: the RCP may assert INT for a new frame in the window
            // between our last transfer and this wait, and an edge-only `Wait`
            // impl would then block forever. This mirrors OpenThread's POSIX
            // `spi_interface`, which services the bus when `CheckInterrupt()`
            // (a level read) is asserted and only otherwise waits on the edge.
            //
            // A transfer is due if a prior header already announced a pending
            // frame (`rcp_data_len != 0`) or the INT line is currently asserted.
            // Otherwise, block until the next asserting edge, then re-loop and
            // re-check the level. "Asserted" is per the configured polarity.
            let asserted = match self.int_polarity {
                IntPolarity::ActiveLow => self.int.is_low(),
                IntPolarity::ActiveHigh => self.int.is_high(),
            }
            .map_err(SpiTransportError::Int)?;

            if self.rcp_data_len == 0 && !asserted {
                match self.int_polarity {
                    IntPolarity::ActiveLow => self.int.wait_for_low().await,
                    IntPolarity::ActiveHigh => self.int.wait_for_high().await,
                }
                .map_err(SpiTransportError::Int)?;
            }

            self.push_pull().await?;
        }
    }
}

/// The resources (buffers) needed by a [`SpiSpinelTransport`].
///
/// A separate type so that the (large) buffers can be allocated separately
/// from the transport itself — e.g. in a `static` — rather than travel by
/// value inside the transport through constructor returns, risking transient
/// stack blow-ups on small MCUs.
///
/// `new` is `const`, and the buffers start their life as `MaybeUninit`, so a
/// `SpiTransportResources` can be statically-allocated (e.g. in a
/// `static_cell::ConstStaticCell`) without any stack traffic; they are
/// initialized in-place by [`SpiSpinelTransport::new`].
pub struct SpiTransportResources {
    /// Full SPI transfer buffers (header + payload), reused every transaction.
    tx_buf: MaybeUninit<[u8; SPI_BUF_SIZE]>,
    rx_buf: MaybeUninit<[u8; SPI_BUF_SIZE]>,
    /// A received spinel frame not yet handed to `recv`.
    rx_frame: MaybeUninit<[u8; MAX_SPINEL_FRAME]>,
}

impl SpiTransportResources {
    /// Create a new `SpiTransportResources` instance.
    pub const fn new() -> Self {
        Self {
            tx_buf: MaybeUninit::uninit(),
            rx_buf: MaybeUninit::uninit(),
            rx_frame: MaybeUninit::uninit(),
        }
    }

    /// Initialize the resources, as they start their life as `MaybeUninit` so
    /// as to avoid mem-moves.
    fn init(
        &mut self,
    ) -> (
        &mut [u8; SPI_BUF_SIZE],
        &mut [u8; SPI_BUF_SIZE],
        &mut [u8; MAX_SPINEL_FRAME],
    ) {
        (
            self.tx_buf.write([0; SPI_BUF_SIZE]),
            self.rx_buf.write([0; SPI_BUF_SIZE]),
            self.rx_frame.write([0; MAX_SPINEL_FRAME]),
        )
    }
}

impl Default for SpiTransportResources {
    fn default() -> Self {
        Self::new()
    }
}

/// Error type for [`SpiSpinelTransport`].
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpiTransportError<SpiErr, IntErr> {
    /// The underlying SPI device returned an error.
    Spi(SpiErr),
    /// Waiting on the interrupt line failed.
    Int(IntErr),
    /// The RCP's SPI header failed the pattern/length sanity checks.
    BadHeader,
    /// A spinel frame exceeded the SPI transfer buffer.
    FrameTooLarge,
}
