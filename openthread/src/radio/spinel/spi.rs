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
//! has not been run against a real `ot-rcp` over SPI. It also implements the
//! spec's *baseline* only: no optional on-SPI CRC, no receive-alignment
//! allowance, and a simple "retry until accepted" loop rather than the
//! reference's backoff/rate-limiting. Treat as experimental. See the [`super`]
//! module docs.

use super::{SpinelTransport, MAX_SPINEL_FRAME};

/// SPI framing header size (flag byte + accept-len + data-len, all before the
/// payload). See [`SpiSpinelTransport`] for the layout.
const SPI_HEADER_SIZE: usize = 5;

/// Flag-byte constants (from OpenThread's `spi_frame.hpp`).
const SPI_FLAG_RESET: u8 = 1 << 7;
const SPI_FLAG_PATTERN: u8 = 0x02;
const SPI_FLAG_PATTERN_MASK: u8 = 0x03;

/// A minimum SPI transfer size, so a small frame the RCP wants to send us can be
/// picked up in a single transaction even before we know its length.
const SPI_SMALL_PACKET_SIZE: usize = 32;

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
/// - **An interrupt line** (`int`, active-low): the RCP asserts it to say "I have
///   a frame for you, please clock a transfer". This is the peripheral-initiated
///   channel raw SPI lacks. This transport [`Wait`](embedded_hal_async::digital::Wait)s
///   on it before reading.
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
/// // `spi: embedded_hal_async::spi::SpiDevice`, `int: embedded_hal_async::digital::Wait`
/// let radio = SpinelRadio::new(SpiSpinelTransport::new(spi, int));
/// ```
pub struct SpiSpinelTransport<S, I> {
    spi: S,
    int: I,
    /// Full SPI transfer buffers (header + payload), reused every transaction.
    tx_buf: [u8; SPI_HEADER_SIZE + MAX_SPINEL_FRAME],
    rx_buf: [u8; SPI_HEADER_SIZE + MAX_SPINEL_FRAME],
    /// A single queued outbound spinel frame waiting to be accepted by the RCP.
    tx_pending: Option<usize>,
    /// A single received spinel frame not yet handed to `recv`, `[.. len]` of
    /// `rx_frame`.
    rx_frame: [u8; MAX_SPINEL_FRAME],
    rx_ready: Option<usize>,
    /// The RCP's last-advertised pending data length (learned from a prior header
    /// so the next transfer is sized to fit it).
    rcp_data_len: usize,
    /// Whether we still owe the RCP the reset flag on our next header (set on the
    /// very first transfer so the RCP knows the host came up fresh).
    send_reset: bool,
}

impl<S, I> SpiSpinelTransport<S, I> {
    /// Create an SPI spinel transport over SPI device `spi` and interrupt line
    /// `int` (active-low: asserted low when the RCP has data to send).
    pub const fn new(spi: S, int: I) -> Self {
        Self {
            spi,
            int,
            tx_buf: [0; SPI_HEADER_SIZE + MAX_SPINEL_FRAME],
            rx_buf: [0; SPI_HEADER_SIZE + MAX_SPINEL_FRAME],
            tx_pending: None,
            rx_frame: [0; MAX_SPINEL_FRAME],
            rx_ready: None,
            rcp_data_len: 0,
            send_reset: true,
        }
    }
}

impl<S, I> SpiSpinelTransport<S, I>
where
    S: embedded_hal_async::spi::SpiDevice,
    I: embedded_hal_async::digital::Wait,
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

impl<S, I> SpinelTransport for SpiSpinelTransport<S, I>
where
    S: embedded_hal_async::spi::SpiDevice,
    I: embedded_hal_async::digital::Wait,
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

            // If the RCP has not yet told us it has data, wait for its interrupt.
            // (If a prior header already announced a pending frame, go straight to
            // a transfer to fetch it.)
            if self.rcp_data_len == 0 {
                self.int
                    .wait_for_low()
                    .await
                    .map_err(SpiTransportError::Int)?;
            }

            self.push_pull().await?;
        }
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
