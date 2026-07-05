//! [`UartSpinelTransport`]: spinel frames over a UART.
//!
//! A UART is a raw, unframed byte stream, so this transport owns the framing:
//! it HDLC byte-stuffs each spinel frame on the way out (RFC 1662 + CRC-16/X.25
//! FCS) and runs an incremental HDLC decoder on the way in — exactly OpenThread's
//! POSIX `hdlc_interface`. (SPI does its own framing and needs no HDLC — see
//! [`super::spi`].)
//!
//! This path is validated against a real `ot-rcp` over USB CDC-ACM (see the
//! [`super`] module docs).

use super::{SpinelTransport, MAX_SPINEL_FRAME};

// ---------------------------------------------------------------------------
// HDLC framing (RFC 1662 byte-stuffing + CRC-16/X.25 FCS).
// ---------------------------------------------------------------------------

const HDLC_FLAG: u8 = 0x7e;
const HDLC_ESCAPE: u8 = 0x7d;
const HDLC_XOR: u8 = 0x20;
const HDLC_FCS_INIT: u16 = 0xffff;
const HDLC_FCS_GOOD: u16 = 0xf0b8;

/// CRC-16/X.25 (reflected 0x1021) FCS, one byte at a time.
fn hdlc_fcs_update(fcs: u16, byte: u8) -> u16 {
    let mut fcs = fcs;
    let mut b = byte;
    for _ in 0..8 {
        if ((fcs ^ (b as u16)) & 0x0001) != 0 {
            fcs = (fcs >> 1) ^ 0x8408;
        } else {
            fcs >>= 1;
        }
        b >>= 1;
    }
    fcs
}

fn hdlc_needs_escape(byte: u8) -> bool {
    matches!(byte, HDLC_FLAG | HDLC_ESCAPE) || byte == 0x11 || byte == 0x13
}

/// HDLC-encode `payload` into `out`, returning the encoded length (or `None` if
/// `out` is too small). Wraps with flags, byte-stuffs, and appends the FCS.
fn hdlc_encode(payload: &[u8], out: &mut [u8]) -> Option<usize> {
    let mut n = 0;

    let push = |out: &mut [u8], n: &mut usize, byte: u8| -> bool {
        if hdlc_needs_escape(byte) {
            if *n + 2 > out.len() {
                return false;
            }
            out[*n] = HDLC_ESCAPE;
            out[*n + 1] = byte ^ HDLC_XOR;
            *n += 2;
        } else {
            if *n + 1 > out.len() {
                return false;
            }
            out[*n] = byte;
            *n += 1;
        }
        true
    };

    if n >= out.len() {
        return None;
    }
    out[n] = HDLC_FLAG;
    n += 1;

    let mut fcs = HDLC_FCS_INIT;
    for &b in payload {
        fcs = hdlc_fcs_update(fcs, b);
        if !push(out, &mut n, b) {
            return None;
        }
    }

    // Append the (complemented) FCS, little-endian, byte-stuffed.
    fcs ^= 0xffff;
    if !push(out, &mut n, (fcs & 0xff) as u8) {
        return None;
    }
    if !push(out, &mut n, (fcs >> 8) as u8) {
        return None;
    }

    if n >= out.len() {
        return None;
    }
    out[n] = HDLC_FLAG;
    n += 1;

    Some(n)
}

/// Incremental HDLC decoder: fed bytes from the transport, yields complete,
/// FCS-checked spinel frames.
struct HdlcDecoder {
    buf: [u8; MAX_SPINEL_FRAME],
    len: usize,
    fcs: u16,
    escaped: bool,
    in_frame: bool,
}

impl HdlcDecoder {
    const fn new() -> Self {
        Self {
            buf: [0; MAX_SPINEL_FRAME],
            len: 0,
            fcs: HDLC_FCS_INIT,
            escaped: false,
            in_frame: false,
        }
    }

    fn reset(&mut self) {
        self.len = 0;
        self.fcs = HDLC_FCS_INIT;
        self.escaped = false;
        self.in_frame = true;
    }

    /// Push one received byte. Returns `Some(len)` when a complete, valid frame
    /// is available in `self.buf[..len - 2]` (payload, FCS stripped).
    fn push(&mut self, byte: u8) -> Option<usize> {
        match byte {
            HDLC_FLAG => {
                let complete = self.in_frame && self.len >= 2 && self.fcs == HDLC_FCS_GOOD;

                let payload_len = self.len.saturating_sub(2);
                self.reset();

                complete.then_some(payload_len)
            }
            HDLC_ESCAPE => {
                self.escaped = true;
                None
            }
            _ => {
                if !self.in_frame {
                    return None;
                }
                let b = if self.escaped {
                    self.escaped = false;
                    byte ^ HDLC_XOR
                } else {
                    byte
                };
                if self.len < self.buf.len() {
                    self.buf[self.len] = b;
                    self.len += 1;
                    self.fcs = hdlc_fcs_update(self.fcs, b);
                } else {
                    // Overflow — drop the frame.
                    self.in_frame = false;
                }
                None
            }
        }
    }
}

// ---------------------------------------------------------------------------
// UartSpinelTransport
// ---------------------------------------------------------------------------

/// A [`SpinelTransport`] over a UART (or any `embedded-io-async` byte stream).
///
/// A UART is a raw, unframed byte stream, so this transport HDLC-frames each
/// spinel frame on the way out (byte-stuffing + FCS) and runs an incremental
/// HDLC decoder on the way in — exactly OpenThread's POSIX `hdlc_interface`.
///
/// Wrap any full-duplex byte stream implementing [`embedded_io_async::Read`] +
/// [`embedded_io_async::Write`] (e.g. an embassy UART):
///
/// ```ignore
/// let radio = SpinelRadio::new(UartSpinelTransport::new(uart));
/// ```
pub struct UartSpinelTransport<U> {
    uart: U,
    /// HDLC-encode scratch (worst case: every byte escaped, plus flags + FCS).
    tx_hdlc: [u8; MAX_SPINEL_FRAME * 2 + 4],
    /// Incremental decoder for the inbound byte stream.
    decoder: HdlcDecoder,
    /// Read scratch pulled from the UART in chunks.
    rx_chunk: [u8; 128],
    /// Bytes buffered in `rx_chunk` not yet fed to the decoder, `[pos, fill)`.
    rx_pos: usize,
    rx_fill: usize,
}

impl<U> UartSpinelTransport<U> {
    /// Create a UART spinel transport over `uart`.
    pub const fn new(uart: U) -> Self {
        Self {
            uart,
            tx_hdlc: [0; MAX_SPINEL_FRAME * 2 + 4],
            decoder: HdlcDecoder::new(),
            rx_chunk: [0; 128],
            rx_pos: 0,
            rx_fill: 0,
        }
    }
}

impl<U> SpinelTransport for UartSpinelTransport<U>
where
    U: embedded_io_async::Read + embedded_io_async::Write,
{
    type Error = UartTransportError<U::Error>;

    async fn send(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
        let n = hdlc_encode(frame, &mut self.tx_hdlc).ok_or(UartTransportError::FrameTooLarge)?;
        self.uart
            .write_all(&self.tx_hdlc[..n])
            .await
            .map_err(UartTransportError::Io)
    }

    async fn recv(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        loop {
            // Drain whatever is already buffered, feeding the HDLC decoder.
            while self.rx_pos < self.rx_fill {
                let byte = self.rx_chunk[self.rx_pos];
                self.rx_pos += 1;

                if let Some(len) = self.decoder.push(byte) {
                    let len = len.min(buf.len());
                    buf[..len].copy_from_slice(&self.decoder.buf[..len]);
                    return Ok(len);
                }
            }

            // Refill from the UART.
            let n = self
                .uart
                .read(&mut self.rx_chunk)
                .await
                .map_err(UartTransportError::Io)?;
            if n == 0 {
                return Err(UartTransportError::Eof);
            }
            self.rx_pos = 0;
            self.rx_fill = n;
        }
    }
}

/// Error type for [`UartSpinelTransport`].
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UartTransportError<E> {
    /// The underlying UART returned an error.
    Io(E),
    /// The UART reached end-of-stream (a `read` returned 0 bytes).
    Eof,
    /// A spinel frame exceeded the HDLC encode buffer.
    FrameTooLarge,
}
