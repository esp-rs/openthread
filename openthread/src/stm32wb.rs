//! Provides a fake radio

use crate::{Capabilities, MacCapabilities, Radio, RadioErrorKind};

pub struct OffloadRadio;

impl Radio for OffloadRadio {
    type Error = RadioErrorKind;

    const CAPS: crate::Capabilities = Capabilities::all();

    const MAC_CAPS: MacCapabilities = MacCapabilities::all();

    async fn set_config(&mut self, _config: &crate::Config) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn receive(&mut self, _psdu_buf: &mut [u8]) -> Result<crate::PsduMeta, Self::Error> {
        unreachable!()
    }

    async fn transmit(
        &mut self,
        _psdu: &[u8],
        _cca: bool,
        _ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<crate::PsduMeta>, Self::Error> {
        unreachable!()
    }
}
