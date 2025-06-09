use anyhow::Error;

use crate::OtError;


impl From<OtError> for Error {
    fn from(value: OtError) -> Self {
        anyhow::anyhow!("OtError: {:?}", value)
    }
}