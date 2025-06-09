use core::fmt;

use anyhow::Error;

use crate::OtError;


impl From<OtError> for Error {
    fn from(value: OtError) -> Self {
        let code: OtErrorCodes = value.into();
        anyhow::anyhow!("OtError: {}", code)
    }
}

/// Represents error codes for the OT library.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OtErrorCodes {
    /// No error.
    None = 0,

    /// Operational failed.
    Failed = 1,

    /// Message was dropped.
    Drop = 2,

    /// Insufficient buffers.
    NoBufs = 3,

    /// No route available.
    NoRoute = 4,

    /// Service is busy and could not service the operation.
    Busy = 5,

    /// Failed to parse message.
    Parse = 6,

    /// Input arguments are invalid.
    InvalidArgs = 7,

    /// Security checks failed.
    Security = 8,

    /// Address resolution requires an address query operation.
    AddressQuery = 9,

    /// Address is not in the source match table.
    NoAddress = 10,

    /// Operation was aborted.
    Abort = 11,

    /// Function or method is not implemented.
    NotImplemented = 12,

    /// Cannot complete due to invalid state.
    InvalidState = 13,

    /// No acknowledgment was received after macMaxFrameRetries (IEEE 802.15.4-2006).
    NoAck = 14,

    /// A transmission could not take place due to activity on the channel, i.e., the CSMA-CA mechanism has failed
    /// (IEEE 802.15.4-2006).
    ChannelAccessFailure = 15,

    /// Not currently attached to a Thread Partition.
    Detached = 16,

    /// FCS check failure while receiving.
    Fcs = 17,

    /// No frame received.
    NoFrameReceived = 18,

    /// Received a frame from an unknown neighbor.
    UnknownNeighbor = 19,

    /// Received a frame from an invalid source address.
    InvalidSourceAddress = 20,

    /// Received a frame filtered by the address filter (allowlisted or denylisted).
    AddressFiltered = 21,

    /// Received a frame filtered by the destination address check.
    DestinationAddressFiltered = 22,

    /// The requested item could not be found.
    NotFound = 23,

    /// The operation is already in progress.
    Already = 24,

    /// The creation of IPv6 address failed.
    Ip6AddressCreationFailure = 26,

    /// Operation prevented by mode flags.
    NotCapable = 27,

    /// Coap response or acknowledgment or DNS, SNTP response not received.
    ResponseTimeout = 28,

    /// Received a duplicated frame.
    Duplicated = 29,

    /// Message is being dropped from reassembly list due to timeout.
    ReassemblyTimeout = 30,

    /// Message is not a TMF Message.
    NotTmf = 31,

    /// Received a non-lowpan data frame.
    NotLowpanDataFrame = 32,

    /// The link margin was too low.
    LinkMarginLow = 34,

    /// Input (CLI) command is invalid.
    InvalidCommand = 35,

    /// Special error code used to indicate success/error status is pending and not yet known.
    Pending = 36,

    /// Request rejected.
    Rejected = 37,

    /// The number of defined errors.
    NumErrors,

    /// Generic error (should not use).
    Generic = 255,
}

impl From<OtError> for OtErrorCodes {
    fn from(value: OtError) -> Self {
        match value.0 {
            0 => OtErrorCodes::None,
            1 => OtErrorCodes::Failed,
            2 => OtErrorCodes::Drop,
            3 => OtErrorCodes::NoBufs,
            4 => OtErrorCodes::NoRoute,
            5 => OtErrorCodes::Busy,
            6 => OtErrorCodes::Parse,
            7 => OtErrorCodes::InvalidArgs,
            8 => OtErrorCodes::Security,
            9 => OtErrorCodes::AddressQuery,
            10 => OtErrorCodes::NoAddress,
            11 => OtErrorCodes::Abort,
            12 => OtErrorCodes::NotImplemented,
            13 => OtErrorCodes::InvalidState,
            14 => OtErrorCodes::NoAck,
            15 => OtErrorCodes::ChannelAccessFailure,
            16 => OtErrorCodes::Detached,
            17 => OtErrorCodes::Fcs,
            18 => OtErrorCodes::NoFrameReceived,
            19 => OtErrorCodes::UnknownNeighbor,
            20 => OtErrorCodes::InvalidSourceAddress,
            21 => OtErrorCodes::AddressFiltered,
            22 => OtErrorCodes::DestinationAddressFiltered,
            23 => OtErrorCodes::NotFound,
            24 => OtErrorCodes::Already,
            26 => OtErrorCodes::Ip6AddressCreationFailure,
            27 => OtErrorCodes::NotCapable,
            28 => OtErrorCodes::ResponseTimeout,
            29 => OtErrorCodes::Duplicated,
            30 => OtErrorCodes::ReassemblyTimeout,
            31 => OtErrorCodes::NotTmf,
            32 => OtErrorCodes::NotLowpanDataFrame,
            34 => OtErrorCodes::LinkMarginLow,
            35 => OtErrorCodes::InvalidCommand,
            36 => OtErrorCodes::Pending,
            37 => OtErrorCodes::Rejected,
            255 => OtErrorCodes::Generic,
            _ => OtErrorCodes::Generic, // Default case for unknown error codes
        }
    }
}

impl fmt::Display for OtErrorCodes {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (name, description) = match self {
            OtErrorCodes::None => ("None", "No error."),
            OtErrorCodes::Failed => ("Failed", "Operational failed."),
            OtErrorCodes::Drop => ("Drop", "Message was dropped."),
            OtErrorCodes::NoBufs => ("NoBufs", "Insufficient buffers."),
            OtErrorCodes::NoRoute => ("NoRoute", "No route available."),
            OtErrorCodes::Busy => ("Busy", "Service is busy and could not service the operation."),
            OtErrorCodes::Parse => ("Parse", "Failed to parse message."),
            OtErrorCodes::InvalidArgs => ("InvalidArgs", "Input arguments are invalid."),
            OtErrorCodes::Security => ("Security", "Security checks failed."),
            OtErrorCodes::AddressQuery => ("AddressQuery", "Address resolution requires an address query operation."),
            OtErrorCodes::NoAddress => ("NoAddress", "Address is not in the source match table."),
            OtErrorCodes::Abort => ("Abort", "Operation was aborted."),
            OtErrorCodes::NotImplemented => ("NotImplemented", "Function or method is not implemented."),
            OtErrorCodes::InvalidState => ("InvalidState", "Cannot complete due to invalid state."),
            OtErrorCodes::NoAck => ("NoAck", "No acknowledgment was received after macMaxFrameRetries."),
            OtErrorCodes::ChannelAccessFailure => ("ChannelAccessFailure", "CSMA-CA mechanism has failed."),
            OtErrorCodes::Detached => ("Detached", "Not currently attached to a Thread Partition."),
            OtErrorCodes::Fcs => ("Fcs", "FCS check failure while receiving."),
            OtErrorCodes::NoFrameReceived => ("NoFrameReceived", "No frame received."),
            OtErrorCodes::UnknownNeighbor => ("UnknownNeighbor", "Received a frame from an unknown neighbor."),
            OtErrorCodes::InvalidSourceAddress => ("InvalidSourceAddress", "Received a frame from an invalid source address."),
            OtErrorCodes::AddressFiltered => ("AddressFiltered", "Received a frame filtered by the address filter."),
            OtErrorCodes::DestinationAddressFiltered => ("DestinationAddressFiltered", "Received a frame filtered by the destination address check."),
            OtErrorCodes::NotFound => ("NotFound", "The requested item could not be found."),
            OtErrorCodes::Already => ("Already", "The operation is already in progress."),
            OtErrorCodes::Ip6AddressCreationFailure => ("Ip6AddressCreationFailure", "The creation of IPv6 address failed."),
            OtErrorCodes::NotCapable => ("NotCapable", "Operation prevented by mode flags."),
            OtErrorCodes::ResponseTimeout => ("ResponseTimeout", "Response or acknowledgment not received."),
            OtErrorCodes::Duplicated => ("Duplicated", "Received a duplicated frame."),
            OtErrorCodes::ReassemblyTimeout => ("ReassemblyTimeout", "Message dropped from reassembly list due to timeout."),
            OtErrorCodes::NotTmf => ("NotTmf", "Message is not a TMF Message."),
            OtErrorCodes::NotLowpanDataFrame => ("NotLowpanDataFrame", "Received a non-lowpan data frame."),
            OtErrorCodes::LinkMarginLow => ("LinkMarginLow", "The link margin was too low."),
            OtErrorCodes::InvalidCommand => ("InvalidCommand", "Input (CLI) command is invalid."),
            OtErrorCodes::Pending => ("Pending", "Success/error status is pending and not yet known."),
            OtErrorCodes::Rejected => ("Rejected", "Request rejected."),
            OtErrorCodes::Generic => ("Generic", "Generic error (should not use)."),
            OtErrorCodes::NumErrors => ("NumErrors", "The number of defined errors."),
        };
        write!(f, "{} ({})", name, description)
    }
}