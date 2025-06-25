use core::{
    fmt,
    net::{Ipv4Addr, Ipv6Addr},
};

use openthread_sys::{
    otIp4Address, otIp4Address__bindgen_ty_1, otIp4ExtractFromIp6Address, otIp6Address,
    otIp6Address__bindgen_ty_1, otNat64SynthesizeIp6Address,
};

use crate::{ot, OpenThread, OtError};

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub enum Nat64Error {
    InvalidPrefixLength(u8),
}

impl fmt::Display for Nat64Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Nat64Error::InvalidPrefixLength(length) => {
                write!(f, "Invalid prefix length: {}", length)
            }
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Nat64Error {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Nat64Error::InvalidPrefixLength(length) => {
                defmt::write!(fmt, "Invalid prefix length: {}", length)
            }
        }
    }
}

impl OpenThread<'_> {
    /// Creates the IPv6 address by performing NAT64 address translation from the preferred NAT64 prefix and the given IPv4
    /// address as specified in RFC 6052.
    pub fn nat64_synthesize_ipv6(&self, ipv4: &Ipv4Addr) -> Result<Ipv6Addr, OtError> {
        let mut ot = self.activate();
        let state = ot.state();

        let mut ipv6 = otIp6Address::default();
        let ipv4 = ipv4_to_ot_ipv4(ipv4);

        ot!(unsafe { otNat64SynthesizeIp6Address(state.ot.instance, &ipv4, &mut ipv6) })?;

        Ok(ot_ipv6_to_ipv6(&ipv6))
    }
}

/// Returns IPv4 address by performing NAT64 address translation from IPv6 as specified in RFC 6052.
///
/// The NAT64 `prefix_length` MUST be one of the following values: 32, 40, 48, 56, 64, or 96, otherwise the
/// function returns `Nat64Error::InvalidPrefixLength`
pub fn ipv4_extract_from_ipv6(prefix_length: u8, ipv6: &Ipv6Addr) -> Result<Ipv4Addr, Nat64Error> {
    let valid_prefix_lengths: [u8; 6] = [32, 40, 48, 56, 64, 96];

    if !valid_prefix_lengths.contains(&prefix_length) {
        return Err(Nat64Error::InvalidPrefixLength(prefix_length));
    }

    let mut ot_ipv4 = otIp4Address::default();

    unsafe { otIp4ExtractFromIp6Address(prefix_length, &ipv6_to_ot_ipv6(ipv6), &mut ot_ipv4) }

    Ok(ot_ipv4_to_ipv4(&ot_ipv4))
}

fn ipv4_to_ot_ipv4(ipv4: &Ipv4Addr) -> otIp4Address {
    otIp4Address {
        mFields: otIp4Address__bindgen_ty_1 { m8: ipv4.octets() },
    }
}

fn ipv6_to_ot_ipv6(ipv6: &Ipv6Addr) -> otIp6Address {
    otIp6Address {
        mFields: otIp6Address__bindgen_ty_1 { m8: ipv6.octets() },
    }
}

fn ot_ipv4_to_ipv4(ipv4: &otIp4Address) -> Ipv4Addr {
    unsafe { ipv4.mFields.m8 }.into()
}

fn ot_ipv6_to_ipv6(ipv6: &otIp6Address) -> Ipv6Addr {
    unsafe { ipv6.mFields.m8 }.into()
}
