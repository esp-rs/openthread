use core::{
    default, fmt,
    net::{Ipv4Addr, Ipv6Addr},
};

use openthread_sys::{
    otError, otIp4Address, otIp4Address__bindgen_ty_1, otIp4ExtractFromIp6Address, otIp6Address,
    otIp6Address__bindgen_ty_1, otNat64GetTranslatorState, otNat64State,
    otNat64State_OT_NAT64_STATE_ACTIVE, otNat64State_OT_NAT64_STATE_DISABLED,
    otNat64State_OT_NAT64_STATE_IDLE, otNat64State_OT_NAT64_STATE_NOT_RUNNING,
    otNat64SynthesizeIp6Address,
};

use crate::{OpenThread, OtError};

pub enum Nat64State {
    Disabled = 0,
    NotRunning,
    Idel,
    Active,
    Unkown,
}

impl Nat64State {
    fn from_ot_state(state: otNat64State) -> Self {
        match state {
            otNat64State_OT_NAT64_STATE_DISABLED => Nat64State::Disabled,
            otNat64State_OT_NAT64_STATE_NOT_RUNNING => Nat64State::NotRunning,
            otNat64State_OT_NAT64_STATE_IDLE => Nat64State::Idel,
            otNat64State_OT_NAT64_STATE_ACTIVE => Nat64State::Active,
            _ => Nat64State::Unkown,
        }
    }
}

#[derive(Debug)]
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

impl<'a> OpenThread<'a> {
    pub fn nat64_get_translator_state(&self) -> Nat64State {
        let mut ot = self.activate();
        let state = ot.state();

        Nat64State::from_ot_state(unsafe { otNat64GetTranslatorState(state.ot.instance) })
    }

    pub fn nat64_synthezise_ipv6_address(&self, ipv4: &Ipv4Addr) -> Ipv6Addr {
        let mut ot = self.activate();
        let state = ot.state();

        let mut ipv6 = otIp6Address::default();
        let ipv4 = ipv4_to_ot_ipv4(ipv4);

        unsafe {
            otNat64SynthesizeIp6Address(state.ot.instance, &ipv4, &mut ipv6);
        }

        ot_ipv6_to_ipv6(&ipv6)
    }
}

pub fn ipv4_extract_from_ipv6_address(
    prefix_length: u8,
    ipv6: &Ipv6Addr,
) -> Result<Ipv4Addr, Nat64Error> {
    let valid_prefix_lengths: [u8; 6] = [32, 40, 48, 56, 64, 96];

    if !valid_prefix_lengths.contains(&prefix_length) {
        return Err(Nat64Error::InvalidPrefixLength(prefix_length));
    }

    let mut ot_ipv4 = otIp4Address::default();

    unsafe { otIp4ExtractFromIp6Address(prefix_length, &ipv6_to_ot_ipv6(ipv6), &mut ot_ipv4) }

    Ok(ot_ipv4_to_ipv4(&ot_ipv4))
}

pub fn test_ip_address_conversion() -> bool {
    let mut failed = false;

    let ipv4_original = Ipv4Addr::new(192, 168, 1, 1);
    let ipv6 = Ipv6Addr::new(1, 2, 3, 4, 5, 6, 7, 8);

    if !ot_ipv4_to_ipv4(&ipv4_to_ot_ipv4(&ipv4_original)) != ipv4_original {
        failed = false;
    }

    if !ot_ipv6_to_ipv6(&ipv6_to_ot_ipv6(&ipv6)) != ipv6{
        failed = false;
    }

    failed
}

fn ipv4_to_ot_ipv4(ipv4: &Ipv4Addr) -> otIp4Address {
    otIp4Address {
        mFields: otIp4Address__bindgen_ty_1 {
            m32: ipv4.to_bits(),
        },
    }
}

fn ipv6_to_ot_ipv6(ipv6: &Ipv6Addr) -> otIp6Address {
    otIp6Address {
        mFields: otIp6Address__bindgen_ty_1 {
            m16: ipv6.segments(),
        },
    }
}

fn ot_ipv4_to_ipv4(ipv4: &otIp4Address) -> Ipv4Addr {
    unsafe { Ipv4Addr::from_bits(ipv4.mFields.m32) }
}

fn ot_ipv6_to_ipv6(ipv6: &otIp6Address) -> Ipv6Addr {
    unsafe { ipv6.mFields.m8.into() }
}
