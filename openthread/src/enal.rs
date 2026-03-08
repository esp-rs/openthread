use core::net::{Ipv4Addr, SocketAddr, SocketAddrV6};

use crate::sys::OT_ERROR_FAILED;
use crate::{OpenThread, OtError, UdpSocket};

impl edge_nal::io::Error for OtError {
    fn kind(&self) -> edge_nal::io::ErrorKind {
        // TODO
        edge_nal::io::ErrorKind::Other
    }
}

impl edge_nal::io::ErrorType for UdpSocket<'_> {
    type Error = OtError;
}

impl edge_nal::UdpSend for UdpSocket<'_> {
    async fn send(&mut self, remote: SocketAddr, data: &[u8]) -> Result<(), Self::Error> {
        UdpSocket::send(self, data, None, &socket_addr_v6(remote)?).await
    }
}

impl edge_nal::UdpReceive for UdpSocket<'_> {
    async fn receive(&mut self, buf: &mut [u8]) -> Result<(usize, SocketAddr), Self::Error> {
        UdpSocket::recv(self, buf)
            .await
            .map(|(n, _, addr)| (n, addr.into()))
    }
}

impl edge_nal::Readable for UdpSocket<'_> {
    async fn readable(&mut self) -> Result<(), Self::Error> {
        UdpSocket::wait_recv_available(self).await
    }
}

impl edge_nal::MulticastV4 for UdpSocket<'_> {
    async fn join_v4(
        &mut self,
        _multicast_addr: Ipv4Addr,
        _interface: Ipv4Addr,
    ) -> Result<(), Self::Error> {
        warn!("Multicast not supported with Thread networks");

        Ok(())
    }

    async fn leave_v4(
        &mut self,
        _multicast_addr: Ipv4Addr,
        _interface: Ipv4Addr,
    ) -> Result<(), Self::Error> {
        warn!("Multicast not supported with Thread networks");

        Ok(())
    }
}

impl edge_nal::MulticastV6 for UdpSocket<'_> {
    async fn join_v6(
        &mut self,
        _multicast_addr: core::net::Ipv6Addr,
        _interface: u32,
    ) -> Result<(), Self::Error> {
        warn!("Multicast not supported with Thread networks");

        Ok(())
    }

    async fn leave_v6(
        &mut self,
        _multicast_addr: core::net::Ipv6Addr,
        _interface: u32,
    ) -> Result<(), Self::Error> {
        warn!("Multicast not supported with Thread networks");

        Ok(())
    }
}

impl edge_nal::UdpSocket for UdpSocket<'_> {}

impl edge_nal::UdpSplit for UdpSocket<'_> {
    type Receive<'a>
        = &'a Self
    where
        Self: 'a;

    type Send<'a>
        = &'a Self
    where
        Self: 'a;

    fn split(&mut self) -> (Self::Receive<'_>, Self::Send<'_>) {
        (&*self, &*self)
    }
}

impl edge_nal::io::ErrorType for &UdpSocket<'_> {
    type Error = OtError;
}

impl edge_nal::UdpSend for &UdpSocket<'_> {
    async fn send(&mut self, remote: SocketAddr, data: &[u8]) -> Result<(), Self::Error> {
        UdpSocket::send(self, data, None, &socket_addr_v6(remote)?).await
    }
}

impl edge_nal::UdpReceive for &UdpSocket<'_> {
    async fn receive(&mut self, buf: &mut [u8]) -> Result<(usize, SocketAddr), Self::Error> {
        UdpSocket::recv(self, buf)
            .await
            .map(|(n, _, addr)| (n, addr.into()))
    }
}

impl edge_nal::Readable for &UdpSocket<'_> {
    async fn readable(&mut self) -> Result<(), Self::Error> {
        UdpSocket::wait_recv_available(self).await
    }
}

impl edge_nal::MulticastV4 for &UdpSocket<'_> {
    async fn join_v4(
        &mut self,
        _multicast_addr: Ipv4Addr,
        _interface: Ipv4Addr,
    ) -> Result<(), Self::Error> {
        warn!("Multicast not supported with Thread networks");

        Ok(())
    }

    async fn leave_v4(
        &mut self,
        _multicast_addr: Ipv4Addr,
        _interface: Ipv4Addr,
    ) -> Result<(), Self::Error> {
        warn!("Multicast not supported with Thread networks");

        Ok(())
    }
}

impl edge_nal::MulticastV6 for &UdpSocket<'_> {
    async fn join_v6(
        &mut self,
        _multicast_addr: core::net::Ipv6Addr,
        _interface: u32,
    ) -> Result<(), Self::Error> {
        warn!("Multicast not supported with Thread networks");

        Ok(())
    }

    async fn leave_v6(
        &mut self,
        _multicast_addr: core::net::Ipv6Addr,
        _interface: u32,
    ) -> Result<(), Self::Error> {
        warn!("Multicast not supported with Thread networks");

        Ok(())
    }
}

impl edge_nal::UdpSocket for &UdpSocket<'_> {}

impl<'a> edge_nal::UdpConnect for OpenThread<'a> {
    type Error = OtError;

    type Socket<'t>
        = UdpSocket<'a>
    where
        Self: 't;

    async fn connect(
        &self,
        _local: SocketAddr,
        remote: SocketAddr,
    ) -> Result<Self::Socket<'_>, Self::Error> {
        // TODO: Local
        UdpSocket::connect(self.clone(), &socket_addr_v6(remote)?)
    }
}

impl<'a> edge_nal::UdpBind for OpenThread<'a> {
    type Error = OtError;

    type Socket<'t>
        = UdpSocket<'a>
    where
        Self: 't;

    async fn bind(&self, addr: SocketAddr) -> Result<Self::Socket<'_>, Self::Error> {
        UdpSocket::bind(self.clone(), &socket_addr_v6(addr)?)
    }
}

fn socket_addr_v6(addr: SocketAddr) -> Result<SocketAddrV6, OtError> {
    match addr {
        SocketAddr::V4(_) => Err(OtError::new(OT_ERROR_FAILED)),
        SocketAddr::V6(v6) => Ok(v6),
    }
}
