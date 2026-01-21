#include "platform/ns3/transport/ns3_socket_transport.h"

#include <utility>

#include "ns3/udp-socket-factory.h"

#include "platform/ns3/radio_environment.h"

namespace sim {

Ns3SocketTransport::Ns3SocketTransport(::ns3::Ptr<::ns3::Node> node) {
  m_ep = sim::RadioEnvironment::Get().Install(node);

  if (m_ep.socket) {
    m_ep.socket->SetRecvCallback(::ns3::MakeCallback(&Ns3SocketTransport::OnSocketRx, this));
  }
}

void Ns3SocketTransport::RegisterPeer(uint8_t id, uint32_t address) {
  m_id_to_ip[id] = ::ns3::Ipv4Address(address);
}

void Ns3SocketTransport::SetRxCallback(RxCallback cb) {
  m_rx_cb = std::move(cb);
}

void Ns3SocketTransport::SendUnicast(uint8_t dst_id, const Bytes& bytes) {
  if (!m_ep.socket || bytes.empty()) {
    return;
  }

  auto it = m_id_to_ip.find(dst_id);
  if (it == m_id_to_ip.end()) {
    return;
  }

  ::ns3::Ptr<::ns3::Packet> p = ::ns3::Create<::ns3::Packet>(bytes.data(), static_cast<uint32_t>(bytes.size()));
  ::ns3::InetSocketAddress addr(it->second, sim::RadioEnvironment::Get().Port());
  m_ep.socket->SendTo(p, 0, addr);
}

void Ns3SocketTransport::SendBroadcast(const Bytes& bytes) {
  if (!m_ep.socket || bytes.empty()) {
    return;
  }

  // Emulate swarm broadcast by unicast fan-out to all known peers.
  const auto peers = sim::RadioEnvironment::Get().AllIps();
  for (const auto& ip : peers) {
    if (ip == m_ep.ip) {
      continue;
    }
    ::ns3::Ptr<::ns3::Packet> p = ::ns3::Create<::ns3::Packet>(bytes.data(), static_cast<uint32_t>(bytes.size()));
    ::ns3::InetSocketAddress addr(ip, sim::RadioEnvironment::Get().Port());
    m_ep.socket->SendTo(p, 0, addr);
  }
}

void Ns3SocketTransport::OnSocketRx(::ns3::Ptr<::ns3::Socket> sock) {
  if (!sock || !m_rx_cb) {
    return;
  }

  ::ns3::Address from;
  ::ns3::Ptr<::ns3::Packet> p;
  while ((p = sock->RecvFrom(from))) {
    const uint32_t size = p->GetSize();
    if (size == 0) {
      continue;
    }

    // Enforce coverage radius at receive-time.
    // This is important because our broadcast is implemented as a unicast fan-out.
    if (::ns3::InetSocketAddress::IsMatchingType(from)) {
      const auto fromInet = ::ns3::InetSocketAddress::ConvertFrom(from);
      const auto fromIp = fromInet.GetIpv4();

      const auto env = sim::RadioEnvironment::Get();
      ::ns3::Ptr<::ns3::Node> srcNode = env.FindNodeByIp(fromIp);
      ::ns3::Ptr<::ns3::Node> dstNode = sock->GetNode();
      if (srcNode && dstNode) {
        auto srcMob = srcNode->GetObject<::ns3::MobilityModel>();
        auto dstMob = dstNode->GetObject<::ns3::MobilityModel>();
        if (srcMob && dstMob) {
          const double dist = ::ns3::CalculateDistance(srcMob->GetPosition(), dstMob->GetPosition());
          if (dist > env.MaxRangeMeters()) {
            continue;
          }
        }
      }
    }

    Bytes bytes(size);
    p->CopyData(bytes.data(), size);
    m_rx_cb(bytes);
  }
}

}  // namespace sim
