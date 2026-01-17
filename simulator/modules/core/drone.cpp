#include "modules/core/drone.h"

#include <cmath>
#include <cstring>

#include "ns3/constant-position-mobility-model.h"


Drone::Drone(uint32_t id, ::ns3::Ptr<::ns3::Node> node)
    : m_id(id),
      m_node(node),
        m_comm(
          static_cast<uint8_t>(id),
          [this](::ns3::Ipv4Address dst, ::ns3::Ptr<::ns3::Packet> p) { SendTo(dst, p); },
          [this](::ns3::Ptr<::ns3::Packet> p) { SendBroadcast(p); }
      ) {
  if (!m_node) {
    return;
  }

  m_mobility = m_node->GetObject<::ns3::ConstantPositionMobilityModel>();
  if (!m_mobility) {
    m_mobility = ::ns3::CreateObject<::ns3::ConstantPositionMobilityModel>();
    m_node->AggregateObject(m_mobility);
  }

  m_radio = sim::RadioEnvironment::Get().Install(m_node);
  m_ip = m_radio.ip;

  if (m_radio.socket) {
    m_radio.socket->SetRecvCallback(::ns3::MakeCallback(&Drone::SocketRx, this));
  }

  m_comm.SetReceiveHandler([this](const ::Packet& pkt) { DispatchPacket(pkt); });

  m_flooding = std::make_unique<Flooding>(
      static_cast<uint8_t>(m_id),
      m_comm,
      [this]() { return IsBaseReachable(); }
  );

  m_dispatcher.SetFloodingManager(m_flooding.get());
  m_dispatcher.SetFallbackHandler([this](const ::Packet& pkt) { HandleCorePacket(pkt); });
}


void Drone::Start() {
  ::ns3::Simulator::Schedule(::ns3::Seconds(0.0), ::ns3::MakeCallback(&Drone::OnTick, this));
}


void Drone::OnTick() {
  if (m_mobility) {
    ::ns3::Vector pos = m_mobility->GetPosition();
    m_mobility->SetPosition(pos);
  }
  MaybeProbeBase();
  ::ns3::Simulator::Schedule(::ns3::Seconds(m_dt), ::ns3::MakeCallback(&Drone::OnTick, this));
}


void Drone::SocketRx(::ns3::Ptr<::ns3::Socket> sock) {
  ::ns3::Address from;
  ::ns3::Ptr<::ns3::Packet> p;
  while ((p = sock->RecvFrom(from))) {
    if (::ns3::InetSocketAddress::IsMatchingType(from)) {
      ::ns3::InetSocketAddress src = ::ns3::InetSocketAddress::ConvertFrom(from);
      std::cout << "[Drone " << m_id << "] RX " << p->GetSize() << "B from " << src.GetIpv4() << ":" << src.GetPort()
                << std::endl;
    }
    m_comm.HandleRadioPacket(p);
  }
}


void Drone::SendTo(::ns3::Ipv4Address dst, ::ns3::Ptr<::ns3::Packet> p) {
  if (!m_radio.socket || !p) {
    return;
  }
  ::ns3::InetSocketAddress addr(dst, sim::RadioEnvironment::Get().Port());
  m_radio.socket->SendTo(p, 0, addr);
}


void Drone::SendBroadcast(::ns3::Ptr<::ns3::Packet> p) {
  if (!m_radio.socket || !p) {
    return;
  }
  // Emulate swarm broadcast by unicast fan-out to all known peers.
  // This still respects radio range limits and keeps Drone<->Drone traffic explicit.
  const auto peers = sim::RadioEnvironment::Get().AllIps();
  for (const auto& ip : peers) {
    if (ip == m_ip) {
      continue;
    }
    SendTo(ip, p->Copy());
  }
}


void Drone::OnReceive(::ns3::Ptr<::ns3::Packet> p) {}


void Drone::SetBaseStation(uint8_t base_id, ::ns3::Ipv4Address base_ip) {
  m_base_id = base_id;
  m_base_ip = base_ip;
  m_comm.RegisterPeer(base_id, base_ip);
  if (m_flooding) {
    m_flooding->SetBaseId(base_id);
  }
}


void Drone::DispatchPacket(const ::Packet& pkt) {
  if (pkt.payload.empty()) {
    return;
  }

  if (pkt.dst != static_cast<uint8_t>(m_id) && pkt.dst != BROADCAST_ID) {
    return;
  }

  m_dispatcher.HandlePacket(pkt);
}


void Drone::HandleCorePacket(const ::Packet& pkt) {
  const uint8_t type = pkt.payload[0];
  if (type == static_cast<uint8_t>(SimMsgType::NODE_INFO_REQUEST)) {
    if (pkt.payload.size() < sizeof(NodeInfoRequestMsg)) {
      return;
    }
    NodeInfoRequestMsg msg;
    std::memcpy(&msg, pkt.payload.data(), sizeof(msg));
    HandleNodeInfoRequest(msg);
    return;
  }

  if (type == static_cast<uint8_t>(SimMsgType::NODE_INFO_RESPONSE)) {
    if (pkt.payload.size() < sizeof(NodeInfoResponseMsg)) {
      return;
    }
    NodeInfoResponseMsg msg;
    std::memcpy(&msg, pkt.payload.data(), sizeof(msg));
    HandleNodeInfoResponse(msg);
    return;
  }
}


void Drone::HandleNodeInfoRequest(const NodeInfoRequestMsg& msg) {
  if (!m_mobility) {
    return;
  }
  ::ns3::Vector pos = m_mobility->GetPosition();
  NodeInfoResponseMsg resp;
  resp.node_id = static_cast<uint8_t>(m_id);
  resp.x = static_cast<float>(pos.x);
  resp.y = static_cast<float>(pos.y);
  resp.z = static_cast<float>(pos.z);

  ::Packet out;
  out.src = static_cast<uint8_t>(m_id);
  out.dst = msg.requester_id;
  out.payload.resize(sizeof(resp));
  std::memcpy(out.payload.data(), &resp, sizeof(resp));
  m_comm.send(out);
}


void Drone::HandleNodeInfoResponse(const NodeInfoResponseMsg& msg) {
  if (msg.node_id != m_base_id) {
    return;
  }
  m_last_base_reply_time = ::ns3::Simulator::Now().GetSeconds();
  m_has_base_position = true;
  m_last_base_position = ::ns3::Vector(msg.x, msg.y, msg.z);
}


void Drone::MaybeProbeBase() {
  const double now = ::ns3::Simulator::Now().GetSeconds();
  if (m_last_probe_time >= 0.0 && (now - m_last_probe_time) < m_probe_interval) {
    return;
  }
  m_last_probe_time = now;

  NodeInfoRequestMsg req;
  req.requester_id = static_cast<uint8_t>(m_id);

  ::Packet out;
  out.src = static_cast<uint8_t>(m_id);
  out.dst = m_base_id;
  out.payload.resize(sizeof(req));
  std::memcpy(out.payload.data(), &req, sizeof(req));
  m_comm.send(out);
}


bool Drone::IsBaseReachable() const {
  if (!m_has_base_position || !m_mobility) {
    return false;
  }
  const double now = ::ns3::Simulator::Now().GetSeconds();
  if (m_last_base_reply_time < 0.0 || (now - m_last_base_reply_time) > m_base_reply_timeout) {
    return false;
  }

  const ::ns3::Vector pos = m_mobility->GetPosition();
  const double dx = pos.x - m_last_base_position.x;
  const double dy = pos.y - m_last_base_position.y;
  const double dz = pos.z - m_last_base_position.z;
  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
  return dist <= sim::RadioEnvironment::Get().MaxRangeMeters();
}
