#include "modules/core/base_station.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "ns3/constant-position-mobility-model.h"


BaseStation::BaseStation(uint32_t id, ::ns3::Ptr<::ns3::Node> node)
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
    m_radio.socket->SetRecvCallback(::ns3::MakeCallback(&BaseStation::SocketRx, this));
  }

  m_comm.SetReceiveHandler([this](const ::Packet& pkt) { DispatchPacket(pkt); });
  m_dispatcher.SetFallbackHandler([this](const ::Packet& pkt) { HandleCorePacket(pkt); });
}


void BaseStation::Start() {
  ::ns3::Simulator::Schedule(::ns3::Seconds(0.0), ::ns3::MakeCallback(&BaseStation::OnTick, this));
}


void BaseStation::OnTick() {
  if (m_mobility) {
    ::ns3::Vector pos = m_mobility->GetPosition();
    m_mobility->SetPosition(pos);
  }
  ::ns3::Simulator::Schedule(::ns3::Seconds(m_dt), ::ns3::MakeCallback(&BaseStation::OnTick, this));
}


void BaseStation::SocketRx(::ns3::Ptr<::ns3::Socket> sock) {
  ::ns3::Address from;
  ::ns3::Ptr<::ns3::Packet> p;
  while ((p = sock->RecvFrom(from))) {
    if (::ns3::InetSocketAddress::IsMatchingType(from)) {
      ::ns3::InetSocketAddress src = ::ns3::InetSocketAddress::ConvertFrom(from);
      std::cout << "[BaseStation " << m_id << "] RX " << p->GetSize() << "B from " << src.GetIpv4() << ":" << src.GetPort()
                << std::endl;
    }
    m_comm.HandleRadioPacket(p);
  }
}


void BaseStation::SendTo(::ns3::Ipv4Address dst, ::ns3::Ptr<::ns3::Packet> p) {
  if (!m_radio.socket || !p) {
    return;
  }
  ::ns3::InetSocketAddress addr(dst, sim::RadioEnvironment::Get().Port());
  m_radio.socket->SendTo(p, 0, addr);
}


void BaseStation::SendBroadcast(::ns3::Ptr<::ns3::Packet> p) {
  if (!m_radio.socket || !p) {
    return;
  }
  const auto peers = sim::RadioEnvironment::Get().AllIps();
  for (const auto& ip : peers) {
    if (ip == m_ip) {
      continue;
    }
    SendTo(ip, p->Copy());
  }
}


void BaseStation::OnReceive(::ns3::Ptr<::ns3::Packet> p) {}


void BaseStation::RegisterDrone(uint8_t id, ::ns3::Ipv4Address ip, ::ns3::Ptr<::ns3::ConstantPositionMobilityModel> mobility) {
  m_comm.RegisterPeer(id, ip);
  m_peers.push_back({id, ip, mobility});
}


void BaseStation::RequestFlood(uint16_t flood_id) {
  ::ns3::Simulator::ScheduleNow(&BaseStation::FloodRequestWorker, this, flood_id);
}


void BaseStation::RequestNodeInfo(uint8_t drone_id) {
  NodeInfoRequestMsg req;
  req.requester_id = static_cast<uint8_t>(m_id);

  ::Packet out;
  out.src = static_cast<uint8_t>(m_id);
  out.dst = drone_id;
  out.payload.resize(sizeof(req));
  std::memcpy(out.payload.data(), &req, sizeof(req));
  m_comm.send(out);
}


void BaseStation::DispatchPacket(const ::Packet& pkt) {
  if (pkt.payload.empty()) {
    return;
  }

  if (pkt.dst != static_cast<uint8_t>(m_id) && pkt.dst != BROADCAST_ID) {
    return;
  }

  const uint8_t type = pkt.payload[0];
  if (type == static_cast<uint8_t>(FloodMsgType::BASE_PROBE)) {
    if (pkt.payload.size() < sizeof(FloodBaseProbeMsg)) {
      return;
    }
    FloodBaseProbeMsg msg;
    std::memcpy(&msg, pkt.payload.data(), sizeof(msg));
    HandleFloodProbe(msg);
    return;
  }

  m_dispatcher.HandlePacket(pkt);
}


void BaseStation::HandleCorePacket(const ::Packet& pkt) {
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


void BaseStation::HandleNodeInfoRequest(const NodeInfoRequestMsg& msg) {
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


void BaseStation::HandleNodeInfoResponse(const NodeInfoResponseMsg& msg) {
  (void)msg;
}


void BaseStation::HandleFloodProbe(const FloodBaseProbeMsg& msg) {
  // Reply with the same information drones include in a flood report.
  FloodReportMsg report;
  report.flood_id = msg.flood_id;
  report.initiator_id = msg.initiator_id;
  report.reporter_id = static_cast<uint8_t>(m_id);
  report.hop_to_initiator = 0;

  ::Packet out;
  out.src = static_cast<uint8_t>(m_id);
  out.dst = msg.reporter_id;
  out.payload.resize(sizeof(report));
  std::memcpy(out.payload.data(), &report, sizeof(report));
  m_comm.send(out);

  // Also send base position update so drones can refresh base state.
  if (!m_mobility) {
    return;
  }
  ::ns3::Vector pos = m_mobility->GetPosition();
  NodeInfoResponseMsg resp;
  resp.node_id = static_cast<uint8_t>(m_id);
  resp.x = static_cast<float>(pos.x);
  resp.y = static_cast<float>(pos.y);
  resp.z = static_cast<float>(pos.z);

  ::Packet info_pkt;
  info_pkt.src = static_cast<uint8_t>(m_id);
  info_pkt.dst = msg.reporter_id;
  info_pkt.payload.resize(sizeof(resp));
  std::memcpy(info_pkt.payload.data(), &resp, sizeof(resp));
  m_comm.send(info_pkt);
}


void BaseStation::FloodRequestWorker(uint16_t flood_id) {
  if (m_peers.empty()) {
    return;
  }

  std::vector<DronePeer> reachable;
  reachable.reserve(m_peers.size());
  for (const auto& peer : m_peers) {
    if (IsDroneReachable(peer.mobility)) {
      reachable.push_back(peer);
    }
  }

  if (reachable.empty()) {
    std::cerr << "[BaseStation " << m_id << "] no reachable drones for flood " << flood_id << std::endl;
    return;
  }

  const auto chosen = *std::min_element(
      reachable.begin(),
      reachable.end(),
      [this](const DronePeer& a, const DronePeer& b) {
        if (!a.mobility || !b.mobility || !m_mobility) {
          return a.id < b.id;
        }
        const ::ns3::Vector pa = a.mobility->GetPosition();
        const ::ns3::Vector pb = b.mobility->GetPosition();
        const ::ns3::Vector base = m_mobility->GetPosition();
        const double da = std::hypot(pa.x - base.x, pa.y - base.y);
        const double db = std::hypot(pb.x - base.x, pb.y - base.y);
        if (da == db) {
          return a.id < b.id;
        }
        return da < db;
      });

  FloodStartMsg msg;
  msg.flood_id = flood_id;

  ::Packet out;
  out.src = static_cast<uint8_t>(m_id);
  out.dst = chosen.id;
  out.payload.resize(sizeof(msg));
  std::memcpy(out.payload.data(), &msg, sizeof(msg));
  m_comm.send(out);
}


bool BaseStation::IsDroneReachable(::ns3::Ptr<::ns3::ConstantPositionMobilityModel> mob) const {
  if (!m_mobility || !mob) {
    return false;
  }
  const ::ns3::Vector base = m_mobility->GetPosition();
  const ::ns3::Vector other = mob->GetPosition();
  const double dx = base.x - other.x;
  const double dy = base.y - other.y;
  const double dz = base.z - other.z;
  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
  return dist <= sim::RadioEnvironment::Get().MaxRangeMeters();
}
