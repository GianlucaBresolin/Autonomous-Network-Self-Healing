#include "platform/ns3/base_station/ns3_base_station.h"

Ns3BaseStation::Ns3BaseStation(uint8_t id, ::ns3::Ptr<::ns3::Node> node)
    : m_id(id),
      m_node(node),
      m_comm(std::make_unique<sim::Ns3SocketTransport>(node), id) {
  if (!m_node) {
    return;
  }

  m_transport_ip = sim::RadioEnvironment::Get().Install(m_node).ip;

  auto mobility = m_node->GetObject<::ns3::GeocentricConstantPositionMobilityModel>();
  if (!mobility) {
    mobility = ::ns3::CreateObject<::ns3::GeocentricConstantPositionMobilityModel>();
    m_node->AggregateObject(mobility);
  }

  m_custom_mobility = std::make_unique<CustomMobility>(mobility);
  m_position = std::make_unique<Ns3Position>(m_custom_mobility.get());

  m_comm.setReceiveHandler([this](const ::Packet& pkt) { dispatchPacket(pkt); });
  m_dispatcher.setFallbackHandler([this](const ::Packet& pkt) { handleCorePacket(pkt); });
}

void Ns3BaseStation::start() {
  ::ns3::Simulator::Schedule(::ns3::Seconds(0.5), ::ns3::MakeCallback(&Ns3BaseStation::onTick, this));
}

void Ns3BaseStation::onTick() {
  if (!m_drone_ips.empty()) {
    // Choose a stable initiator: the lowest registered drone id.
    uint8_t initiator = 0;
    for (const auto& kv : m_drone_ips) {
      initiator = (initiator == 0) ? kv.first : static_cast<uint8_t>(std::min<int>(initiator, kv.first));
    }
    if (initiator != 0) {
      requestFlood(++m_flood_seq, initiator);
    }
  }

  ::ns3::Simulator::Schedule(::ns3::Seconds(m_tick_dt_s), ::ns3::MakeCallback(&Ns3BaseStation::onTick, this));
}

void Ns3BaseStation::setPosition(double x, double y, double z) {
  if (!m_custom_mobility) {
    return;
  }
  m_custom_mobility->setPosition(x, y, z);
  if (m_position) {
    m_position->retrieveCurrentPosition();
  }
}

void Ns3BaseStation::registerDrone(uint8_t id, ::ns3::Ipv4Address ip) {
  m_drone_ips[id] = ip;
  m_comm.registerPeer(id, ip.Get());
}

void Ns3BaseStation::requestFlood(uint16_t flood_id, uint8_t initiator_drone_id) {
  FloodStartMsg msg;
  msg.flood_id = flood_id;

  ::Packet out;
  out.type = ::PacketType::FLOOD;
  out.src = m_id;
  out.dst = initiator_drone_id;
  out.payload.resize(sizeof(msg));
  std::memcpy(out.payload.data(), &msg, sizeof(msg));

  // Base station never broadcasts.
  m_comm.send(out);
}

void Ns3BaseStation::dispatchPacket(const ::Packet& pkt) {
  if (pkt.payload.empty()) {
    return;
  }

  if (pkt.dst != m_id && pkt.dst != BROADCAST_ID) {
    return;
  }

  m_dispatcher.handlePacket(pkt);
}

void Ns3BaseStation::handleCorePacket(const ::Packet& pkt) {
  if (pkt.payload.size() < 1) {
    return;
  }

  const auto type = static_cast<SimMsgType>(pkt.payload[0]);
  switch (type) {
    case SimMsgType::POS_UPDATE: {
      if (pkt.payload.size() < sizeof(PositionUpdateMsg)) {
        return;
      }
      PositionUpdateMsg msg;
      std::memcpy(&msg, pkt.payload.data(), sizeof(msg));
      handlePositionUpdate(msg);
      return;
    }

    case SimMsgType::HELP_PROXY:
    case SimMsgType::POS_ACK:
    default:
      return;
  }
}

void Ns3BaseStation::handlePositionUpdate(const PositionUpdateMsg& msg) {
  if (msg.base_id != m_id) {
    return;
  }

  // Track last seen position.
  m_last_position[msg.drone_id] = msg;

  sendPositionAck(msg.drone_id, msg.seq);
}

void Ns3BaseStation::sendPositionAck(uint8_t drone_id, uint16_t seq) {
  if (m_position) {
    m_position->retrieveCurrentPosition();
  }
  const auto coords = m_position ? m_position->getCoordinates() : std::vector<double>{};

  PositionAckMsg ack;
  ack.base_id = m_id;
  ack.drone_id = drone_id;
  ack.seq = seq;
  ack.base_hops_to_base_station = 0;
  ack.x = coords.size() > 0 ? coords[0] : 0.0;
  ack.y = coords.size() > 1 ? coords[1] : 0.0;
  ack.z = coords.size() > 2 ? coords[2] : 0.0;

  ::Packet out;
  out.type = ::PacketType::CORE;
  out.src = m_id;
  out.dst = drone_id;
  out.payload.resize(sizeof(ack));
  std::memcpy(out.payload.data(), &ack, sizeof(ack));

  // Unicast only.
  m_comm.send(out);
}
