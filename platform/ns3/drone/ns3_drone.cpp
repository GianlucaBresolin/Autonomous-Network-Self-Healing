#include "platform/ns3/drone/ns3_drone.h"

Ns3Drone::Ns3Drone(uint8_t id, ::ns3::Ptr<::ns3::Node> node)
    : m_id(id),
      m_node(node),
      m_comm(std::make_unique<sim::Ns3SocketTransport>(node), id),
      m_controller(id, DEFAULT_K_ATT, DEFAULT_K_REP, DEFAULT_D_SAFE, DEFAULT_V_MAX) {
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
  m_velocity_actuator = std::make_unique<Ns3VelocityActuator>(m_custom_mobility.get());

  m_comm.setReceiveHandler([this](const ::Packet& pkt) { dispatchPacket(pkt); });

  m_flood_manager = std::make_unique<FloodManager>(m_id, m_comm, [this]() { return isBaseReachable(); });
  m_neighbor_manager = std::make_unique<NeighborManager>(&m_comm);

  m_dispatcher.setFloodManager(m_flood_manager.get());
  m_dispatcher.setNeighborManager(m_neighbor_manager.get());
  m_dispatcher.setFallbackHandler([this](const ::Packet& pkt) { handleCorePacket(pkt); });

  m_last_ack_rx_s = ::ns3::Simulator::Now().GetSeconds();

  // Stagger periodic behavior to avoid all drones transmitting at the same instant.
  // This reduces Wi-Fi contention and makes ACK timeouts correlate with real disconnection.
  m_tick_phase_s = 0.05 * static_cast<double>(m_id);
}

void Ns3Drone::setBaseStation(uint8_t base_id, ::ns3::Ipv4Address base_ip, PositionInterface* base_position) {
  m_base_id = base_id;
  m_base_ip = base_ip;
  m_base_position = base_position;
  m_has_base = true;

  m_last_ack_rx_s = ::ns3::Simulator::Now().GetSeconds();

  m_comm.registerPeer(base_id, base_ip.Get());
  if (m_flood_manager) {
    m_flood_manager->setBaseId(base_id);
  }
}

void Ns3Drone::start() {
  ::ns3::Simulator::Schedule(::ns3::Seconds(m_tick_phase_s), ::ns3::MakeCallback(&Ns3Drone::onTick, this));
}

void Ns3Drone::startMission() {
  if (m_mission_active) {
    return;
  }
  if (!m_flood_manager || !m_velocity_actuator || !m_neighbor_manager || !m_position) {
    return;
  }

  m_mission_active = true;
  m_controller.setMissionActive(true);

  m_mission_start_s = ::ns3::Simulator::Now().GetSeconds();
  m_last_mission_log_s = -1.0;

  std::cout << "[Mission] t=" << m_mission_start_s << "s drone=" << static_cast<int>(m_id)
            << " mission_active=1" << std::endl;
}

void Ns3Drone::stopMission() {
  m_mission_active = false;
  m_controller.setMissionActive(false);
}

void Ns3Drone::onTick() {
  // Always send periodic updates so reachability (ACK-based) is meaningful.
  // HELP_PROXY is only emitted when we are not in mission.
  sendPositionUpdate();

  const double now_s = ::ns3::Simulator::Now().GetSeconds();
  if (!m_mission_active && m_waiting_ack && (now_s - m_last_ack_rx_s) > m_ack_timeout_s) {
    sendHelpProxy();
    m_waiting_ack = false;
  }

  // Post-mission debug: log how drones reposition for a short window after mission activation.
  if (m_mission_active && m_mission_start_s >= 0.0 && (now_s - m_mission_start_s) <= m_mission_log_window_s) {
    if (m_last_mission_log_s < 0.0 || (now_s - m_last_mission_log_s) >= m_mission_log_dt_s) {
      if (m_position) {
        m_position->retrieveCurrentPosition();
        const auto coords = m_position->getCoordinates();
        const uint8_t hops = m_flood_manager ? m_flood_manager->getHopsFromBase() : UINT8_MAX;
        const size_t n_neighbors = m_neighbor_manager ? m_neighbor_manager->getNeighbors().size() : 0;
        std::cout << "[Reposition] t=" << now_s << "s drone=" << static_cast<int>(m_id)
                  << " hops=" << static_cast<int>(hops)
                  << " neighbors=" << n_neighbors
                  << " pos=(" << (coords.size() > 0 ? coords[0] : 0.0)
                  << "," << (coords.size() > 1 ? coords[1] : 0.0)
                  << "," << (coords.size() > 2 ? coords[2] : 0.0) << ")";

        if (m_last_help_proxy_rx_s >= 0.0 && (now_s - m_last_help_proxy_rx_s) <= m_mission_log_window_s) {
          std::cout << " after_HELP_PROXY_rx(t=" << m_last_help_proxy_rx_s << "s)";
        }
        if (m_last_help_proxy_tx_s >= 0.0 && (now_s - m_last_help_proxy_tx_s) <= m_mission_log_window_s) {
          std::cout << " after_HELP_PROXY_tx(t=" << m_last_help_proxy_tx_s << "s)";
        }
        std::cout << std::endl;
      }
      m_last_mission_log_s = now_s;
    }
  }

  // Drive motion in lockstep with simulation time.
  // - If mission is active: one potential-field iteration per tick.
  // - If mission is off: apply a default "idle" velocity so drones move and can leave coverage.
  if (m_flood_manager && m_velocity_actuator && m_neighbor_manager && m_position) {
    m_controller.step(m_flood_manager.get(), m_velocity_actuator.get(), m_neighbor_manager.get(), m_position.get());
  }

  // Idle trace: confirm motion/coverage exit for debugging (only drone 4).
  if (!m_mission_active && m_id == 4 && m_position) {
    if (m_last_idle_log_s < 0.0 || (now_s - m_last_idle_log_s) >= m_idle_log_dt_s) {
      m_position->retrieveCurrentPosition();
      const auto coords = m_position->getCoordinates();
      std::cout << "[Idle] t=" << now_s << "s drone=4 pos=(" << (coords.size() > 0 ? coords[0] : 0.0)
                << "," << (coords.size() > 1 ? coords[1] : 0.0) << "," << (coords.size() > 2 ? coords[2] : 0.0)
                << ")";
      if (m_base_position) {
        const Vector3D diff = m_position->distanceFrom(m_base_position);
        std::cout << " dist_to_base=" << diff.module();
      }
      std::cout << std::endl;
      m_last_idle_log_s = now_s;
    }
  }

  ::ns3::Simulator::Schedule(::ns3::Seconds(m_tick_dt_s), ::ns3::MakeCallback(&Ns3Drone::onTick, this));
}

void Ns3Drone::dispatchPacket(const ::Packet& pkt) {
  if (pkt.payload.empty()) {
    return;
  }

  if (pkt.dst != m_id && pkt.dst != BROADCAST_ID) {
    return;
  }

  m_dispatcher.handlePacket(pkt);
}

void Ns3Drone::handleCorePacket(const ::Packet& pkt) {
  if (pkt.payload.size() < 1) {
    return;
  }

  const auto type = static_cast<SimMsgType>(pkt.payload[0]);
  switch (type) {
    case SimMsgType::POS_ACK: {
      if (pkt.payload.size() < sizeof(PositionAckMsg)) {
        return;
      }
      PositionAckMsg ack;
      std::memcpy(&ack, pkt.payload.data(), sizeof(ack));

      if (ack.base_id != m_base_id || ack.drone_id != m_id) {
        return;
      }

      m_last_ack_rx_s = ::ns3::Simulator::Now().GetSeconds();
      m_last_acked_seq = ack.seq;
      m_waiting_ack = false;

      std::cout << "[ACK RX] t=" << m_last_ack_rx_s << "s drone=" << static_cast<int>(m_id)
                << " seq=" << ack.seq << std::endl;

      // Treat the base station as a regular neighbor entry.
      // We translate the ACK's embedded base info into the same payload format used
      // by NeighborManager broadcasts: [id][hops][double coords...].
      if (m_neighbor_manager) {
        ::Packet base_as_neighbor;
        base_as_neighbor.type = ::PacketType::NEIGHBOR;
        base_as_neighbor.src = ack.base_id;
        base_as_neighbor.dst = m_id;

        base_as_neighbor.payload.resize(2 + 3 * sizeof(double));
        base_as_neighbor.payload[0] = ack.base_id;
        base_as_neighbor.payload[1] = ack.base_hops_to_base_station;

        const double base_coords[3] = {ack.x, ack.y, ack.z};
        std::memcpy(base_as_neighbor.payload.data() + 2, base_coords, sizeof(base_coords));

        m_neighbor_manager->onPacketReceived(base_as_neighbor);
      }
      return;
    }

    case SimMsgType::POS_UPDATE:
    case SimMsgType::HELP_PROXY: {
      if (pkt.payload.size() < sizeof(HelpProxyMsg)) {
        return;
      }
      HelpProxyMsg msg;
      std::memcpy(&msg, pkt.payload.data(), sizeof(msg));

      // Only react to HELP_PROXY requests that target our base station.
      if (msg.base_id != m_base_id) {
        return;
      }

      m_last_help_proxy_rx_s = ::ns3::Simulator::Now().GetSeconds();
      if (m_position) {
        m_position->retrieveCurrentPosition();
        const auto coords = m_position->getCoordinates();
        std::cout << "[HELP_PROXY RX] t=" << m_last_help_proxy_rx_s << "s drone=" << static_cast<int>(m_id)
                  << " requester=" << static_cast<int>(msg.requester_id)
                  << " pos=(" << (coords.size() > 0 ? coords[0] : 0.0)
                  << "," << (coords.size() > 1 ? coords[1] : 0.0)
                  << "," << (coords.size() > 2 ? coords[2] : 0.0) << ")" << std::endl;
      }

      // Enter mission mode to reposition the swarm.
      startMission();
      return;
    }
    default:
      return;
  }
}

void Ns3Drone::sendPositionUpdate() {
  if (!m_position || !m_has_base) {
    return;
  }

  const double now_s = ::ns3::Simulator::Now().GetSeconds();
  if ((now_s - m_last_pos_send_s) < m_tick_dt_s) {
    return;
  }

  m_position->retrieveCurrentPosition();
  const auto coords = m_position->getCoordinates();

  PositionUpdateMsg msg;
  msg.drone_id = m_id;
  msg.base_id = m_base_id;
  msg.seq = ++m_pos_seq;
  msg.x = coords.size() > 0 ? static_cast<float>(coords[0]) : 0.0f;
  msg.y = coords.size() > 1 ? static_cast<float>(coords[1]) : 0.0f;
  msg.z = coords.size() > 2 ? static_cast<float>(coords[2]) : 0.0f;

  ::Packet out;
  out.type = ::PacketType::CORE;
  out.src = m_id;
  out.dst = m_base_id;
  out.payload.resize(sizeof(msg));
  std::memcpy(out.payload.data(), &msg, sizeof(msg));

  m_comm.send(out);
  m_last_pos_send_s = now_s;
  m_waiting_ack = true;
}

void Ns3Drone::sendHelpProxy() {
  m_last_help_proxy_tx_s = ::ns3::Simulator::Now().GetSeconds();

  if (m_position) {
    m_position->retrieveCurrentPosition();
    const auto coords = m_position->getCoordinates();
    std::cout << "[HELP_PROXY TX] t=" << m_last_help_proxy_tx_s << "s drone=" << static_cast<int>(m_id)
              << " reason=ACK_TIMEOUT"
              << " last_ack=" << m_last_ack_rx_s << "s"
              << " pos=(" << (coords.size() > 0 ? coords[0] : 0.0)
              << "," << (coords.size() > 1 ? coords[1] : 0.0)
              << "," << (coords.size() > 2 ? coords[2] : 0.0) << ")" << std::endl;
  } else {
    std::cout << "[HELP_PROXY TX] t=" << m_last_help_proxy_tx_s << "s drone=" << static_cast<int>(m_id)
              << " reason=ACK_TIMEOUT" << std::endl;
  }

  HelpProxyMsg help;
  help.requester_id = m_id;
  help.base_id = m_base_id;

  ::Packet out;
  out.type = ::PacketType::CORE;
  out.src = m_id;
  out.dst = BROADCAST_ID;
  out.payload.resize(sizeof(help));
  std::memcpy(out.payload.data(), &help, sizeof(help));

  m_comm.send(out);

  // HELP_PROXY is the trigger to start the mission behavior.
  // startMission();
}

bool Ns3Drone::isBaseReachable() const {
  if (!m_has_base) {
    return false;
  }

  const double now_s = ::ns3::Simulator::Now().GetSeconds();
  return (now_s - m_last_ack_rx_s) <= m_ack_timeout_s;
}
