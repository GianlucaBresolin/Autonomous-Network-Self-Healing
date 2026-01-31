#pragma once

#include <cstdint>
#include <memory>
#include <cstring>
#include <iostream>
#include <fstream>

#include "interfaces/position.h"

#include "modules/communication/communication_manager.h"
#include "modules/controller/controller.h"
#include "modules/dispatch/dispatch_manager.h"
#include "modules/flood/flood_manager.h"
#include "modules/neighbor/neighbor_manager.h"

#include "common/messages.h"
#include "common/packet.h"

#include "ns3/core-module.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/internet-module.h"

#include "platform/ns3/custom_mobility/custom_mobility.h"
#include "platform/ns3/position/ns3_position.h"
#include "platform/ns3/transport/ns3_socket_transport.h"
#include "platform/ns3/velocity_actuator/ns3_velocity_actuator.h"
#include "platform/ns3/radio_environment/radio_environment.h"

// NS-3 bound drone node logic.
// - While not in mission: periodically unicast PositionUpdateMsg to base and wait for PositionAckMsg.
// - If ACK is missing for too long: broadcast HelpProxyMsg.
// - When mission starts: Controller drives motion and NeighborManager broadcasts to neighbors.
class Ns3Drone {
 public:
  Ns3Drone(
    uint8_t id,
    ::ns3::Ptr<::ns3::Node> node,
    float k_att = 1.5f,
    float k_rep = 5.0f,
    float d_safe = 1.0f,
    float v_max = 2.5f,
    float drone_weight_kg = 0.029f
  );

  uint8_t id() const { return m_id; }
  ::ns3::Ipv4Address ip() const { return m_transport_ip; }

  PositionInterface* position() const { return m_position.get(); }

  void setBaseStation(uint8_t base_id, ::ns3::Ipv4Address base_ip, PositionInterface* base_position = nullptr);

  void startMission();
  void stopMission();

  void start();

  void setRepositionLogger(const std::shared_ptr<std::ofstream>& csv);

 private:
  void onTick();
  void dispatchPacket(const ::Packet& pkt);
  void handleCorePacket(const ::Packet& pkt);

  void sendPositionUpdate();
  void sendHelpProxy();

  bool isBaseReachable() const;

  uint8_t m_id;
  ::ns3::Ptr<::ns3::Node> m_node;

  ::ns3::Ipv4Address m_transport_ip;

  std::unique_ptr<CustomMobility> m_custom_mobility;
  std::unique_ptr<Ns3Position> m_position;
  std::unique_ptr<Ns3VelocityActuator> m_velocity_actuator;

  uint8_t m_base_id = 0;
  ::ns3::Ipv4Address m_base_ip;
  PositionInterface* m_base_position = nullptr;
  bool m_has_base = false;

  CommunicationManager m_comm;

  std::unique_ptr<FloodManager> m_flood_manager;
  std::unique_ptr<NeighborManager> m_neighbor_manager;
  DispatchManager m_dispatcher;

  Controller m_controller;

  bool help_proxy_sent = false;

  // Debug logging for mission transitions / post-HELP_PROXY repositioning.
  double m_last_help_proxy_tx_s = -1.0;
  double m_last_help_proxy_rx_s = -1.0;

  double m_mission_start_s = -1.0;
  double m_last_mission_log_s = -1.0;
  double m_mission_log_dt_s = 0.5;

  double m_last_idle_log_s = -1.0;
  double m_idle_log_dt_s = 2.0;

  std::shared_ptr<std::ofstream> m_reposition_csv;

  // Heartbeat/ack tracking (reachability is based on receiving ACKs)
  double m_tick_dt_s = 0.05;
  
  double m_tick_phase_s = 0.0;
  double m_ack_timeout_s = 1.5;
  uint16_t m_pos_seq = 0;
  uint16_t m_last_acked_seq = 0;
  double m_last_pos_send_s = 0.0;
  double m_last_ack_rx_s = 0.0;
  bool m_waiting_ack = false;
};
