#pragma once

#include <cstdint>
#include <unordered_map>

#include "interfaces/position.h"

#include "modules/communication/communication_manager.h"
#include "modules/dispatch/dispatch_manager.h"

#include "modules/messages/messages.h"

#include "ns3/core-module.h"
#include "ns3/geocentric-constant-position-mobility-model.h"
#include "ns3/internet-module.h"

#include "platform/ns3/custom_mobility/custom_mobility.h"
#include "platform/ns3/position/ns3_position.h"
#include "platform/ns3/transport/ns3_socket_transport.h"

// NS-3 bound base station node logic.
// - Static position.
// - Never broadcasts. All sends are unicast.
// - Receives PositionUpdateMsg from drones and replies with PositionAckMsg.
class Ns3BaseStation {
 public:
  Ns3BaseStation(uint8_t id, ::ns3::Ptr<::ns3::Node> node);

  uint8_t id() const { return m_id; }
  ::ns3::Ipv4Address ip() const { return m_transport_ip; }

  PositionInterface* position() const { return m_position.get(); }

  void setPosition(double x, double y, double z);

  void registerDrone(uint8_t id, ::ns3::Ipv4Address ip);

  // Starts base station periodic behaviors (currently: flood triggering).
  void start();

  // Optional: base can trigger a new flood by unicast start to an initiator drone.
  void requestFlood(uint16_t flood_id, uint8_t initiator_drone_id);

 private:
  void onTick();

  void dispatchPacket(const ::Packet& pkt);
  void handleCorePacket(const ::Packet& pkt);

  void handlePositionUpdate(const PositionUpdateMsg& msg);

  void sendPositionAck(uint8_t drone_id, uint16_t seq);

  uint8_t m_id;
  ::ns3::Ptr<::ns3::Node> m_node;

  ::ns3::Ipv4Address m_transport_ip;

  std::unique_ptr<CustomMobility> m_custom_mobility;
  std::unique_ptr<Ns3Position> m_position;

  CommunicationManager m_comm;
  DispatchManager m_dispatcher;

  std::unordered_map<uint8_t, ::ns3::Ipv4Address> m_drone_ips;
  std::unordered_map<uint8_t, PositionUpdateMsg> m_last_position;

  double m_tick_dt_s = 2.0;
  uint16_t m_flood_seq = 0;
};
