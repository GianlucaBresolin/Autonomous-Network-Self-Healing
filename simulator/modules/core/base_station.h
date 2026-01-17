#pragma once
#include <cstdint>
#include <thread>
#include <vector>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

#include "modules/core/messages.h"
#include "modules/dispatch/dispatch_manager.h"
#include "modules/flood/messages.h"
#include "platform/ns3/radio_environment.h"
#include "platform/ns3/communication_manager.h"

class BaseStation {
  public:
    BaseStation(uint32_t id, ns3::Ptr<ns3::Node> node);
    void Start();
    void OnTick();
    void OnReceive(ns3::Ptr<ns3::Packet> p);

    void SendTo(ns3::Ipv4Address dst, ns3::Ptr<ns3::Packet> p);
    void SendBroadcast(ns3::Ptr<ns3::Packet> p);

    ns3::Ipv4Address Ip() const { return m_ip; }

    void RegisterDrone(uint8_t id, ns3::Ipv4Address ip, ns3::Ptr<ns3::ConstantPositionMobilityModel> mobility);
    void RequestFlood(uint16_t flood_id);
    void RequestNodeInfo(uint8_t drone_id);
  private:
    void SocketRx(ns3::Ptr<ns3::Socket> sock);
    void DispatchPacket(const ::Packet& pkt);
    void HandleCorePacket(const ::Packet& pkt);
    void HandleNodeInfoRequest(const NodeInfoRequestMsg& msg);
    void HandleNodeInfoResponse(const NodeInfoResponseMsg& msg);
    void HandleFloodProbe(const FloodBaseProbeMsg& msg);
    void FloodRequestWorker(uint16_t flood_id);
    bool IsDroneReachable(ns3::Ptr<ns3::ConstantPositionMobilityModel> mob) const;

    struct DronePeer {
      uint8_t id;
      ns3::Ipv4Address ip;
      ns3::Ptr<ns3::ConstantPositionMobilityModel> mobility;
    };

    uint32_t m_id;
    ns3::Ptr<ns3::Node> m_node;
    ns3::Ptr<ns3::ConstantPositionMobilityModel> m_mobility;

    sim::RadioEndpoint m_radio;
    ns3::Ipv4Address m_ip;
    double m_dt = 0.05;

    sim::CommunicationManager m_comm;
    std::vector<DronePeer> m_peers;
    DispatchManager m_dispatcher;
};
