#pragma once
#include <cstdint>
#include <memory>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

#include "modules/core/messages.h"
#include "modules/dispatch/dispatch_manager.h"
#include "modules/flood/flood.h"
#include "platform/ns3/radio_environment.h"
#include "platform/ns3/communication_manager.h"

class Drone {
  public:
    Drone(uint32_t id, ns3::Ptr<ns3::Node> node);
    void Start();
    void OnTick();
    void OnReceive(ns3::Ptr<ns3::Packet> p);

    // UDP helpers. Unicast is used for base station <-> drone,
    // broadcast is used for drone <-> swarm.
    void SendTo(ns3::Ipv4Address dst, ns3::Ptr<ns3::Packet> p);
    void SendBroadcast(ns3::Ptr<ns3::Packet> p);

    ns3::Ipv4Address Ip() const { return m_ip; }
    uint32_t Id() const { return m_id; }

    uint8_t HopsFromBase() const {
      return m_flooding ? m_flooding->getHopsFromBase() : UINT8_MAX;
    }

    void SetBaseStation(uint8_t base_id, ns3::Ipv4Address base_ip);
  private:
    void SocketRx(ns3::Ptr<ns3::Socket> sock);
    void DispatchPacket(const ::Packet& pkt);
    void HandleCorePacket(const ::Packet& pkt);
    void HandleNodeInfoRequest(const NodeInfoRequestMsg& msg);
    void HandleNodeInfoResponse(const NodeInfoResponseMsg& msg);
    void MaybeProbeBase();
    bool IsBaseReachable() const;

    uint32_t m_id;
    ns3::Ptr<ns3::Node> m_node;
    ns3::Ptr<ns3::ConstantPositionMobilityModel> m_mobility;

    sim::RadioEndpoint m_radio;
    ns3::Ipv4Address m_ip;
    double m_dt = 0.05;

    uint8_t m_base_id = 0;
    ns3::Ipv4Address m_base_ip;
    double m_probe_interval = 0.5;
    double m_base_reply_timeout = 1.0;
    double m_last_probe_time = -1.0;
    double m_last_base_reply_time = -1.0;
    bool m_has_base_position = false;
    ns3::Vector m_last_base_position{0.0, 0.0, 0.0};

    sim::CommunicationManager m_comm;
    std::unique_ptr<Flooding> m_flooding;
    DispatchManager m_dispatcher;
};
