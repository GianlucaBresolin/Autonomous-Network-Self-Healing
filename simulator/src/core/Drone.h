#pragma once
#include <cstdint>
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

#include "comm/RadioEnvironment.h"


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
  private:
    void SocketRx(ns3::Ptr<ns3::Socket> sock);

    uint32_t m_id;
    ns3::Ptr<ns3::Node> m_node;
    ns3::Ptr<ns3::ConstantPositionMobilityModel> m_mobility;

    sim::RadioEndpoint m_radio;
    ns3::Ipv4Address m_ip;
    double m_dt = 0.05;
};