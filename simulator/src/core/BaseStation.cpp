#include "core/BaseStation.h"
#include "ns3/core-module.h"
#include "ns3/constant-position-mobility-model.h"
using namespace ns3;


BaseStation::BaseStation(uint32_t id, Ptr<Node> node) : m_id(id), m_node(node) {
  if (!m_node) {
    return;
  }

  m_mobility = m_node->GetObject<ConstantPositionMobilityModel>();
  if (!m_mobility) {
    m_mobility = CreateObject<ConstantPositionMobilityModel>();
    m_node->AggregateObject(m_mobility);
  }

  m_radio = sim::RadioEnvironment::Get().Install(m_node);
  m_ip = m_radio.ip;

  if (m_radio.socket) {
    m_radio.socket->SetRecvCallback(MakeCallback(&BaseStation::SocketRx, this));
  }
}


void BaseStation::Start() {
  Simulator::Schedule(Seconds(0.0), MakeCallback(&BaseStation::OnTick, this));
}


void BaseStation::OnTick() {
  if (m_mobility) {
    Vector pos = m_mobility->GetPosition();
    m_mobility->SetPosition(pos);
  }
  Simulator::Schedule(Seconds(m_dt), MakeCallback(&BaseStation::OnTick, this));
}


void BaseStation::SocketRx(Ptr<Socket> sock) {
  Address from;
  Ptr<Packet> p;
  while ((p = sock->RecvFrom(from))) {
    if (InetSocketAddress::IsMatchingType(from)) {
      InetSocketAddress src = InetSocketAddress::ConvertFrom(from);
      std::cout << "[BaseStation " << m_id << "] RX " << p->GetSize() << "B from " << src.GetIpv4() << ":" << src.GetPort()
                << std::endl;
    }
    OnReceive(p);
  }
}


void BaseStation::SendTo(Ipv4Address dst, Ptr<Packet> p) {
  if (!m_radio.socket || !p) {
    return;
  }
  InetSocketAddress addr(dst, sim::RadioEnvironment::Get().Port());
  m_radio.socket->SendTo(p, 0, addr);
}


void BaseStation::SendBroadcast(Ptr<Packet> p) {
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


void BaseStation::OnReceive(Ptr<Packet> p) {}