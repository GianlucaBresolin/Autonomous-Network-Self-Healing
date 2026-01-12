#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/udp-socket-factory.h"

#include <cstring>
#include <cstdint>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

#include "modules/flooding/flood.h"
#include "modules/flooding/flood.cpp"

namespace {

constexpr uint16_t kPort = 9999;

std::vector<uint8_t> SerializePkt(const ::Packet& pkt) {
  std::vector<uint8_t> out;
  out.reserve(2 + pkt.payload.size());
  out.push_back(pkt.src);
  out.push_back(pkt.dst);
  out.insert(out.end(), pkt.payload.begin(), pkt.payload.end());
  return out;
}

bool DeserializePkt(ns3::Ptr<ns3::Packet> p, ::Packet& out) {
  const uint32_t n = p->GetSize();
  if (n < 2) {
    return false;
  }
  std::vector<uint8_t> buf(n);
  p->CopyData(buf.data(), n);
  out.src = buf[0];
  out.dst = buf[1];
  out.payload.assign(buf.begin() + 2, buf.end());
  return true;
}

class Ns3CommunicationManagerAdapter : public CommunicationManagerInterface {
 public:
  Ns3CommunicationManagerAdapter(ns3::Ptr<ns3::Socket> socket,
                                 uint8_t selfId,
                                 std::vector<ns3::Ipv4Address> idToIp,
                                 uint16_t port)
      : socket_(socket), selfId_(selfId), idToIp_(std::move(idToIp)), port_(port) {
    socket_->SetAllowBroadcast(true);
  }

  void send(const ::Packet& pkt) override {
    const auto bytes = SerializePkt(pkt);
    ns3::Ptr<ns3::Packet> p = ns3::Create<ns3::Packet>(bytes.data(), static_cast<uint32_t>(bytes.size()));

    ns3::Ipv4Address dstIp;
    if (pkt.dst == BROADCAST_ID) {
      dstIp = ns3::Ipv4Address("255.255.255.255");
    } else if (pkt.dst < idToIp_.size()) {
      dstIp = idToIp_[pkt.dst];
    } else {
      std::cerr << "[Node " << int(selfId_) << "] invalid dst id " << int(pkt.dst) << std::endl;
      return;
    }

    ns3::InetSocketAddress dst = ns3::InetSocketAddress(dstIp, port_);
    socket_->SendTo(p, 0, dst);
  }

  void receive(::Packet& /*pkt*/) override {}

 private:
  ns3::Ptr<ns3::Socket> socket_;
  uint8_t selfId_;
  std::vector<ns3::Ipv4Address> idToIp_;
  uint16_t port_;
};

struct NodeCtx {
  uint8_t id;
  ns3::Ptr<ns3::Socket> socket;
  std::unique_ptr<Ns3CommunicationManagerAdapter> cm;
  std::unique_ptr<Flooding> flooding;
};

void SocketRx(NodeCtx* self, ns3::Ptr<ns3::Socket> sock) {
  ns3::Address from;
  ns3::Ptr<ns3::Packet> p;
  while ((p = sock->RecvFrom(from))) {
    ::Packet pkt;
    if (!DeserializePkt(p, pkt)) {
      continue;
    }

    // Drop self-originated frames to keep logs sane.
    if (pkt.src == self->id) {
      continue;
    }

    self->flooding->onPacketReceived(pkt);
  }
}

struct BaseCtx {
  uint8_t id;
  NodeCtx* cm_node;
  std::unordered_map<uint8_t, uint8_t> hop_to_base;
};

bool DecodeHopEntry(const ::Packet& pkt, FloodHopEntryMsg& msg) {
  if (pkt.payload.size() < sizeof(FloodHopEntryMsg)) {
    return false;
  }
  std::memcpy(&msg, pkt.payload.data(), sizeof(FloodHopEntryMsg));
  return msg.type == FloodMsgType::HOP_ENTRY;
}

void BaseSocketRx(BaseCtx* base, ns3::Ptr<ns3::Socket> sock) {
  ns3::Address from;
  ns3::Ptr<ns3::Packet> p;
  while ((p = sock->RecvFrom(from))) {
    ::Packet pkt;
    if (!DeserializePkt(p, pkt)) {
      continue;
    }

    // Base is off-swarm
    if (pkt.dst != base->id) {
      continue;
    }

    FloodHopEntryMsg msg;
    if (!DecodeHopEntry(pkt, msg)) {
      continue;
    }

    base->hop_to_base[msg.node_id] = msg.hop_to_base;
    std::cout << "[Base] hop_to_base(node " << int(msg.node_id) << ") = " << int(msg.hop_to_base)
              << " (initiator " << int(msg.initiator_id) << ", flood " << msg.flood_id << ")" << std::endl;
  }
}

void BaseSendStart(BaseCtx* base, uint8_t initiator_id, uint16_t flood_id) {
  FloodStartMsg start;
  start.flood_id = flood_id;

  ::Packet pkt;
  pkt.src = base->id;
  pkt.dst = initiator_id;
  pkt.payload.resize(sizeof(start));
  std::memcpy(pkt.payload.data(), &start, sizeof(start));

  std::cout << "[Base] requesting flood " << flood_id << " from initiator " << int(initiator_id) << std::endl;
  base->cm_node->cm->send(pkt);
}

void InitiatorSendHopEntry(NodeCtx* initiator, uint8_t base_id, FloodHopEntryMsg msg) {
  ::Packet pkt;
  pkt.src = initiator->id;
  pkt.dst = base_id;
  pkt.payload.resize(sizeof(msg));
  std::memcpy(pkt.payload.data(), &msg, sizeof(msg));
  initiator->cm->send(pkt);
}

void InitiatorSendHopTable(NodeCtx* initiator, uint8_t base_id, uint16_t flood_id) {
  const auto entries = initiator->flooding->getHopTableToBase(flood_id);
  std::cout << "[Initiator " << int(initiator->id) << "] sending " << entries.size() << " hop table entries to base"
            << std::endl;

  uint32_t idx = 0;
  for (const auto& kv : entries) {
    FloodHopEntryMsg msg;
    msg.flood_id = flood_id;
    msg.initiator_id = initiator->id;
    msg.node_id = kv.first;
    msg.hop_to_base = kv.second;

    ns3::Simulator::Schedule(ns3::MilliSeconds(20 * idx++),
                             ns3::MakeEvent(&InitiatorSendHopEntry, initiator, base_id, msg));
  }
}

}

int main(int argc, char* argv[]) {
 ns3::Time::SetResolution(ns3::Time::NS);

  const uint32_t nNodes = 10;
  ns3::NodeContainer nodes;
  nodes.Create(nNodes);

  ns3::WifiHelper wifi;
  wifi.SetStandard(ns3::WIFI_STANDARD_80211b);

  ns3::YansWifiPhyHelper phy;

  ns3::YansWifiChannelHelper channel;
  channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  channel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", ns3::DoubleValue(7.5));
  phy.SetChannel(channel.Create());

  ns3::WifiMacHelper mac;
  mac.SetType("ns3::AdhocWifiMac");

  ns3::NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

  ns3::MobilityHelper mobility;
  ns3::Ptr<ns3::ListPositionAllocator> pos = ns3::CreateObject<ns3::ListPositionAllocator>();

  const uint8_t initiator_id = 2;
  const double spacing_m = 6.0;

  pos->Add(ns3::Vector(0.0, 0.0, 0.0));
  pos->Add(ns3::Vector(2 * spacing_m, 0.0, 0.0));
  pos->Add(ns3::Vector(1 * spacing_m, 0.0, 0.0));
  double x = 3 * spacing_m;
  for (uint32_t i = 3; i < nNodes; ++i) {
    pos->Add(ns3::Vector(x, 0.0, 0.0));
    x += spacing_m;
  }
  mobility.SetPositionAllocator(pos);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);

  ns3::InternetStackHelper internet;
  internet.Install(nodes);

  ns3::Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  ns3::Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

  std::vector<ns3::Ipv4Address> idToIp;
  idToIp.reserve(nNodes);
  for (uint32_t i = 0; i < nNodes; ++i) {
    idToIp.push_back(ifaces.GetAddress(i));
  }

  std::vector<NodeCtx> ctx;
  ctx.resize(nNodes);
  for (uint32_t i = 0; i < nNodes; ++i) {
    ctx[i].id = static_cast<uint8_t>(i);
    ctx[i].socket = ns3::Socket::CreateSocket(nodes.Get(i), ns3::UdpSocketFactory::GetTypeId());
    ctx[i].socket->Bind(ns3::InetSocketAddress(ns3::Ipv4Address::GetAny(), kPort));
    ctx[i].socket->SetAllowBroadcast(true);

    ctx[i].cm = std::make_unique<Ns3CommunicationManagerAdapter>(ctx[i].socket, ctx[i].id, idToIp, kPort);
    if (i != 0) {
      ctx[i].flooding = std::make_unique<Flooding>(ctx[i].id, *ctx[i].cm);
    }
  }

  BaseCtx base;
  base.id = 0;
  base.cm_node = &ctx[0];

  ctx[0].socket->SetRecvCallback(ns3::MakeBoundCallback(&BaseSocketRx, &base));
  for (uint32_t i = 1; i < nNodes; ++i) {
    ctx[i].socket->SetRecvCallback(ns3::MakeBoundCallback(&SocketRx, &ctx[i]));
  }

  ns3::Simulator::Stop(ns3::Seconds(12));

  const uint16_t flood_id = 42;

  // Base requests the initiator to start the flood
  ns3::Simulator::Schedule(ns3::Seconds(1), ns3::MakeEvent(&BaseSendStart, &base, initiator_id, flood_id));

  // After the flood has time to propagate and collect reports, initiator forwards the table to base
  ns3::Simulator::Schedule(ns3::Seconds(7), ns3::MakeEvent(&InitiatorSendHopTable, &ctx[initiator_id], base.id, flood_id));

  ns3::Simulator::Run();
  ns3::Simulator::Destroy();
  return 0;
}
