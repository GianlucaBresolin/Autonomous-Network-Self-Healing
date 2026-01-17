#include "platform/ns3/radio_environment.h"

#include <utility>

#include "ns3/traffic-control-helper.h"

namespace sim {

RadioEnvironment& RadioEnvironment::Get() {
  static RadioEnvironment env;
  return env;
}

RadioEnvironment::RadioEnvironment() = default;

void RadioEnvironment::Configure(const RadioEnvironmentConfig& cfg) {
  // Configuration must happen before the first install to avoid mixing setups.
  if (m_initialized) {
    return;
  }
  m_cfg = cfg;
}

void RadioEnvironment::InitIfNeeded() {
  if (m_initialized) {
    return;
  }

  // Wi-Fi ad-hoc approximates a simple swarm radio reasonably well.
  // Using 802.11b as a conservative baseline (robust / low data rate).
    m_wifi.SetStandard(::ns3::WIFI_STANDARD_80211b);
  m_wifi.SetRemoteStationManager(
      "ns3::ConstantRateWifiManager",
      "DataMode",
      ::ns3::StringValue("DsssRate1Mbps"),
      "ControlMode",
      ::ns3::StringValue("DsssRate1Mbps"));

    ::ns3::YansWifiChannelHelper channel;
  channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  channel.AddPropagationLoss(
      "ns3::RangePropagationLossModel",
      "MaxRange",
      ::ns3::DoubleValue(m_cfg.maxRangeMeters));

  m_phy.SetChannel(channel.Create());

  m_mac.SetType("ns3::AdhocWifiMac");

  m_ipv4.SetBase(m_cfg.networkBase.c_str(), m_cfg.networkMask.c_str());

  // Compute directed broadcast (e.g. 10.1.1.255 for /24).
  ::ns3::Ipv4Address base(m_cfg.networkBase.c_str());
  ::ns3::Ipv4Mask mask(m_cfg.networkMask.c_str());
  m_broadcast = ::ns3::Ipv4Address(base.Get() | ~mask.Get());

  m_initialized = true;
}

RadioEndpoint RadioEnvironment::Install(::ns3::Ptr<::ns3::Node> node) {
  InitIfNeeded();

  RadioEndpoint empty;
  if (!node) {
    return empty;
  }

  const uint32_t nodeId = node->GetId();
  const auto existing = m_endpoints.find(nodeId);
  if (existing != m_endpoints.end()) {
    return existing->second;
  }

  // Ensure the Internet stack is present.
  if (!node->GetObject<::ns3::Ipv4>()) {
    m_internet.Install(node);
  }

  ::ns3::NodeContainer one;
  one.Add(node);

  ::ns3::NetDeviceContainer devices = m_wifi.Install(m_phy, m_mac, one);
  ::ns3::Ptr<::ns3::NetDevice> dev = devices.Get(0);

  ::ns3::TrafficControlHelper tch;
  tch.SetRootQueueDisc("ns3::PfifoFastQueueDisc");
  tch.Install(devices);

  ::ns3::Ipv4InterfaceContainer ifaces = m_ipv4.Assign(devices);
  ::ns3::Ipv4Address ip = ifaces.GetAddress(0);

  ::ns3::Ptr<::ns3::Socket> sock = ::ns3::Socket::CreateSocket(node, ::ns3::UdpSocketFactory::GetTypeId());
  sock->Bind(::ns3::InetSocketAddress(::ns3::Ipv4Address::GetAny(), m_cfg.port));
  sock->SetAllowBroadcast(true);

  RadioEndpoint ep;
  ep.device = dev;
  ep.ip = ip;
  ep.socket = sock;

  m_endpoints.emplace(nodeId, ep);
  return ep;
}

std::vector<::ns3::Ipv4Address> RadioEnvironment::AllIps() const {
  std::vector<::ns3::Ipv4Address> out;
  out.reserve(m_endpoints.size());
  for (const auto& kv : m_endpoints) {
    out.push_back(kv.second.ip);
  }
  return out;
}

}  // namespace sim
