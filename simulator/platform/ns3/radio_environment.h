#pragma once

#include <unordered_map>
#include <vector>

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/propagation-module.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/wifi-helper.h"
#include "ns3/wifi-mac-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/yans-wifi-channel.h"

#include "platform/ns3/radio_endpoint.h"
#include "platform/ns3/radio_environment_config.h"

namespace sim {

// Shared radio+IP environment for a swarm.
//
// Intent:
// - approximate Crazyflie-like short-range 2.4GHz radio behavior
// - provide both unicast (base station <-> drone) and broadcast (drone <-> swarm)
// - enforce a strict coverage area using RangePropagationLossModel
class RadioEnvironment {
 public:
  static RadioEnvironment& Get();

  // Call once (optional). If not called, defaults are used.
  // Must be called before installing the first node to take effect.
  void Configure(const RadioEnvironmentConfig& cfg);

  // Installs Wi-Fi ad-hoc + Internet stack + IPv4 address + UDP socket on `node`.
  // Safe to call multiple times for the same node (returns the cached endpoint).
  RadioEndpoint Install(::ns3::Ptr<::ns3::Node> node);

  uint16_t Port() const { return m_cfg.port; }
  double MaxRangeMeters() const { return m_cfg.maxRangeMeters; }
  ::ns3::Ipv4Address BroadcastAddress() const { return m_broadcast; }
  std::vector<::ns3::Ipv4Address> AllIps() const;

  // Best-effort lookup of a node by its assigned IP (returns nullptr if unknown).
  ::ns3::Ptr<::ns3::Node> FindNodeByIp(::ns3::Ipv4Address ip) const;

 private:
  RadioEnvironment();
  void InitIfNeeded();

  RadioEnvironmentConfig m_cfg;
  bool m_initialized = false;

  ::ns3::WifiHelper m_wifi;
  ::ns3::YansWifiPhyHelper m_phy;
  ::ns3::WifiMacHelper m_mac;
  ::ns3::InternetStackHelper m_internet;
  ::ns3::Ipv4AddressHelper m_ipv4;
  ::ns3::Ipv4Address m_broadcast;

  std::unordered_map<uint32_t, RadioEndpoint> m_endpoints;
};

}  // namespace sim
