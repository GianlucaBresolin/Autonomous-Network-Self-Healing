#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <unordered_map>

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/propagation-module.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/wifi-module.h"

namespace sim {

struct RadioEnvironmentConfig {
  // Simple “coverage area” model: beyond this distance, frames are not received.
  double maxRangeMeters = 30.0;

  // IPv4 subnet used for all nodes that install this radio.
  std::string networkBase = "10.1.1.0";
  std::string networkMask = "255.255.255.0";

  // UDP port used for both unicast and broadcast.
  uint16_t port = 9999;
};

struct RadioEndpoint {
  ns3::Ptr<ns3::NetDevice> device;
  ns3::Ipv4Address ip;
  ns3::Ptr<ns3::Socket> socket;
};

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
  RadioEndpoint Install(ns3::Ptr<ns3::Node> node);

  uint16_t Port() const { return m_cfg.port; }
  ns3::Ipv4Address BroadcastAddress() const { return m_broadcast; }
  std::vector<ns3::Ipv4Address> AllIps() const;

 private:
  RadioEnvironment();
  void InitIfNeeded();

  RadioEnvironmentConfig m_cfg;
  bool m_initialized = false;

  ns3::WifiHelper m_wifi;
  ns3::YansWifiPhyHelper m_phy;
  ns3::WifiMacHelper m_mac;
  ns3::InternetStackHelper m_internet;
  ns3::Ipv4AddressHelper m_ipv4;
  ns3::Ipv4Address m_broadcast;

  std::unordered_map<uint32_t, RadioEndpoint> m_endpoints;
};

}  // namespace sim
