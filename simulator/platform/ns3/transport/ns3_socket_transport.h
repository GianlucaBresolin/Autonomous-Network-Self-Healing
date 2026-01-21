#pragma once

#include <cstdint>
#include <unordered_map>

#include "ns3/network-module.h"

#include "modules/communication/transport.h"
#include "platform/ns3/radio_endpoint.h"

namespace sim {

// NS-3 UDP socket transport.
// Owns socket recv callback and implements unicast + broadcast fan-out.
class Ns3SocketTransport final : public ::Transport {
 public:
  explicit Ns3SocketTransport(::ns3::Ptr<::ns3::Node> node);

  void RegisterPeer(uint8_t id, uint32_t address) override;
  void SendUnicast(uint8_t dst_id, const Bytes& bytes) override;
  void SendBroadcast(const Bytes& bytes) override;
  void SetRxCallback(RxCallback cb) override;

  ::ns3::Ipv4Address SelfIp() const { return m_ep.ip; }

 private:
  void OnSocketRx(::ns3::Ptr<::ns3::Socket> sock);

  RadioEndpoint m_ep;
  RxCallback m_rx_cb;
  std::unordered_map<uint8_t, ::ns3::Ipv4Address> m_id_to_ip;
};

}  // namespace sim
