#pragma once

#include <cstdint>
#include <functional>
#include <unordered_map>

#include "interfaces/communication_manager.h"
#include "ns3/network-module.h"

namespace sim {

class CommunicationManager : public CommunicationManagerInterface {
  public:
    using SendUnicastFn = std::function<void(::ns3::Ipv4Address, ::ns3::Ptr<::ns3::Packet>)>;
    using SendBroadcastFn = std::function<void(::ns3::Ptr<::ns3::Packet>)>;
    using ReceiveHandler = std::function<void(const ::Packet&)>;

    CommunicationManager(
        uint8_t self_id,
        SendUnicastFn send_unicast,
        SendBroadcastFn send_broadcast
    );

    void RegisterPeer(uint8_t id, ::ns3::Ipv4Address ip);
    void SetReceiveHandler(ReceiveHandler handler);

    void send(const ::Packet& pkt) override;
    void receive(::Packet& pkt) override;

    void HandleRadioPacket(::ns3::Ptr<::ns3::Packet> p);

  private:
    uint8_t m_self_id;
    SendUnicastFn m_send_unicast;
    SendBroadcastFn m_send_broadcast;
    ReceiveHandler m_on_receive;
    std::unordered_map<uint8_t, ::ns3::Ipv4Address> m_id_to_ip;
};

}  // namespace sim
