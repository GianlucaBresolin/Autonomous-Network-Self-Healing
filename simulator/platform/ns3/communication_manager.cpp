#include "platform/ns3/communication_manager.h"

#include <cstring>
#include <iostream>
#include <vector>

#include "interfaces/communication_manager.h"

namespace sim {

CommunicationManager::CommunicationManager(
    uint8_t self_id,
    SendUnicastFn send_unicast,
    SendBroadcastFn send_broadcast
) :
    m_self_id(self_id),
    m_send_unicast(std::move(send_unicast)),
    m_send_broadcast(std::move(send_broadcast))
{ }

void CommunicationManager::RegisterPeer(uint8_t id, ::ns3::Ipv4Address ip) {
    m_id_to_ip[id] = ip;
}

void CommunicationManager::SetReceiveHandler(ReceiveHandler handler) {
    m_on_receive = std::move(handler);
}

void CommunicationManager::send(const ::Packet& pkt) {
    if (!m_send_unicast || !m_send_broadcast) {
        return;
    }

    std::vector<uint8_t> bytes;
    bytes.resize(2 + pkt.payload.size());
    bytes[0] = pkt.src;
    bytes[1] = pkt.dst;
    if (!pkt.payload.empty()) {
        std::memcpy(bytes.data() + 2, pkt.payload.data(), pkt.payload.size());
    }

    ::ns3::Ptr<::ns3::Packet> p = ::ns3::Create<::ns3::Packet>(bytes.data(), static_cast<uint32_t>(bytes.size()));

    if (pkt.dst == BROADCAST_ID) {
        m_send_broadcast(p);
        return;
    }

    auto it = m_id_to_ip.find(pkt.dst);
    if (it == m_id_to_ip.end()) {
        std::cerr << "[Comm " << int(m_self_id) << "] missing IP for dst " << int(pkt.dst) << std::endl;
        return;
    }
    m_send_unicast(it->second, p);
}

void CommunicationManager::receive(::Packet& pkt) {
    if (m_on_receive) {
        m_on_receive(pkt);
    }
}

void CommunicationManager::HandleRadioPacket(::ns3::Ptr<::ns3::Packet> p) {
    if (!p || !m_on_receive) {
        return;
    }

    const uint32_t size = p->GetSize();
    if (size < 2) {
        return;
    }

    std::vector<uint8_t> bytes(size);
    p->CopyData(bytes.data(), size);

    ::Packet decoded;
    decoded.src = bytes[0];
    decoded.dst = bytes[1];
    if (size > 2) {
        decoded.payload.assign(bytes.begin() + 2, bytes.end());
    }

    m_on_receive(decoded);
}

}  // namespace sim
