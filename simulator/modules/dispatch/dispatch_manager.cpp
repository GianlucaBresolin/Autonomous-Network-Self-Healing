#include "modules/dispatch/dispatch_manager.h"

void DispatchManager::SetFloodingManager(FloodingManagerInterface* flooding_manager) {
    m_flooding_manager = flooding_manager;
}

void DispatchManager::SetFallbackHandler(FallbackHandler handler) {
    m_fallback_handler = std::move(handler);
}

void DispatchManager::HandlePacket(const ::Packet& pkt) const {
    if (pkt.payload.empty()) {
        return;
    }

    if (IsFloodingPacket(pkt)) {
        if (m_flooding_manager) {
            m_flooding_manager->onPacketReceived(pkt);
        }
        return;
    }

    if (m_fallback_handler) {
        m_fallback_handler(pkt);
    }
}

bool DispatchManager::IsFloodingPacket(const ::Packet& pkt) {
    if (pkt.payload.empty()) {
        return false;
    }

    const uint8_t type = pkt.payload[0];
    return type == static_cast<uint8_t>(FloodMsgType::START)
        || type == static_cast<uint8_t>(FloodMsgType::DISCOVERY)
        || type == static_cast<uint8_t>(FloodMsgType::REPORT)
        || type == static_cast<uint8_t>(FloodMsgType::HOP_ENTRY)
        || type == static_cast<uint8_t>(FloodMsgType::BASE_PROBE);
}
