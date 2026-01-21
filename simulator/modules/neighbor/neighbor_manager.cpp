#include "modules/neighbor/neighbor_manager.h"

#include <cstring>

NeighborManager::NeighborManager(
    CommunicationManagerInterface* communication_manager
) : 
    m_communication_manager(communication_manager) 
{ }

void NeighborManager::onPacketReceived(const ::Packet& pkt) {
    if (pkt.type != ::PacketType::NEIGHBOR) {
        return;
    }
    if (pkt.payload.size() < 2) {
        return;
    }

    // Payload format: [neighbor_id][hops][double coords...]
    const uint8_t neighbor_id = pkt.payload[0];
    if (neighbor_id != pkt.src) {
        // Basic sanity check: outer header src should match payload id.
        return;
    }

    const uint8_t hops = pkt.payload[1];
    const size_t coord_bytes = pkt.payload.size() - 2;
    if (coord_bytes % sizeof(double) != 0) {
        return;
    }

    std::vector<double> coords;
    coords.resize(coord_bytes / sizeof(double));
    if (coord_bytes > 0) {
        std::memcpy(coords.data(), pkt.payload.data() + 2, coord_bytes);
    }

    m_neighbors[neighbor_id] = std::make_unique<NeighborInfo>(neighbor_id, hops, coords);
}
 
std::vector<NeighborInfoInterface*> NeighborManager::getNeighbors() const {
    std::vector<NeighborInfoInterface*> neighbors;
    neighbors.reserve(m_neighbors.size());
    for (const auto& kv : m_neighbors) {
        neighbors.push_back(kv.second.get());
    }
    return neighbors;
}

void NeighborManager::sendToNeighbors(
    uint8_t id,
    PositionInterface* position,
    uint8_t hops_to_base_station
) {
    if (!m_communication_manager || !position) {
        return;
    }

    const std::vector<double> coords = position->getCoordinates();
    NeighborInfo info(id, hops_to_base_station, coords);

    ::Packet pkt;
    pkt.type = ::PacketType::NEIGHBOR;
    pkt.src = id;
    pkt.dst = BROADCAST_ID;
    info.serialize(pkt.payload);

    m_communication_manager->send(pkt);
}