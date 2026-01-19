#include "ns3_neighbor_manager.h"

Ns3NeighborManager::Ns3NeighborManager(
    CommunicationManagerInterface* communication_manager
) : 
    communication_manager(communication_manager) 
{ }

Ns3NeighborManager::~Ns3NeighborManager() {
    for (auto const& [id, info] : neighbor_info_buffer) {
        delete info;
    }
}

void Ns3NeighborManager::onPacketReceived(const ::Packet& pkt) {
    if (neighbor_info_buffer.count(pkt.src)) {
        delete neighbor_info_buffer[pkt.src];
    }

    NeighborInfo* neighbor_info = new NeighborInfo();
    neighbor_info->deserialize(pkt.payload);

    neighbor_info_buffer[pkt.src] = neighbor_info;
}
 
std::vector<NeighborInfoInterface*> Ns3NeighborManager::getNeighbors() const {
    std::vector<NeighborInfoInterface> neighbors;
    for (const auto& [id, info] : neighbor_info_buffer) {
        neighbors.push_back(info);
    }
    return neighbors;
}

void Ns3NeighborManager::sendToNeighbors(
    uint8_t id,
    PositionInterface* position,
    uint8_t hops_to_base_station
) {
    // create NeighborInfo 
    NeighborInfo our_neighbor_info(
        id, 
        hops_to_base_station,
        position->getCoordinates()
    )
    // buil packet
    ::Packet pkt;
    pkt.src = id;
    pkt.dst = BROADCAST_ID; 

    our_neighbor_info.serialize(pkt.payload);

    // broadcast to neighbors
    communication_manager->send(pkt);
}