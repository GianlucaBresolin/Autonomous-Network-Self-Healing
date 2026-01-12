#include "neighbor_manager.h"

NeighborManager::NeighborManager(
    CommunicationManagerInterface* communication_manager
) : 
    communication_manager(communication_manager) 
{ }
 
std::vector<NeighborInfoInterface*> NeighborManager::getNeighbors() const {
    // to be implemented: retrieve neighbor information via
    // communication_manager as NeighborInfo objects
}

void NeighborManager::sendToNeighbors(
    uint8_t id,
    PositionInterface* position,
    uint8_t hops_to_base_station
) {
    // to be implemented: send neighbor information via
    // communication_manager (only in our range)
}