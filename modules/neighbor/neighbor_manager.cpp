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
