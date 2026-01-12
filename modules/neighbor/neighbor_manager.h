#pragma once
#include "../../interfaces/neighbor_manager.h"
#include "../../interfaces/communication_manager.h"
#include "neighbor_info.h"

class NeighborManager : public NeighborManagerInterface {
    public:
        NeighborManager(CommunicationManagerInterface* communication_manager);
        std::vector<NeighborInfoInterface*> getNeighbors() const override;
        void sendToNeighbors(
            uint8_t id, 
            PositionInterface* position,
            uint8_t hops_to_base_station
        ) override; 

    private: 
        CommunicationManagerInterface* communication_manager;
};