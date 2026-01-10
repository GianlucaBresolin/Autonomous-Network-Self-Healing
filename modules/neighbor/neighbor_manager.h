#pragma once
#include "../../interfaces/neighbor_manager.h"
#include "../../interfaces/communication_manager.h"
#include "neighbor_info.h"

class NeighborManager : public NeighborManagerInterface {
    public:
        NeighborManager(CommunicationManagerInterface* communication_manager);
        std::vector<NeighborInfoInterface*> getNeighbors() const override;

    private: 
        CommunicationManagerInterface* communication_manager;
};