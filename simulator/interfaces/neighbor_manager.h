#pragma once
#include <vector> 
#include "neighbor_info.h"

class NeighborManagerInterface {
    public:
        virtual ~NeighborManagerInterface() = default;
        virtual std::vector<NeighborInfoInterface*> getNeighbors() const = 0;
        virtual void sendToNeighbors(
            uint8_t id, 
            PositionInterface* position,
            uint8_t hops_to_base_station
        ) = 0;
};