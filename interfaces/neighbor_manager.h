#pragma once
#include <vector> 
#include "neighbor_info.h"

class NeighborManagerInterface {
    public:
        virtual ~NeighborManagerInterface() = default;
        virtual std::vector<NeighborInfoInterface*> getNeighbors() const = 0;
};