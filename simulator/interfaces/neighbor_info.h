#pragma once
#include "position.h"

class NeighborInfoInterface {
    public:
        virtual ~NeighborInfoInterface() = default;
        virtual PositionInterface* getPosition() const = 0;
        virtual uint8_t getHopsToBaseStation() const = 0;
};