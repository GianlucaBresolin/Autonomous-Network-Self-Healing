#pragma once
#include <cstdint>  
#include "../../interfaces/neighbor_manager.h"

class NeighborInfo : public NeighborInfoInterface {
    public: 
        NeighborInfo(
            uint8_t id,
            uint8_t hops,
            PositionInterface* position
        );

        PositionInterface* getPosition() const override;
        uint8_t getHopsToBaseStation() const override;

    private:
    uint8_t neighbor_id;
    uint8_t hops_from_base_station;
    PositionInterface* position;
};