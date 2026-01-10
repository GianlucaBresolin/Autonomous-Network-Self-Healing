#include "neighbor_info.h"

NeighborInfo::NeighborInfo(
    uint8_t id,
    uint8_t hops,
    PositionInterface* position
) : 
    neighbor_id(id),
    hops_from_base_station(hops),
    position(position)
{ }

PositionInterface* NeighborInfo::getPosition() const {
    return position;
}

uint8_t NeighborInfo::getHopsToBaseStation() const {
    return hops_from_base_station;
}