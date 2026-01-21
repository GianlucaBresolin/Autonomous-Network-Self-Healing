#include "modules/neighbor/neighbor_info.h"

NeighborInfo::NeighborInfo(
    uint8_t id,
    uint8_t hops,
    const std::vector<double>& coordinates
) : 
    neighbor_id(id),
    hops_from_base_station(hops),
    position(coordinates)
{ }

std::vector<double> NeighborInfo::getPosition() const {
    return position;
}

uint8_t NeighborInfo::getHopsToBaseStation() const {
    return hops_from_base_station;
}

void NeighborInfo::serialize(std::vector<uint8_t>& out_payload) const {
    const size_t position_size = position.size() * sizeof(double);
    out_payload.resize(2 + position_size);

    out_payload[0] = neighbor_id;
    out_payload[1] = hops_from_base_station;
    if (position_size > 0) {
        std::memcpy(&out_payload[2], position.data(), position_size);
    }
}

void NeighborInfo::deserialize(const std::vector<uint8_t>& in_payload) {
    if (in_payload.size() < 2) {
        throw std::invalid_argument("Payload too small to deserialize NeighborInfo");
    }

    const size_t position_size = in_payload.size() - 2;
    position.resize(position_size / sizeof(double));

    neighbor_id = in_payload[0];
    hops_from_base_station = in_payload[1];
    if (position_size > 0) {
        std::memcpy(position.data(), &in_payload[2], position_size);
    }
}