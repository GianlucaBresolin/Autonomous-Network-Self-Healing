#pragma once
#include <cstdint>
#include "../../interfaces/position.h"

struct ControlMessage {
    uint8_t id;
    PositionInterface* position;
    uint8_t hops_from_base_station;
};