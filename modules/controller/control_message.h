#pragma once
#include <cstdint>
#include "../position/position.h"

struct ControlMessage {
    uint8_t id;
    Position position;
    uint8_t hops_from_base_station;
};