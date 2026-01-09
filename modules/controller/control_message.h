#pragma once
#include <cstdint>
#include "../../interfaces/position.h"

template <typename T>
struct ControlMessage {
    uint8_t id;
    T position;
    uint8_t hops_from_base_station;
};