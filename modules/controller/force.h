#pragma once
#include "../../interfaces/position.h"

struct Force {
    float x;
    float y;
    float z;
};

Force operator+(const Force& f, const std::unique_ptr<PositionInterface>& p) {
    std::vector<float> coords = p->getCoordinates();
    return Force{
        f.x + (coords.size() > 0 ? coords[0] : 0), // x
        f.y + (coords.size() > 1 ? coords[1] : 0), // y
        f.z + (coords.size() > 2 ? coords[2] : 0)  // z
    };
}

Force operator-(const Force& f, const std::unique_ptr<PositionInterface>& p) {
    std::vector<float> coords = p->getCoordinates();
    return Force{
        f.x - (coords.size() > 0 ? coords[0] : 0), // x
        f.y - (coords.size() > 1 ? coords[1] : 0), // y
        f.z - (coords.size() > 2 ? coords[2] : 0)  // z
    };
}