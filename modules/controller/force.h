#pragma once
#include "../position/position.h"

struct Force {
    float x;
    float y;
    float z;
};

Force operator+(const Force& f, const Position& p) {
    std::vector<float> coords = p.getCoordinates();
    return Force{
        f.x + coords[0], // x
        f.y + coords[1], // y
        f.z + coords[2]  // z
    };
}