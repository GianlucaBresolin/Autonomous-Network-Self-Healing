#pragma once
#include <vector>
#include "common/vector3D.h"

class PositionInterface {
    public:
        virtual ~PositionInterface() = default;
        virtual void retrieveCurrentPosition() = 0;
        virtual std::vector<double> getCoordinates() const = 0;
        virtual Vector3D distanceFrom(const PositionInterface* other) const = 0;
        virtual Vector3D distanceFromCoords(const std::vector<double>& other_coords) const = 0;
};