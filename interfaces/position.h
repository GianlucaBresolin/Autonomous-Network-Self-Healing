#pragma once
#include <vector>

class PositionInterface {
    public:
        virtual ~PositionInterface() = default;
        virtual void retrieveCurrentPosition() = 0;
        virtual std::vector<float> getCoordinates() const = 0;
        virtual float module() const = 0;
        virtual PositionInterface* distanceFrom(const PositionInterface* other) const = 0;
        virtual PositionInterface* unit_vector() const = 0;
        virtual PositionInterface* multiplyByScalar(float scalar) const = 0;
};