#pragma once
#include <vector>
#include <memory>

class PositionInterface {
    public:
        virtual ~PositionInterface() = default;
        virtual void retrieveCurrentPosition() = 0;
        virtual std::vector<float> getCoordinates() const = 0;
        virtual float module() const = 0;
        virtual std::unique_ptr<PositionInterface> distanceFrom(const PositionInterface* other) const = 0;
        virtual std::unique_ptr<PositionInterface> unit_vector() const = 0;
        virtual std::unique_ptr<PositionInterface> multiplyByScalar(float scalar) const = 0;
};