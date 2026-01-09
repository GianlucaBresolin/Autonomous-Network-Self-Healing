#pragma once
#include <vector>

template <typename T> // Position
class PositionInterface {
    public:
        virtual ~PositionInterface() = default;
        virtual void retrieveCurrentPosition() = 0;
        virtual std::vector<float> getCoordinates() const = 0;
        virtual float module() const = 0;
        virtual T unit_vector() const = 0;
        virtual T operator-(const T& other) const = 0;
        friend T operator*(float scalar, const T& position) {
            return position.multiplyByScalar(scalar);
        }
    private: 
        virtual T multiplyByScalar(float scalar) const = 0;
};