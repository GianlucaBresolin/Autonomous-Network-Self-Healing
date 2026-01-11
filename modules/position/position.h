#pragma once
#include <vector>
#include <cmath>
#include <memory>
#include "../../interfaces/position.h"
#include "position.h"

class Position : public PositionInterface {
    public: 
        Position(
            float x_value = 0.0,
            float y_value = 0.0,
            float z_value = 0.0
        );
        void retrieveCurrentPosition() override;
        std::vector<float> getCoordinates() const override;
        float module() const override;
        std::unique_ptr<PositionInterface> distanceFrom(const PositionInterface* other) const override;
        std::unique_ptr<PositionInterface> unit_vector() const;
        std::unique_ptr<PositionInterface> multiplyByScalar(float scalar) const;
    private: 
        float x;
        float y;
        float z;
};