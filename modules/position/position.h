#pragma once
#include <vector>
#include <cmath>
#include "../../interfaces/position.h"

class Position : public PositionInterface<Position> {
    public: 
        Position(
            float x_value,
            float y_value,
            float z_value
        );
        ~Position() = default; 
        void retrieveCurrentPosition() override;
        std::vector<float> getCoordinates() const override;
        float module() const override;
        Position unit_vector() const override;
        Position operator-(const Position& other) const override;
        friend Position operator*(float scalar, const Position& position);
    
    private:
        float x;
        float y;
        float z;

        Position multiplyByScalar(float scalar) const override;
};