#pragma once
#include <vector>
#include <cmath>

class Position {
    public: 
        Position(
            float x_value,
            float y_value,
            float z_value
        );
        ~Position() = default; 
        void retrieveCurrentPosition();
        std::vector<float> getCoordinates() const;
        float module() const;
        Position unit_vector() const;
        Position operator-(const Position& other) const;
        friend Position operator*(float scalar, const Position& position);
    
    private:
        float x;
        float y;
        float z;
};