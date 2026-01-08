#include "position.h"

Position::Position(
    float x_value,
    float y_value,
    float z_value
) : x(x_value), y(y_value), z(z_value) 
{ }

void Position::retrieveCurrentPosition() {
    // todo: implement actual position retrieval
}

std::vector<float> Position::getCoordinates() const {
    return {x, y, z};
}

float Position::module() const {
    return std::sqrt(x * x + y * y + z * z);
}

Position Position::unit_vector() const {
    float mod = module();
    if (mod == 0) {
        // Avoid division by zero
        return Position(0.0f, 0.0f, 0.0f); 
    } 
    return Position(
        x / mod,
        y / mod,
        z / mod
    );
}

Position Position::operator-(const Position& other) const {
    Position aux = Position(
        x - other.x,
        y - other.y,
        z - other.z
    );
    return aux;
}

Position operator*(float scalar, const Position& position) {
    return Position(
        scalar * position.x,
        scalar * position.y,
        scalar * position.z
    );
}