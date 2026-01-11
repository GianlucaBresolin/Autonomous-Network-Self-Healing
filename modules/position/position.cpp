#include "position.h"

Position::Position(
    float x_value,
    float y_value,
    float z_value
) : 
    x(x_value), 
    y(y_value), 
    z(z_value) 
{ }

void Position::retrieveCurrentPosition() {
    // todo: implement actual position retrieval
}

std::vector<float> Position::getCoordinates() const {
    return {x, y, z};
}

float Position::module() const {
    return std::sqrt( x * x + y * y + z * z);
}

std::unique_ptr<PositionInterface> Position::distanceFrom(const PositionInterface* other) const {
    const Position* other_position = dynamic_cast<const Position*>(other);
    if (other_position == nullptr) {
        return nullptr;
    }
    return std::make_unique<Position>(
        other_position->x - x,
        other_position->y - y,
        other_position->z - z
    );
}

std::unique_ptr<PositionInterface> Position::unit_vector() const {
    float mod = module();
    if (mod == 0) {
        // Avoid division by zero
        return std::make_unique<Position>(0.0, 0.0, 0.0);
    } 
    return std::make_unique<Position>(
        x / mod,
        y / mod,
        z / mod
    );
}

std::unique_ptr<PositionInterface> Position::multiplyByScalar(float scalar) const {
    return std::make_unique<Position>(
        x * scalar,
        y * scalar,
        z * scalar
    );
}