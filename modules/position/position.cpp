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

PositionInterface* Position::distanceFrom(const PositionInterface* other) const {
    const Position* other_position = dynamic_cast<const Position*>(other);
    if (other_position == nullptr) {
        return nullptr;
    }
    return new Position(
        other_position->x - this->x,
        other_position->y - this->y,
        other_position->z - this->z
    );
}

PositionInterface* Position::unit_vector() const {
    float mod = this->module();
    if (mod == 0) {
        // Avoid division by zero
        return new Position(0.0, 0.0, 0.0);
    } 
    return new Position{
        this->x / mod,
        this->y / mod,
        this->z / mod
    };
}

PositionInterface* Position::multiplyByScalar(float scalar) const {
    return new Position(
        this->x * scalar,
        this->y * scalar,
        this->z * scalar
    );
}