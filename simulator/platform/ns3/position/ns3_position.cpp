#include "ns3_position.h"

Ns3Position::Ns3Position(
    CustomMobility* mobility
) : mobility(mobility)
{ 
    if (mobility) {
        const std::vector<double> pos = mobility->getPosition();
        if (pos.size() >= 3) {
            latitude = pos[0];
            longitude = pos[1];
            altitude = pos[2];
        }
    }
}

void Ns3Position::retrieveCurrentPosition() {
    if (!mobility) {
        return;
    }
    const std::vector<double> pos = mobility->getPosition();
    if (pos.size() >= 3) {
        latitude = pos[0];
        longitude = pos[1];
        altitude = pos[2];
    }
}

std::vector<double> Ns3Position::getCoordinates() const {
    return {latitude, longitude, altitude};
}

Vector3D Ns3Position::distanceFrom(const PositionInterface* other) const {
    std::vector<double> my_coords = getCoordinates();
    std::vector<double> other_coords = other->getCoordinates();
    return Vector3D(
        other_coords[0] - my_coords[0],
        other_coords[1] - my_coords[1],
        other_coords[2] - my_coords[2]
    );
}

Vector3D Ns3Position::distanceFromCoords(const std::vector<double>& other_coords) const {
    std::vector<double> my_coords = getCoordinates();
    return Vector3D(
        other_coords[0] - my_coords[0],
        other_coords[1] - my_coords[1],
        other_coords[2] - my_coords[2]
    );
}