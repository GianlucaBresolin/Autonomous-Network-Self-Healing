#include "ns3_position.h"

Ns3Position::Ns3Position(
    ns3::Ptr<ns3::MobilityModel> mobility
) : mobility(mobility)
{ 
    ns3::Vector pos = mobility->GetPosition();
    latitude = pos.x;
    longitude = pos.y;
    altitude = pos.z;
}

void Ns3Position::retrieveCurrentPosition() {
    ns3::Vector pos = mobility->GetGeographicPosition();
    latitude = pos.x;
    longitude = pos.y;
    altitude = pos.z;
}

std::vector<double> Ns3Position::getCoordinates() const {
    ns3::Vector coordinates = ns3::GeographicPositions::GeographicToGeocentricCoordinates(
        latitude, longitude, altitude,
        ns3::GeographicPosition::WGS84
    );
    return {
        coordinates.x, 
        coordinates.y, 
        coordinates.z
    };
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