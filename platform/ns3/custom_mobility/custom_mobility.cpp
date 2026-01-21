#include "platform/ns3/custom_mobility/custom_mobility.h"

CustomMobility::CustomMobility(
    ns3::Ptr<ns3::GeocentricConstantPositionMobilityModel> mobility
) : 
    mobility(mobility),
    previous_position(0.0, 0.0, 0.0),
    previous_time_s(ns3::Simulator::Now().GetSeconds()),
    velocity(0.0, 0.0, 0.0) // init as a stationary object
{
    if (this->mobility) {
        previous_position = this->mobility->GetPosition();
    }
}

void CustomMobility::setPosition(
    double longitude,
    double latitude, 
    double altitude
) { 
    if (!mobility) {
        return;
    }

    // Treat (longitude, latitude, altitude) as a generic (x, y, z) coordinate.
    mobility->SetPosition(ns3::Vector(longitude, latitude, altitude));
    previous_position = mobility->GetPosition();
    previous_time_s = ns3::Simulator::Now().GetSeconds();
}

std::vector<double> CustomMobility::getPosition() {
    if (!mobility) {
        return {0.0, 0.0, 0.0};
    }

    const double now_s = ns3::Simulator::Now().GetSeconds();
    const double dt_s = now_s - previous_time_s;
    if (dt_s <= 0.0) {
        // No simulation time advanced since last update.
        return {previous_position.x, previous_position.y, previous_position.z};
    }

    // Update position based on velocity and time elapsed:
    const double new_longitude = previous_position.x + velocity.x * dt_s;
    const double new_latitude = previous_position.y + velocity.y * dt_s;
    const double new_altitude = previous_position.z + velocity.z * dt_s;

    previous_time_s = now_s;
    mobility->SetPosition(ns3::Vector(new_longitude, new_latitude, new_altitude));
    previous_position = mobility->GetPosition();
    return {
        previous_position.x, 
        previous_position.y, 
        previous_position.z
    };
}

void CustomMobility::setVelocity(const Vector3D new_velocity) {
    // update position before changing velocity
    getPosition();

    velocity = new_velocity;    
}