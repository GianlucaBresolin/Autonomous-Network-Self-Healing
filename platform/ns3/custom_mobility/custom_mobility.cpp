#include "custom_mobility.h"

CustomMobility::CustomMobility(
    ns3::Ptr<ns3::GeocentricConstantPositionMobilityModel> mobility
) : 
    mobility(mobility)
    velocity(0, 0, 0) // init as a stationary object
{ }

void CustomMobility::setPosition(
    double longitude,
    double latitude, 
    double altitude
) { 
    mobility->SetGeographicPosition(
        latitude, 
        longitude, 
        altitude,
        ns3::GeographicPosition::WGS84
    );
    previous_time = std::chrono::steady_clock::now();
}

std::vector<double> CustomMobility::getPosition() {
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_diff = current_time - previous_time;

    // update position based on velocity and time elapsed:
    double new_longitude = previous_position.x + velocity.x * time_diff.count(); 
    double new_latitude = previous_position.y + velocity.y * time_diff.count();  
    double new_altitude = previous_position.z + velocity.z * time_diff.count(); 

    previous_time = current_time;
    mobility->SetPosition(
        ns3::Vector(new_longitude, new_latitude, new_altitude)
    );
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