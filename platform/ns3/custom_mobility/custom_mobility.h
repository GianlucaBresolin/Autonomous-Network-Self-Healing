#pragma once
#include <chrono>
#include <vector>
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "../../../common/vector3D.h"

class CustomMobility {
    public:
        CustomMobility(ns3::Ptr<ns3::GeocentricConstantPositionMobilityModel> mobility);
        void setPosition(double longitude, double latitude, double altitude); 
        std::vector<double> getPosition();
        void setVelocity(const Vector3D new_velocity);
        
    private:
        ns3::Ptr<ns3::MobilityModel> mobility;
        ns3::Vector previous_position;
        std::chrono::steady_clock::time_point previous_time;
        Vector3D velocity;
}