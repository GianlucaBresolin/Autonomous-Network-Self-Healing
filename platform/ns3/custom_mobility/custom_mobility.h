#pragma once
#include <vector>
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "common/vector3D.h"

class CustomMobility {
    public:
        CustomMobility(ns3::Ptr<ns3::GeocentricConstantPositionMobilityModel> mobility);
        void setPosition(double longitude, double latitude, double altitude); 
        std::vector<double> getPosition();
        void setVelocity(const Vector3D new_velocity);
        
    private:
        ns3::Ptr<ns3::MobilityModel> mobility;
        ns3::Vector previous_position;
        double previous_time_s = 0.0;
        Vector3D velocity;
    };