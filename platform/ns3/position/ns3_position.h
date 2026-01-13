#pragma once
#include <vector>
#include "../../../interfaces/position.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

class Ns3Position : public PositionInterface {
    public: 
        Ns3Position(ns3::Ptr<ns3::GeocentricConstantPositionMobilityModel> mobility);
        void retrieveCurrentPosition() override;
        std::vector<double> getCoordinates() const override;
        std::unique_ptr<Vector3D> distanceFrom(const PositionInterface* other) const override;
    private: 
        ns3::Ptr<ns3::MobilityModel> mobility;
        double latitude;
        double longitude;
        double altitude;
};