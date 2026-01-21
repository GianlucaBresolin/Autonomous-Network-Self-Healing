#pragma once
#include <vector>
#include "interfaces/position.h"
#include "platform/ns3/custom_mobility/custom_mobility.h"
#include "ns3/core-module.h"

class Ns3Position : public PositionInterface {
    public: 
        Ns3Position(CustomMobility* mobility);
        void retrieveCurrentPosition() override;
        std::vector<double> getCoordinates() const override;
        Vector3D distanceFrom(const PositionInterface* other) const override;
        Vector3D distanceFromCoords(const std::vector<double>& other_coords) const override;
    private: 
        CustomMobility* mobility;
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;
};