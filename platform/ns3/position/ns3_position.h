#pragma once
#include <vector>
#include "../../../interfaces/position.h"
#include "../custom_mobility/custom_mobility.h"
#include "ns3/core-module.h"
#include "ns3/geographic-positions.h"

class Ns3Position : public PositionInterface {
    public: 
        Ns3Position(CustomMobility* mobility);
        void retrieveCurrentPosition() override;
        std::vector<double> getCoordinates() const override;
        Vector3D distanceFrom(const PositionInterface* other) const override;
    private: 
        CustomMobility* mobility;
        double latitude;
        double longitude;
        double altitude;
};