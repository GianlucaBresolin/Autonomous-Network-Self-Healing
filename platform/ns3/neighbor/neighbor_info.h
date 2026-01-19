#pragma once
#include <cstdint>  
#include <vector>
#include "../../interfaces/neighbor_manager.h"
#include "../position/ns3_position.h"

enum class NeighborMsgType : uint8_t {
    NEIGHBOR_INFO = 10,
};

class NeighborInfo : public NeighborInfoInterface {
    public: 
        NeighborInfo(
            uint8_t id,
            uint8_t hops,
            std::vector<double> coordinates
        );

        std::vector<double> getPosition() const override;
        uint8_t getHopsToBaseStation() const override;
        void serialize(std::vector<uint8_t>& out_payload) const override;
        void deserialize(const std::vector<uint8_t>& in_payload) override;

    private:
    uint8_t neighbor_id;
    uint8_t hops_from_base_station;
    std::vector<double> position;
};