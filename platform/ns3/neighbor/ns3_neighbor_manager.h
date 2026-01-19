#pragma once
#include <map>
#include <vector>
#include "../../interfaces/neighbor_manager.h"
#include "../../interfaces/communication_manager.h"
#include "neighbor_info.h"

class Ns3NeighborManager : public NeighborManagerInterface {
    public:
        Ns3NeighborManager(
            uint8_t self_id,
            CommunicationManagerInterface* communication_manager
        );
        ~Ns3NeighborManager() override;
        void onPacketReceived(const ::Packet& pkt) override;
        std::vector<NeighborInfoInterface*> getNeighbors() const override;
        void sendToNeighbors(
            uint8_t id, 
            PositionInterface* position,
            uint8_t hops_to_base_station
        ) override; 

    private: 
        CommunicationManagerInterface* communication_manager;
        std::map<uint8_t, NeighborInfo*> neighbor_info_buffer;
};