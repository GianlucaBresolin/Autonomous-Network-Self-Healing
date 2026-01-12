#pragma once 
#include <cstdint>
#include <vector>
#include "communication_manager.h"

class FloodingManagerInterface {
    public: 
        virtual ~FloodingManagerInterface() = default;
        virtual void onPacketReceived(const ::Packet& pkt) = 0;
        virtual std::vector<std::pair<uint8_t, uint8_t>> getHopTableToBase(uint16_t flood_id) const = 0;
        virtual uint8_t getHopsFromBase() const = 0;
    private:
        virtual void startFlood(uint16_t flood_id) = 0;
};