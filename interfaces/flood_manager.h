#pragma once 
#include <cstdint>
#include <vector>
#include "communication_manager.h"

class FloodManagerInterface {
    public: 
        virtual ~FloodManagerInterface() = default;
        virtual void onPacketReceived(const ::Packet& pkt) = 0;
        virtual uint8_t getHopsFromBase() const = 0;
    private:
        virtual void startFlood(uint16_t flood_id) = 0;
};