#pragma once

#include <unordered_set>

#include "interfaces/radio.h"
#include "modules/flooding/messages.h"

class Flooding {
   public:
    Flooding(uint8_t self_id, uint8_t base_id, RadioInterface& radio);

    // Base station calls this
    void startFlood(uint16_t flood_id);

    // Everyone calls this on RX
    void onPacketReceived(const Packet& pkt);

   private:
    uint8_t self_id;
    uint8_t base_id;
    RadioInterface& radio;

    std::unordered_set<uint16_t> seen_floods;

    void handleDiscovery(const FloodDiscoveryMsg& msg);
};