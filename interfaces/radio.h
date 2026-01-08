#pragma once

#include <cstdint>
#include <vector>

constexpr uint8_t BROADCAST_ID = 0xFF;

struct Packet {
    uint8_t src;
    uint8_t dst;  // BROADCAST_ID = broadcast
    std::vector<uint8_t> payload;
};

struct NeighborInfo {
    uint8_t neighbor_id;
    uint8_t hops_from_base_station;
    Position position;
};

class RadioInterface {
   public:
    virtual ~RadioInterface() = default;
    virtual void send(const Packet& pkt) = 0;
    virtual std::vector<NeighborInfo*> getNeighbors() = 0;
};