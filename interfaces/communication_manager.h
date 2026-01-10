#pragma once

#include <cstdint>
#include <vector>

constexpr uint8_t BROADCAST_ID = 0xFF;

struct Packet {
    uint8_t src;
    uint8_t dst;  // BROADCAST_ID = broadcast
    std::vector<uint8_t> payload;
};

class CommunicationManagerInterface {
   public:
    virtual ~CommunicationManagerInterface() = default;
    virtual void send(const Packet& pkt) = 0;
    virtual void receive(Packet& pkt) = 0;
};