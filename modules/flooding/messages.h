#pragma once
#include <cstdint>

enum class FloodMsgType : uint8_t { DISCOVERY = 1, REPLY = 2 };

struct FloodDiscoveryMsg {
    FloodMsgType type = FloodMsgType::DISCOVERY;
    uint16_t flood_id;
    uint8_t origin;
};

struct FloodReplyMsg {
    FloodMsgType type = FloodMsgType::REPLY;
    uint16_t flood_id;
    uint8_t responder;
};