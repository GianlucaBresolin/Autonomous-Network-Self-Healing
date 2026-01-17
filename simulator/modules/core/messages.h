#pragma once
#include <cstdint>

// Generic simulator messages shared by base and drones.
// Values are outside flooding protocol range (0..3) to simplify dispatch.
enum class SimMsgType : uint8_t {
    NODE_INFO_REQUEST = 0x80,
    NODE_INFO_RESPONSE = 0x81,
};

#pragma pack(push, 1)

struct NodeInfoRequestMsg {
    SimMsgType type = SimMsgType::NODE_INFO_REQUEST;
    uint8_t requester_id;
};

struct NodeInfoResponseMsg {
    SimMsgType type = SimMsgType::NODE_INFO_RESPONSE;
    uint8_t node_id;
    float x;
    float y;
    float z;
};

#pragma pack(pop)

static_assert(sizeof(NodeInfoRequestMsg) == 2, "NodeInfoRequestMsg must be packed");
static_assert(sizeof(NodeInfoResponseMsg) == 14, "NodeInfoResponseMsg must be packed");
