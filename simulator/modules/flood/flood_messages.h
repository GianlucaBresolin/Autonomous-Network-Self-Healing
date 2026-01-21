#pragma once
#include <cstdint>

// Message types used by the flooding protocol.
//
// - START:      base (host) -> initiator (direct link / appchannel)
// - DISCOVERY:  swarm broadcast, forwarded hop-by-hop
// - REPORT:     swarm broadcast, each node reports its best hop-to-initiator
enum class FloodMsgType : uint8_t {
    START = 0,
    DISCOVERY = 1,
    REPORT = 2,
};

#pragma pack(push, 1)

// Base -> initiator
struct FloodStartMsg {
    FloodMsgType type = FloodMsgType::START;
    uint16_t flood_id;
};

struct FloodDiscoveryMsg {
    FloodMsgType type = FloodMsgType::DISCOVERY;
    uint16_t flood_id;
    uint8_t initiator_id;
    uint8_t hop_to_base;
};

// Swarm broadcast: report best hop to base
struct FloodReportMsg {
    FloodMsgType type = FloodMsgType::REPORT;
    uint16_t flood_id;
    uint8_t initiator_id;
    uint8_t reporter_id;
    uint8_t hop_to_base;
};

#pragma pack(pop)

static_assert(sizeof(FloodStartMsg) == 3, "FloodStartMsg must be packed");
static_assert(sizeof(FloodDiscoveryMsg) == 5, "FloodDiscoveryMsg must be packed");
static_assert(sizeof(FloodReportMsg) == 6, "FloodReportMsg must be packed");
