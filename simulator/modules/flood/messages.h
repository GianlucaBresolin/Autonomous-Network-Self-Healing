#pragma once
#include <cstdint>

// Message types used by the flooding protocol.
//
// - START:      base (host) -> initiator (direct link / appchannel)
// - DISCOVERY:  swarm broadcast, forwarded hop-by-hop
// - REPORT:     swarm broadcast, each node reports its best hop-to-initiator
// - HOP_ENTRY:  initiator -> base (direct link / appchannel)
enum class FloodMsgType : uint8_t {
    START = 0,
    DISCOVERY = 1,
    REPORT = 2,
    HOP_ENTRY = 3,
    BASE_PROBE = 4,
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
    uint8_t hop_to_initiator;
};

// Swarm broadcast: report best hop to initiator
struct FloodReportMsg {
    FloodMsgType type = FloodMsgType::REPORT;
    uint16_t flood_id;
    uint8_t initiator_id;
    uint8_t reporter_id;
    uint8_t hop_to_initiator;
};

// Initiator -> base
struct FloodHopEntryMsg {
    FloodMsgType type = FloodMsgType::HOP_ENTRY;
    uint16_t flood_id;
    uint8_t initiator_id;
    uint8_t node_id;
    uint8_t hop_to_base;
};

// Drone -> base (direct link) to check reachability during a flood.
struct FloodBaseProbeMsg {
    FloodMsgType type = FloodMsgType::BASE_PROBE;
    uint16_t flood_id;
    uint8_t initiator_id;
    uint8_t reporter_id;
};

#pragma pack(pop)

static_assert(sizeof(FloodStartMsg) == 3, "FloodStartMsg must be packed");
static_assert(sizeof(FloodDiscoveryMsg) == 5, "FloodDiscoveryMsg must be packed");
static_assert(sizeof(FloodReportMsg) == 6, "FloodReportMsg must be packed");
static_assert(sizeof(FloodHopEntryMsg) == 6, "FloodHopEntryMsg must be packed");
static_assert(sizeof(FloodBaseProbeMsg) == 5, "FloodBaseProbeMsg must be packed");
