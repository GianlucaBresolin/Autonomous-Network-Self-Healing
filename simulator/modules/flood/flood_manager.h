#pragma once

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "interfaces/flood_manager.h"
#include "modules/flood/flood_messages.h"

class FloodManager : public FloodManagerInterface {
    public:
        FloodManager(
            uint8_t self_id,
            CommunicationManagerInterface& communication_manager,
            std::function<bool()> is_base_reachable = {}
        );

        void onPacketReceived(const ::Packet& pkt) override;

        void setBaseId(uint8_t base_id);

        // Returns the number of hops from this node to the base station.
        uint8_t getHopsFromBase() const override;

    private:
        uint8_t base_id = 0;
        uint8_t self_id;
        CommunicationManagerInterface& communication_manager;
        std::function<bool()> is_base_reachable;

        std::unordered_map<uint16_t, uint8_t> best_hop_to_base;

        // For multi-hop reporting: track the best hop_to_initiator we've seen per reporter
        // so we forward each reporter's report at most once per improvement.
        std::unordered_map<uint16_t, std::unordered_map<uint8_t, uint8_t>> best_report_seen;

        std::unordered_set<uint16_t> seen_floods;

        void startFlood(uint16_t flood_id) override;

        void handleStart(const FloodStartMsg& flood_id);
        void handleDiscovery(const FloodDiscoveryMsg& msg);
        void handleReport(const FloodReportMsg& msg);

        ::Packet createReportMsg(uint16_t flood_id, uint8_t initiator_id, uint8_t candidate_hop);
        ::Packet createDiscoveryMsg(uint16_t flood_id, uint8_t initiator_id, uint8_t hop_to_base);

        static bool decodeStart(const ::Packet& pkt, FloodStartMsg& msg);
        static bool decodeDiscovery(const ::Packet& pkt, FloodDiscoveryMsg& msg);
        static bool decodeReport(const ::Packet& pkt, FloodReportMsg& msg);
};
