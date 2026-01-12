#include "flood.h"

#include <cstring>
#include <iostream>

Flooding::Flooding(
    uint8_t self,
    CommunicationManagerInterface& cm
) :
    self_id(self),
    communication_manager(cm)
{ }

void Flooding::onPacketReceived(const ::Packet& pkt) {
    if (pkt.payload.empty()) {
        return;
    }

    auto type = static_cast<FloodMsgType>(pkt.payload[0]);

    switch(type) {
        case FloodMsgType::START: {
            FloodStartMsg msg;
            if (decodeStart(pkt, msg)) {
                handleStart(msg);
            }
            break;
        }
        case FloodMsgType::DISCOVERY: {
            FloodDiscoveryMsg msg; 
            if (decodeDiscovery(pkt, msg)) {
                handleDiscovery(msg);
            }
            break;
        }
        case FloodMsgType::REPORT: {
            FloodReportMsg msg;
            if (decodeReport(pkt, msg)) {
                handleReport(msg);
            }    
        }
        default:
            // unrecognized message type
            return;
    }
}

std::vector<std::pair<uint8_t, uint8_t>> Flooding::getHopTableToBase(uint16_t flood_id) const {
    std::vector<std::pair<uint8_t, uint8_t>> out;
    auto it = hop_table_to_initiator.find(flood_id);
    if (it == hop_table_to_initiator.end()) {
        return out;
    }

    out.reserve(it->second.size());
    for (const auto& kv : it->second) {
        const uint8_t node_id = kv.first;
        const uint8_t hop_to_initiator = kv.second;
        const uint8_t hop_to_base = static_cast<uint8_t>(hop_to_initiator + 1);
        out.emplace_back(node_id, hop_to_base);
    }
    return out;
}

uint8_t Flooding::getHopsFromBase() const {
    // higher flood_id means more recent flood
    uint16_t latest_flood_id = 0;
    for( auto [flood_id, hops_to_initiator] : best_hop_to_initiator ) {
        latest_flood_id = (latest_flood_id < flood_id) ? flood_id : latest_flood_id;
    }
    auto it = best_hop_to_initiator.find(latest_flood_id);
    if (it == best_hop_to_initiator.end()) {
        // no floods seen yet
        return UINT8_MAX;
    }
    return static_cast<uint8_t>(it->second + 1);
}

void Flooding::startFlood(uint16_t flood_id) {
    // Initiator seeds the flood.
    FloodDiscoveryMsg msg;
    msg.flood_id = flood_id;
    msg.initiator_id = self_id;
    msg.hop_to_initiator = 0;

    seen_floods.insert(flood_id);
    best_hop_to_initiator[flood_id] = 0;

    // Initiator already knows itself at hop 0.
    hop_table_to_initiator[flood_id][self_id] = 0;

    ::Packet pkt;
    pkt.src = self_id;
    pkt.dst = BROADCAST_ID;
    pkt.payload.resize(sizeof(msg));
    std::memcpy(pkt.payload.data(), &msg, sizeof(msg));

    communication_manager.send(pkt);
}

void Flooding::handleStart(const FloodStartMsg& msg) {
    // Base requests this node to act as initiator.
    // Avoid restarting the same flood multiple times.
    if (seen_floods.count(msg.flood_id)) {
        return;
    }
    startFlood(msg.flood_id);
}

void Flooding::handleDiscovery(const FloodDiscoveryMsg& msg) {
    const uint16_t flood_id = msg.flood_id;
    const uint8_t initiator_id = msg.initiator_id;

    // Compute candidate hop to initiator.
    const uint8_t candidate_hop = static_cast<uint8_t>(msg.hop_to_initiator + 1);

    bool improved = false;
    auto it = best_hop_to_initiator.find(flood_id);
    if (it == best_hop_to_initiator.end()) {
        improved = true;
        best_hop_to_initiator[flood_id] = candidate_hop;
        seen_floods.insert(flood_id);
    } else if (candidate_hop < it->second) {
        improved = true;
        it->second = candidate_hop;
    }

    if (!improved) {
        return;
    }

    // Broadcast a report with the best hop we currently know.
    FloodReportMsg report;
    report.flood_id = flood_id;
    report.initiator_id = initiator_id;
    report.reporter_id = self_id;
    report.hop_to_initiator = candidate_hop;

    // Mark our own report as seen so we don't forward an echoed copy later.
    best_report_seen[flood_id][self_id] = candidate_hop;

    ::Packet report_pkt;
    report_pkt.src = self_id;
    report_pkt.dst = BROADCAST_ID;
    report_pkt.payload.resize(sizeof(report));
    std::memcpy(report_pkt.payload.data(), &report, sizeof(report));
    communication_manager.send(report_pkt);

    // Rebroadcast discovery with incremented hop.
    FloodDiscoveryMsg fwd = msg;
    fwd.hop_to_initiator = candidate_hop;

    ::Packet flood_pkt;
    flood_pkt.src = self_id;
    flood_pkt.dst = BROADCAST_ID;
    flood_pkt.payload.resize(sizeof(fwd));
    std::memcpy(flood_pkt.payload.data(), &fwd, sizeof(fwd));
    communication_manager.send(flood_pkt);

    std::cout << "[Node " << int(self_id) << "] discovered flood " << flood_id
              << " from initiator " << int(initiator_id)
              << " at hop " << int(candidate_hop) << std::endl;
}

void Flooding::handleReport(const FloodReportMsg& msg) {
    // Ignore reports for floods we never joined (limits propagation scope).
    if (!seen_floods.count(msg.flood_id)) {
        return;
    }

    // Forward each reporter's best-known report at most once per improvement.
    bool improved = false;
    auto& seen = best_report_seen[msg.flood_id];
    auto it_seen = seen.find(msg.reporter_id);
    if (it_seen == seen.end() || msg.hop_to_initiator < it_seen->second) {
        improved = true;
        seen[msg.reporter_id] = msg.hop_to_initiator;
    }

    if (!improved) {
        return;
    }

    // The initiator aggregates reports for its own flood.
    if (msg.initiator_id == self_id) {
        auto& table = hop_table_to_initiator[msg.flood_id];
        auto it = table.find(msg.reporter_id);
        if (it == table.end() || msg.hop_to_initiator < it->second) {
            table[msg.reporter_id] = msg.hop_to_initiator;
        }
        return;
    }

    // Non-initiators rebroadcast reports so they can reach the initiator over multiple hops.
    ::Packet pkt;
    pkt.src = self_id;
    pkt.dst = BROADCAST_ID;
    pkt.payload.resize(sizeof(msg));
    std::memcpy(pkt.payload.data(), &msg, sizeof(msg));
    communication_manager.send(pkt);
}

bool Flooding::decodeStart(const ::Packet& pkt, FloodStartMsg& msg) {
    if (pkt.payload.size() < sizeof(FloodStartMsg)) {
        return false;
    }
    std::memcpy(&msg, pkt.payload.data(), sizeof(FloodStartMsg));
    return msg.type == FloodMsgType::START;
}

bool Flooding::decodeDiscovery(const ::Packet& pkt, FloodDiscoveryMsg& msg) {
    if (pkt.payload.size() < sizeof(FloodDiscoveryMsg)) {
        return false;
    }
    std::memcpy(&msg, pkt.payload.data(), sizeof(FloodDiscoveryMsg));
    return msg.type == FloodMsgType::DISCOVERY;
}

bool Flooding::decodeReport(const ::Packet& pkt, FloodReportMsg& msg) {
    if (pkt.payload.size() < sizeof(FloodReportMsg)) {
        return false;
    }
    std::memcpy(&msg, pkt.payload.data(), sizeof(FloodReportMsg));
    return msg.type == FloodMsgType::REPORT;
}