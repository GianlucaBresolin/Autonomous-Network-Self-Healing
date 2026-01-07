#include "modules/flooding/flood.h"

#include <cstring>
#include <iostream>

Flooding::Flooding(uint8_t self, uint8_t base, RadioInterface& r)
    : self_id(self), base_id(base), radio(r) {}

void Flooding::startFlood(uint16_t flood_id) {
    FloodDiscoveryMsg msg;
    msg.flood_id = flood_id;
    msg.origin = self_id;

    Packet pkt;
    pkt.src = self_id;
    pkt.dst = BROADCAST_ID;
    pkt.payload.resize(sizeof(msg));
    std::memcpy(pkt.payload.data(), &msg, sizeof(msg));

    radio.send(pkt);
}

void Flooding::onPacketReceived(const Packet& pkt) {
    if (pkt.payload.empty()) return;

    auto type = static_cast<FloodMsgType>(pkt.payload[0]);

    if (type == FloodMsgType::DISCOVERY) {
        FloodDiscoveryMsg msg;
        std::memcpy(&msg, pkt.payload.data(), sizeof(msg));
        handleDiscovery(msg);
    }
}

void Flooding::handleDiscovery(const FloodDiscoveryMsg& msg) {
    if (seen_floods.count(msg.flood_id)) return;

    seen_floods.insert(msg.flood_id);

    // Send reply to base
    FloodReplyMsg reply;
    reply.flood_id = msg.flood_id;
    reply.responder = self_id;

    Packet reply_pkt;
    reply_pkt.src = self_id;
    reply_pkt.dst = base_id;
    reply_pkt.payload.resize(sizeof(reply));
    std::memcpy(reply_pkt.payload.data(), &reply, sizeof(reply));

    radio.send(reply_pkt);

    // Rebroadcast discovery
    Packet flood_pkt;
    flood_pkt.src = self_id;
    flood_pkt.dst = BROADCAST_ID;
    flood_pkt.payload.resize(sizeof(msg));
    std::memcpy(flood_pkt.payload.data(), &msg, sizeof(msg));

    radio.send(flood_pkt);

    std::cout << "[Node " << int(self_id) << "] discovered flood " << msg.flood_id
              << " from origin " << int(msg.origin) << std::endl;

    std::cout << "[Node " << int(self_id) << "] replying to base " << int(base_id) << std::endl;
}