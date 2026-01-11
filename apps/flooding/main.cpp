#include <cstring>
#include <iostream>

#include "modules/flooding/flood.h"
#include "tests/fake_communication_manager.h"

int main() {
    FakeCommunicationManager base(100);
    FakeCommunicationManager r0(0), r1(1), r2(2), r3(3), r4(4);

    FakeCommunicationManager::connect(&r0, &r1);
    FakeCommunicationManager::connect(&r1, &r2);
    FakeCommunicationManager::connect(&r2, &r3);
    FakeCommunicationManager::connect(&r2, &r4);

    // Base can directly ask any drone to initiate a flood.
    FakeCommunicationManager::connect(&base, &r2);

    Flooding f0(0, r0);
    Flooding f1(1, r1);
    Flooding f2(2, r2);
    Flooding f3(3, r3);
    Flooding f4(4, r4);

    r0.setRxCallback([&](const Packet& p) { f0.onPacketReceived(p); });
    r1.setRxCallback([&](const Packet& p) { f1.onPacketReceived(p); });
    r2.setRxCallback([&](const Packet& p) { f2.onPacketReceived(p); });
    r3.setRxCallback([&](const Packet& p) { f3.onPacketReceived(p); });
    r4.setRxCallback([&](const Packet& p) { f4.onPacketReceived(p); });

    std::cout << "Base requesting flood from node 2\n";
    FloodStartMsg start;
    start.flood_id = 42;
    ::Packet start_pkt;
    start_pkt.src = 100;
    start_pkt.dst = 2;
    start_pkt.payload.resize(sizeof(start));
    std::memcpy(start_pkt.payload.data(), &start, sizeof(start));
    base.send(start_pkt);
}