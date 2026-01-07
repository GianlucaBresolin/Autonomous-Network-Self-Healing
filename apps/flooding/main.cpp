#include <iostream>

#include "modules/flooding/flood.h"
#include "tests/fake_radio.h"

int main() {
    FakeRadio r0(0), r1(1), r2(2), r3(3), r4(4);

    FakeRadio::connect(&r0, &r1);
    FakeRadio::connect(&r1, &r2);
    FakeRadio::connect(&r2, &r3);
    FakeRadio::connect(&r2, &r4);

    Flooding f0(0, 0, r0);
    Flooding f1(1, 0, r1);
    Flooding f2(2, 0, r2);
    Flooding f3(3, 0, r3);
    Flooding f4(4, 0, r4);

    r0.setRxCallback([&](const Packet& p) { f0.onPacketReceived(p); });
    r1.setRxCallback([&](const Packet& p) { f1.onPacketReceived(p); });
    r2.setRxCallback([&](const Packet& p) { f2.onPacketReceived(p); });
    r3.setRxCallback([&](const Packet& p) { f3.onPacketReceived(p); });
    r4.setRxCallback([&](const Packet& p) { f4.onPacketReceived(p); });
    std::cout << "Starting flood\n";
    f0.startFlood(42);
    f0.startFlood(43);
}