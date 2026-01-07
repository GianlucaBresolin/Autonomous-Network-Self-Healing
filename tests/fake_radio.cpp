#include "tests/fake_radio.h"

FakeRadio::FakeRadio(uint8_t i) : id(i) {}

void FakeRadio::setRxCallback(RxCallback cb) {
    rx_cb = cb;
}

void FakeRadio::connect(FakeRadio* a, FakeRadio* b) {
    a->neighbors.push_back(b);
    b->neighbors.push_back(a);
}

void FakeRadio::send(const Packet& pkt) {
    for (auto* n : neighbors) {
        if (pkt.dst == BROADCAST_ID || pkt.dst == n->id) {
            n->receive(pkt);
        }
    }
}

void FakeRadio::receive(const Packet& pkt) {
    if (rx_cb) rx_cb(pkt);
}