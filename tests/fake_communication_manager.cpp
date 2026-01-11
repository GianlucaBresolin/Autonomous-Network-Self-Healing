#include "tests/fake_communication_manager.h"

FakeCommunicationManager::FakeCommunicationManager(uint8_t i) : id(i) {}

void FakeCommunicationManager::setRxCallback(RxCallback cb) {
    rx_cb = cb;
}

void FakeCommunicationManager::connect(FakeCommunicationManager* a, FakeCommunicationManager* b) {
    a->neighbors.push_back(b);
    b->neighbors.push_back(a);
}

void FakeCommunicationManager::send(const ::Packet& pkt) {
    for (auto* n : neighbors) {
        if (pkt.dst == BROADCAST_ID || pkt.dst == n->id) {
            n->receive(pkt);
        }
    }
}

void FakeCommunicationManager::receive(::Packet& pkt) {
    receive(static_cast<const ::Packet&>(pkt));
}

void FakeCommunicationManager::receive(const ::Packet& pkt) {
    if (rx_cb) {
        rx_cb(pkt);
    }
}