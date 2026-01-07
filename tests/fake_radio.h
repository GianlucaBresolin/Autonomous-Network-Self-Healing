#pragma once

#include <functional>
#include <vector>

#include "interfaces/radio.h"

class FakeRadio : public RadioInterface {
   public:
    using RxCallback = std::function<void(const Packet&)>;

    FakeRadio(uint8_t id);

    void send(const Packet& pkt) override;
    void receive(const Packet& pkt);

    void setRxCallback(RxCallback cb);
    static void connect(FakeRadio* a, FakeRadio* b);

   private:
    uint8_t id;
    RxCallback rx_cb;
    std::vector<FakeRadio*> neighbors;
};