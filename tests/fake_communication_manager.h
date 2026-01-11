#pragma once

#include <functional>
#include <vector>

#include "interfaces/communication_manager.h"

class FakeCommunicationManager : public CommunicationManagerInterface {
    public:
        using RxCallback = std::function<void(const ::Packet&)>;

        FakeCommunicationManager(uint8_t id);

        void send(const ::Packet& pkt) override;
        void receive(::Packet& pkt) override;
        void receive(const ::Packet& pkt);

        void setRxCallback(RxCallback cb);
        static void connect(FakeCommunicationManager* a, FakeCommunicationManager* b);

    private:
        uint8_t id;
        RxCallback rx_cb;
        std::vector<FakeCommunicationManager*> neighbors;
};