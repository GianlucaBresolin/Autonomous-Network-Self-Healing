#pragma once

#include <functional>

#include "../../../interfaces/communication_manager.h"
#include "ns3/socket.h"
#include "ns3/ptr.h"

class Ns3CommunicationManager : public CommunicationManagerInterface {
    public:
        Ns3CommunicationManager(ns3::Ptr<ns3::Socket> socket, uint8_t self_id);

        void send(const ::Packet& pkt) override;
        void receive(::Packet& pkt) override;
        void handleReceive(ns3::Ptr<ns3::Socket> socket);

        void setRxCallback(std::function<void(const ::Packet&)> cb);

    private:
        ns3::Ptr<ns3::Socket> socket;
        uint8_t self_id;
        std::function<void(const ::Packet&)> rx_cb;
};