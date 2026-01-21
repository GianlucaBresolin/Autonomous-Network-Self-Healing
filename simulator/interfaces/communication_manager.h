#pragma once

#include "common/packet.h"

class CommunicationManagerInterface {
    public:
        virtual ~CommunicationManagerInterface() = default;
        virtual void send(const ::Packet& pkt) = 0;
        virtual void receive(::Packet& pkt) = 0;
};