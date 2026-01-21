#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "interfaces/communication_manager.h"
#include "modules/communication/transport.h"

class CommunicationManager : public CommunicationManagerInterface {
    public:
        using ReceiveHandler = std::function<void(const ::Packet&)>;

        CommunicationManager(std::unique_ptr<::Transport> transport, uint8_t self_id);

        // Transport-facing peer registration. For ns-3 transport, `address` is IPv4 as uint32_t.
        void registerPeer(uint8_t id, uint32_t address);
        void setReceiveHandler(ReceiveHandler handler);

        void send(const ::Packet& pkt) override;
        void receive(::Packet& pkt) override;

        private:
        void handleRxBytes(const ::Transport::Bytes& bytes);

        std::unique_ptr<::Transport> m_transport;
        uint8_t m_self_id;
        ReceiveHandler m_on_receive;
};