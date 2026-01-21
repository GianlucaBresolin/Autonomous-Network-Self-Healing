
#include "modules/communication/communication_manager.h"

#include <cstring>
#include <utility>

CommunicationManager::CommunicationManager(std::unique_ptr<::Transport> transport, uint8_t self_id)
		: m_transport(std::move(transport)), m_self_id(self_id) {
	if (m_transport) {
		m_transport->SetRxCallback([this](const ::Transport::Bytes& bytes) { handleRxBytes(bytes); });
	}
}

void CommunicationManager::registerPeer(uint8_t id, uint32_t address) {
	if (!m_transport) {
		return;
	}
	m_transport->RegisterPeer(id, address);
}

void CommunicationManager::setReceiveHandler(ReceiveHandler handler) {
	m_on_receive = std::move(handler);
}

void CommunicationManager::send(const ::Packet& pkt) {
	if (!m_transport) {
		return;
	}

	::Transport::Bytes bytes;
	bytes.resize(3 + pkt.payload.size());
	bytes[0] = pkt.src;
	bytes[1] = pkt.dst;
	bytes[2] = static_cast<uint8_t>(pkt.type);
	if (!pkt.payload.empty()) {
		std::memcpy(bytes.data() + 3, pkt.payload.data(), pkt.payload.size());
	}

	if (pkt.dst == BROADCAST_ID) {
		m_transport->SendBroadcast(bytes);
	} else {
		m_transport->SendUnicast(pkt.dst, bytes);
	}
}

void CommunicationManager::receive(::Packet& pkt) {
	if (m_on_receive) {
		m_on_receive(pkt);
	}
}

void CommunicationManager::handleRxBytes(const ::Transport::Bytes& bytes) {
	if (!m_on_receive) {
		return;
	}
	if (bytes.size() < 3) {
		return;
	}

	::Packet decoded;
	decoded.src = bytes[0];
	decoded.dst = bytes[1];
	decoded.type = static_cast<::PacketType>(bytes[2]);
	if (bytes.size() > 3) {
		decoded.payload.assign(bytes.begin() + 3, bytes.end());
	}

	m_on_receive(decoded);
}

