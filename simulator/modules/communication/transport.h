#pragma once

#include <cstdint>
#include <functional>
#include <vector>

// Transport is the lowest layer used by the simulator's module-level CommunicationManager.
class Transport {
 public:
  using Bytes = std::vector<uint8_t>;
  using RxCallback = std::function<void(const Bytes&)>;

  virtual ~Transport() = default;

  // Optional peer registration.
  // The meaning of `address` is transport-specific (for ns-3 it is an IPv4 address as uint32_t).
  virtual void RegisterPeer(uint8_t id, uint32_t address) = 0;

  virtual void SendUnicast(uint8_t dst_id, const Bytes& bytes) = 0;
  virtual void SendBroadcast(const Bytes& bytes) = 0;

  // The transport calls this for every received datagram payload.
  virtual void SetRxCallback(RxCallback cb) = 0;
};
