#pragma once

#include <utility>

#include "interfaces/dispatch_manager.h"

class DispatchManager final : public DispatchManagerInterface {
 public:
  DispatchManager() = default;

  void setFloodManager(FloodManagerInterface* flood_manager) override;
  void setNeighborManager(NeighborManagerInterface* neighbor_manager) override;
  void setFallbackHandler(FallbackHandler handler) override;

  void handlePacket(const ::Packet& pkt) const override;
 private:
  FloodManagerInterface* m_flood_manager = nullptr;
  NeighborManagerInterface* m_neighbor_manager = nullptr;
  FallbackHandler m_fallback_handler;
};
