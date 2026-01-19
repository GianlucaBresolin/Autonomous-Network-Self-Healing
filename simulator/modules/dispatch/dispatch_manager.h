#pragma once

#include <functional>
#include <utility>

#include "interfaces/flooding_manager.h"
#include "interfaces/neighbor_manager.h"
#include "modules/flood/messages.h"

class DispatchManager {
  public:
    using FallbackHandler = std::function<void(const ::Packet&)>;

    DispatchManager() = default;

    void SetFloodingManager(FloodingManagerInterface* flooding_manager);
    void SetNeighborManager(NeighborManagerInterface* neighbor_manager);
    void SetFallbackHandler(FallbackHandler handler);

    void HandlePacket(const ::Packet& pkt) const;

  private:
    static bool IsFloodingPacket(const ::Packet& pkt);
    static bool IsNeighborPacket(const ::Packet& pkt);

    FloodingManagerInterface* m_flooding_manager = nullptr;
    NeighborManagerInterface* m_neighbor_manager = nullptr;
    FallbackHandler m_fallback_handler;
};
