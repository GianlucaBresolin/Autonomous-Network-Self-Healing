#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>

#include "interfaces/communication_manager.h"
#include "interfaces/neighbor_manager.h"
#include "interfaces/position.h"
#include "modules/neighbor/neighbor_info.h"

class NeighborManager : public NeighborManagerInterface {
 public:
  explicit NeighborManager(CommunicationManagerInterface* communication_manager);

  void onPacketReceived(const ::Packet& pkt) override;
  std::vector<NeighborInfoInterface*> getNeighbors() const override;
  void sendToNeighbors(uint8_t id, PositionInterface* position, uint8_t hops_to_base_station) override;

 private:
  CommunicationManagerInterface* m_communication_manager;
  std::unordered_map<uint8_t, std::unique_ptr<NeighborInfo>> m_neighbors;
};