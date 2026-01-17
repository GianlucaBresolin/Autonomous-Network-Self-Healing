#pragma once

#include <cstdint>
#include <string>

namespace sim {

struct RadioEnvironmentConfig {
  // Simple “coverage area” model: beyond this distance, frames are not received.
  double maxRangeMeters = 30.0;

  // IPv4 subnet used for all nodes that install this radio.
  std::string networkBase = "10.1.1.0";
  std::string networkMask = "255.255.255.0";

  // UDP port used for both unicast and broadcast.
  uint16_t port = 9999;
};

}  // namespace sim
