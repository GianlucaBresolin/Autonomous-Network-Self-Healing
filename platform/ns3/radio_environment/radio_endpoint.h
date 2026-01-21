#pragma once

#include "ns3/network-module.h"

namespace sim {

struct RadioEndpoint {
  ::ns3::Ptr<::ns3::NetDevice> device;
  ::ns3::Ipv4Address ip;
  ::ns3::Ptr<::ns3::Socket> socket;
};

}  // namespace sim
