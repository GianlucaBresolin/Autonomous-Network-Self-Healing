#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/netanim-module.h"

#include "platform/ns3/base_station/ns3_base_station.h"
#include "platform/ns3/drone/ns3_drone.h"
#include "platform/ns3/radio_environment/radio_environment.h"

using namespace ns3;

namespace {

void EnsureMobility(Ptr<Node> node, const Vector& pos) {
  auto mob = node->GetObject<ConstantPositionMobilityModel>();
  if (!mob) {
    mob = CreateObject<ConstantPositionMobilityModel>();
    node->AggregateObject(mob);
  }
  mob->SetPosition(pos);
}

}  // namespace

int main(int argc, char* argv[]) {
  Time::SetResolution(Time::NS);

  // Requirements:
  // - 1 base station + 4 drones
  // - base station coverage range: 50m
  // - drones move by their own behavior (idle velocity) until one leaves coverage
  // - leaving coverage -> missing POS_ACK -> drone sends HELP_PROXY
  // - HELP_PROXY triggers mission_active + repositioning behavior inside drones

  double maxRangeMeters = 50.0;
  uint16_t port = 9999;
  double simSeconds = 30.0;

  CommandLine cmd;
  cmd.AddValue("maxRangeMeters", "Radio max range cutoff (coverage)", maxRangeMeters);
  cmd.AddValue("port", "UDP port for unicast/broadcast", port);
  cmd.AddValue("simSeconds", "Simulation stop time", simSeconds);
  cmd.Parse(argc, argv);

  sim::RadioEnvironmentConfig radioCfg;
  radioCfg.maxRangeMeters = maxRangeMeters;
  radioCfg.port = port;
  sim::RadioEnvironment::Get().Configure(radioCfg);

  NodeContainer nodes;
  nodes.Create(1 + 4);  // node 0: base, 1..4: drones

  // Place drone 3 initially outside base coverage so it will timeout and emit HELP_PROXY.
  // Keep it within range of at least one other drone (drone 4) so the HELP_PROXY can be received,
  // while keeping it far enough from the base (>= 50m) that it doesn't re-enter immediately.
  EnsureMobility(nodes.Get(0), Vector(0.0, 0.0, 0.0));
  EnsureMobility(nodes.Get(1), Vector(15.0, 0.0, 0.0));
  EnsureMobility(nodes.Get(2), Vector(25.0, 0.0, 0.0));
  EnsureMobility(nodes.Get(3), Vector(55.0, 0.0, 0.0));
  EnsureMobility(nodes.Get(4), Vector(45.0, 0.0, 0.0));

  Ns3BaseStation base(0, nodes.Get(0));
  base.setPosition(0.0, 0.0, 0.0);

  std::vector<std::unique_ptr<Ns3Drone>> drones;
  drones.reserve(4);
  for (uint32_t i = 0; i < 4; ++i) {
    drones.push_back(std::make_unique<Ns3Drone>(static_cast<uint8_t>(i + 1), nodes.Get(i + 1)));
  }

  // Register peers (no mission forcing here; just wiring ids/ips).
  for (uint32_t i = 0; i < 4; ++i) {
    drones[i]->setBaseStation(base.id(), base.ip(), base.position());
    base.registerDrone(drones[i]->id(), drones[i]->ip());
  }

  // Base station triggers floods periodically (unicast START to initiator).
  base.start();

  // Start drones' periodic ticks (POS_UPDATE/ACK tracking + idle motion + HELP_PROXY timeout).
  for (const auto& d : drones) {
    d->start();
  }

  std::cout << "[Sim] base coverage=" << maxRangeMeters << "m, drones=4, stop=" << simSeconds << "s" << std::endl;

  AnimationInterface anim("/output/drone-simulation.xml");
  Simulator::Stop(Seconds(simSeconds));
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}
