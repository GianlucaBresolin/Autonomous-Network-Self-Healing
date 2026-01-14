
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include "comm/RadioEnvironment.h"
#include "core/BaseStation.h"
#include "core/Drone.h"

using namespace ns3;

namespace {

Ptr<Packet> MakeTextPacket(const std::string& s) {
	return Create<Packet>(reinterpret_cast<const uint8_t*>(s.data()), static_cast<uint32_t>(s.size()));
}

void MoveNode(uint32_t id, Ptr<ConstantPositionMobilityModel> mob, Vector pos) {
	if (!mob) {
		return;
	}
	mob->SetPosition(pos);
	std::cout << "[Mobility] node " << id << " moved to (" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
}

void DroneBroadcast(uint32_t id, Drone* drone, uint32_t seq) {
	if (!drone) {
		return;
	}
	drone->SendBroadcast(MakeTextPacket("DRONE_BCAST id=" + std::to_string(id) + " seq=" + std::to_string(seq)));
}

void BaseUnicast(uint32_t droneId, BaseStation* base, Ipv4Address droneIp, uint32_t seq) {
	if (!base) {
		return;
	}
	base->SendTo(droneIp, MakeTextPacket("BASE_UCAST to=" + std::to_string(droneId) + " seq=" + std::to_string(seq)));
}

void DroneUnicastToBase(uint32_t id, Drone* drone, Ipv4Address baseIp, uint32_t seq) {
	if (!drone) {
		return;
	}
	drone->SendTo(baseIp, MakeTextPacket("DRONE_UCAST id=" + std::to_string(id) + " seq=" + std::to_string(seq)));
}

}  // namespace

int main(int argc, char* argv[]) {
	Time::SetResolution(Time::NS);

	uint32_t nDrones = 4;
	double spacingMeters = 6.0;
	double maxRangeMeters = 30.0;
	double moveTimeSeconds = 3.0;
	double moveOutX = 80.0;
	uint16_t port = 9999;
	double simSeconds = 7.0;

	CommandLine cmd;
	cmd.AddValue("nDrones", "Number of drones in the swarm", nDrones);
	cmd.AddValue("spacingMeters", "Initial spacing along X axis", spacingMeters);
	cmd.AddValue("maxRangeMeters", "Radio max range cutoff", maxRangeMeters);
	cmd.AddValue("moveTimeSeconds", "Time when one drone is moved out of range", moveTimeSeconds);
	cmd.AddValue("moveOutX", "X position to move the last drone to", moveOutX);
	cmd.AddValue("port", "UDP port for unicast/broadcast", port);
	cmd.AddValue("simSeconds", "Simulation stop time", simSeconds);
	cmd.Parse(argc, argv);

	sim::RadioEnvironmentConfig radioCfg;
	radioCfg.maxRangeMeters = maxRangeMeters;
	radioCfg.port = port;
	sim::RadioEnvironment::Get().Configure(radioCfg);

	NodeContainer nodes;
	nodes.Create(nDrones + 1);  // node 0: base station, 1..N: drones

	BaseStation base(0, nodes.Get(0));
	std::vector<std::unique_ptr<Drone>> drones;
	drones.reserve(nDrones);
	for (uint32_t i = 0; i < nDrones; ++i) {
		drones.push_back(std::make_unique<Drone>(i + 1, nodes.Get(i + 1)));
	}

	// Assign initial constant positions.
	{
		auto baseMob = nodes.Get(0)->GetObject<ConstantPositionMobilityModel>();
		if (baseMob) {
			baseMob->SetPosition(Vector(0.0, 0.0, 0.0));
		}
		for (uint32_t i = 0; i < nDrones; ++i) {
			auto mob = nodes.Get(i + 1)->GetObject<ConstantPositionMobilityModel>();
			if (mob) {
				mob->SetPosition(Vector((i + 1) * spacingMeters, 0.0, 0.0));
			}
		}
	}

	// Start periodic traffic.
	// - Each drone broadcasts to the swarm.
	// - Base station unicasts to each drone.
	for (uint32_t i = 0; i < nDrones; ++i) {
		for (uint32_t seq = 0; seq < 8; ++seq) {
			Simulator::Schedule(Seconds(0.5 + 0.4 * seq), &DroneBroadcast, i + 1, drones[i].get(), seq);
		}
	}

	for (uint32_t i = 0; i < nDrones; ++i) {
		for (uint32_t seq = 0; seq < 6; ++seq) {
			Simulator::Schedule(Seconds(1.0 + 0.6 * seq), &BaseUnicast, i + 1, &base, drones[i]->Ip(), seq);
		}
	}

	for (uint32_t i = 0; i < nDrones; ++i) {
		for (uint32_t seq = 0; seq < 6; ++seq) {
			Simulator::Schedule(Seconds(1.3 + 0.6 * seq), &DroneUnicastToBase, i + 1, drones[i].get(), base.Ip(), seq);
		}
	}

	// Move the last drone out of range to demonstrate coverage cutoff.
	if (nDrones > 0) {
		Ptr<ConstantPositionMobilityModel> mob = nodes.Get(nDrones)->GetObject<ConstantPositionMobilityModel>();
		Simulator::Schedule(Seconds(moveTimeSeconds), &MoveNode, nDrones, mob, Vector(moveOutX, 0.0, 0.0));
	}

	Simulator::Stop(Seconds(simSeconds));
	Simulator::Run();
	Simulator::Destroy();
	return 0;
}

