
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include "modules/core/base_station.h"
#include "modules/core/drone.h"
#include "platform/ns3/radio_environment.h"

using namespace ns3;

namespace {

void MoveNode(uint32_t id, Ptr<ConstantPositionMobilityModel> mob, Vector pos) {
	if (!mob) {
		return;
	}
	mob->SetPosition(pos);
	std::cout << "[Mobility] node " << id << " moved to (" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
}

void RequestCommCheck(BaseStation* base, uint8_t drone_id) {
	if (!base) {
		return;
	}
	base->RequestNodeInfo(drone_id);
	std::cout << "[Test] base requested node info from drone " << int(drone_id) << std::endl;
}

void TriggerFlood(BaseStation* base, uint16_t flood_id) {
	if (!base) {
		return;
	}
	std::cout << "[Test] base starting flood " << flood_id << std::endl;
	base->RequestFlood(flood_id);
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
	double simSeconds = 9.0;

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

	// Register base station with drones and vice versa.
	for (uint32_t i = 0; i < nDrones; ++i) {
		drones[i]->SetBaseStation(0, base.Ip());
		auto mob = nodes.Get(i + 1)->GetObject<ConstantPositionMobilityModel>();
		base.RegisterDrone(static_cast<uint8_t>(i + 1), drones[i]->Ip(), mob);
	}

	// Start node ticks (used for base reachability probes).
	base.Start();
	for (const auto& drone : drones) {
		drone->Start();
	}

	// Communication check: base queries drone positions using CommunicationManager.
	for (uint32_t i = 0; i < nDrones; ++i) {
		Simulator::Schedule(Seconds(0.8 + 0.2 * i), &RequestCommCheck, &base, static_cast<uint8_t>(i + 1));
	}

	// Flooding before movement.
	Simulator::Schedule(Seconds(1.2), &TriggerFlood, &base, 1);

	// Move the last drone out of range to demonstrate coverage cutoff.
	if (nDrones > 0) {
		Ptr<ConstantPositionMobilityModel> mob = nodes.Get(nDrones)->GetObject<ConstantPositionMobilityModel>();
		Simulator::Schedule(Seconds(moveTimeSeconds), &MoveNode, nDrones, mob, Vector(moveOutX, 0.0, 0.0));
	}

	// Flooding after movement to verify coverage change.
	Simulator::Schedule(Seconds(moveTimeSeconds + 1.0), &TriggerFlood, &base, 2);

	Simulator::Stop(Seconds(simSeconds));
	Simulator::Run();

	std::cout << "\n[Test] Final drone state" << std::endl;
	std::cout << std::left
	          << std::setw(6) << "ID"
	          << std::setw(12) << "X"
	          << std::setw(12) << "Y"
	          << std::setw(12) << "Z"
	          << std::setw(12) << "Hops" << std::endl;

	for (uint32_t i = 0; i < nDrones; ++i) {
		Ptr<ConstantPositionMobilityModel> mob = nodes.Get(i + 1)->GetObject<ConstantPositionMobilityModel>();
		Vector pos{0.0, 0.0, 0.0};
		if (mob) {
			pos = mob->GetPosition();
		}
		const uint8_t hops = drones[i]->HopsFromBase();
		std::string hopsText = (hops == UINT8_MAX) ? "N/A" : std::to_string(hops);

		std::cout << std::left
		          << std::setw(6) << (i + 1)
		          << std::setw(12) << pos.x
		          << std::setw(12) << pos.y
		          << std::setw(12) << pos.z
		          << std::setw(12) << hopsText
		          << std::endl;
	}
	Simulator::Destroy();
	return 0;
}
