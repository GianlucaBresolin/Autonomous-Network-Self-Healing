#include "controller.h"
#include <thread>

Controller::Controller(
    uint8_t self_id,
    float K_att_value,
    float K_rep_value,
    float D_safe,
    float V_max
) :
    self_id(self_id),
    K_att(K_att_value > 0.0 ? K_att_value : DEFAULT_K_ATT),
    K_rep(K_rep_value > 0.0 ? K_rep_value : DEFAULT_K_REP),
    D_safe(D_safe > 0.0 ? D_safe : DEFAULT_D_SAFE),
    V_max(V_max > 0.0 ? V_max : DEFAULT_V_MAX),
    current_position(0.0f, 0.0f, 0.0f)
{ }

Controller::~Controller() {
    stopControlLoop();
}

void Controller::startControlLoop(
    RadioInterface& radio
) {
    mission_active.store(true);
    std::thread loop_thread(
        &Controller::distributedPotentialFieldControlLoop, 
        this, 
        std::ref(radio)
    );
    loop_thread.detach();
}

void Controller::stopControlLoop() {
    mission_active.store(false);
}

void Controller::computeAttractiveForces(const float K_att,const NeighborInfo& neighbor, Force& force) {
    force = force + K_att * (neighbor.position - current_position);
}

void Controller::computeRepulsiveForces(const float K_rep, const NeighborInfo& neighbor, Force& force) {
    Position diff = neighbor.position - current_position;
    force = force - K_rep * (1 / std::pow(diff.module(), 2)) * (diff.unit_vector());
}

void Controller::computeVelocityCommand(const Force& force, const float V_max) {
    // to be implemented
}

void Controller::distributedPotentialFieldControlLoop(RadioInterface& radio) {
    while(mission_active.load()) {
        neighbors = radio.getNeighbors();
        current_position.retrieveCurrentPosition();
        // hops_from_base_station = getHopsFromBaseStation(); // to be implemented

        Force F_tot = Force{0.0f, 0.0f, 0.0f};
        for(const NeighborInfo* neighbor: neighbors) {
            if (neighbor->hops_from_base_station < hops_from_base_station || neighbor->hops_from_base_station > hops_from_base_station)
                // Attractive force 
                computeAttractiveForces(K_att, *neighbor, F_tot);
            if ((neighbor->position - current_position).module() < D_safe)
                // Repulsive force
                computeRepulsiveForces(K_rep, *neighbor, F_tot);
        }

        // velocity_command = computeVelocityCommand(F_tot, V_max); // to be implemented
        // apply(velocity_command); // to be implemented

        ControlMessage msg = ControlMessage{
            .id = self_id,
            .position = current_position,
            .hops_from_base_station = hops_from_base_station
        };
        Packet control_pkt;
        control_pkt.src = self_id;
        control_pkt.dst = BROADCAST_ID;
        std::memcpy(control_pkt.payload.data(), &msg, sizeof(msg));
        radio.send(control_pkt);
    }
}