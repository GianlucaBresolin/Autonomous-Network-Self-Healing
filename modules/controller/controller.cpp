#include "controller.h"

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
    V_max(V_max > 0.0 ? V_max : DEFAULT_V_MAX)
{ }

Controller::~Controller() {
    stopControlLoop();
}

void Controller::startControlLoop(
    CommunicationManagerInterface* communication_manager,
    VelocityActuatorInterface* velocity_actuator, 
    NeighborManagerInterface* neighbor_manager,
    PositionInterface* initial_position
) {
    current_position = initial_position;
    mission_active.store(true);
    std::thread loop_thread(
        &Controller::distributedPotentialFieldControlLoop, 
        this, 
        communication_manager, 
        velocity_actuator, 
        neighbor_manager
    );
    loop_thread.detach();
}

void Controller::stopControlLoop() {
    mission_active.store(false);
}

void Controller::computeAttractiveForces(const std::unique_ptr<PositionInterface>& diff, Force& force) {
    force = force + diff->multiplyByScalar(K_att);
}

void Controller::computeRepulsiveForces(const std::unique_ptr<PositionInterface>& diff, Force& force) {
    float distance = diff->module();
    if (diff->module() == 0) {
        // Avoid division by zero
        return; 
    }
    force = force - diff->unit_vector()->multiplyByScalar(K_rep / std::pow(distance, 2));
}

void Controller::computeVelocityCommand(const Force& force, const float V_max, VelocityCommandInterface* cmd) {
    // to be implemented
}

void Controller::distributedPotentialFieldControlLoop(
    CommunicationManagerInterface* communication_manager,
    VelocityActuatorInterface* velocity_actuator,
    NeighborManagerInterface* neighbor_manager
) {
    while(mission_active.load()) {
        neighbors = neighbor_manager->getNeighbors();
        current_position->retrieveCurrentPosition();
        // hops_from_base_station = getHopsFromBaseStation(); // to be implemented

        Force F_tot = Force{0.0f, 0.0f, 0.0f};
        for(const NeighborInfoInterface* neighbor: neighbors) {
            const uint8_t neighbor_hops = neighbor->getHopsToBaseStation();
            std::unique_ptr<PositionInterface> diff = current_position->distanceFrom(neighbor->getPosition());
            if (neighbor_hops < hops_from_base_station || neighbor_hops > hops_from_base_station) {
                // Attractive force 
                computeAttractiveForces(diff, F_tot);
            }
            if (diff->module() < D_safe) {
                // Repulsive force
                computeRepulsiveForces(diff, F_tot);
            }
        }

        // Velocity Command
        VelocityCommandInterface* velocity_command;
        computeVelocityCommand(F_tot, V_max, velocity_command);
        velocity_actuator->applyVelocityCommand(velocity_command);

        // Broadcast Control Message
        // ControlMessage msg = ControlMessage{
        //     .id = self_id,
        //     .position = current_position,
        //     .hops_from_base_station = hops_from_base_station
        // };
        // Packet control_pkt;
        // control_pkt.src = self_id;
        // control_pkt.dst = BROADCAST_ID;
        // std::memcpy(control_pkt.payload.data(), &msg, sizeof(msg));
        // communication_manager->send(control_pkt);
    }
}