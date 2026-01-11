#pragma once
#include <cstdint>
#include <atomic>
#include <vector>
#include <cmath>
#include <thread>
#include "../../interfaces/communication_manager.h"
#include "../../interfaces/velocity_actuator.h"
#include "../../interfaces/position.h"
#include "../../interfaces/neighbor_manager.h"
#include "../../interfaces/neighbor_info.h"
#include "force.h"
#include "control_message.h"

const float DEFAULT_K_ATT = 1;
const float DEFAULT_K_REP = 1;
const float DEFAULT_D_SAFE = 1;
const float DEFAULT_V_MAX = 1;

class Controller {
    public:
        Controller(
            uint8_t self_id,
            float K_att_value,
            float K_rep_value,
            float D_safe,
            float V_max
        );
        ~Controller();
        void startControlLoop(
            CommunicationManagerInterface* communication_manager,
            VelocityActuatorInterface* velocity_actuator,
            NeighborManagerInterface* neighbor_manager, 
            PositionInterface* initial_position
        );
        void stopControlLoop();

    private:
        const uint8_t self_id;
        const float K_att;   
        const float K_rep;
        const float D_safe;
        const float V_max;     
        std::atomic<bool> mission_active{false}; 
        std::vector<NeighborInfoInterface*> neighbors;
        PositionInterface* current_position;
        uint8_t hops_from_base_station;
        
        void computeAttractiveForces(const std::unique_ptr<PositionInterface>& diff, Force& force);
        void computeRepulsiveForces(const std::unique_ptr<PositionInterface>& diff, Force& force);
        void computeVelocityCommand(const Force& force, const float V_max, VelocityCommandInterface* cmd);
        void distributedPotentialFieldControlLoop(
            CommunicationManagerInterface* communication_manager, 
            VelocityActuatorInterface* velocity_actuator,
            NeighborManagerInterface* neighbor_manager
    );
};