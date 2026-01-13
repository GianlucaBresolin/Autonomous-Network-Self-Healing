#pragma once
#include <cstdint>
#include <atomic>
#include <vector>
#include <cmath>
#include <thread>
#include <memory>
#include "../../common/vector3D.h"
#include "../../interfaces/flooding_manager.h"
#include "../../interfaces/velocity_actuator.h"
#include "../../interfaces/position.h"
#include "../../interfaces/neighbor_manager.h"
#include "../../interfaces/neighbor_info.h"

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
            FloodingManagerInterface* flooding_manager,
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
        
        void computeAttractiveForces(const Vector3D& diff, Vector3D& force);
        void computeRepulsiveForces(const Vector3D& diff, Vector3D& force);
        void computeVelocityCommand(const Vector3D& force, const float V_max, VelocityCommandInterface* cmd);
        void distributedPotentialFieldControlLoop(
            FloodingManagerInterface* flooding_manager,
            VelocityActuatorInterface* velocity_actuator,
            NeighborManagerInterface* neighbor_manager
    );
};