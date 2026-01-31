#pragma once
#include <cstdint>
#include <cmath>
#include <vector>

#include "common/vector3D.h"

#include "interfaces/flood_manager.h"
#include "interfaces/velocity_actuator.h"
#include "interfaces/position.h"
#include "interfaces/neighbor_manager.h"
#include "interfaces/neighbor_info.h"

const float DEFAULT_K_ATT = 1;
const float DEFAULT_K_REP = 1;
const float DEFAULT_D_SAFE = 1;
const float DEFAULT_V_MAX = 1;
const float DEFAULT_DRONE_WEIGHT_KG = 2.5;

class Controller {
    public:
        Controller(
            uint8_t self_id,
            float K_att_value,
            float K_rep_value,
            float D_safe,
            float V_max, 
            float drone_weight_kg = 2.5f
        );

        void setMissionActive(bool active);
        bool isMissionActive() const;

        void setHovering(bool hovering);
        bool isHovering() const;

        void setIdleVelocity(const Vector3D& velocity);
        void step(
            FloodManagerInterface* flooding_manager,
            VelocityActuatorInterface* velocity_actuator,
            NeighborManagerInterface* neighbor_manager,
            PositionInterface* position
        );

    private:
        const uint8_t self_id;
        const float K_att;   
        const float K_rep;
        const float D_safe;
        const float V_max;     
        const float drone_weight_kg;
        bool mission_active = false;
        bool hovering = false;  // When true, drone decelerates to stop and hovers
        Vector3D idle_velocity{0.5f, 0.0f, 0.0f};
        float min_difference = 100;
        
        void computeAttractiveForces(const Vector3D& diff, Vector3D& force);
        void computeRepulsiveForces(const Vector3D& diff, Vector3D& force);
        void computeVelocityCommand(const Vector3D& force, Vector3D* new_accelerations);
};