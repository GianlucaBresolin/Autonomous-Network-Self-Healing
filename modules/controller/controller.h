#pragma once
#include <cstdint>
#include <atomic>
#include <vector>
#include <cmath>
#include <thread>
#include "../../interfaces/radio.h"
#include "../position/position.h"
#include "force.h"
#include "control_message.h"
#include "../velocity/velocity_actuator.h"

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
            RadioInterface& radio,
            VelocityActuator& velocity_actuator
        );
        void stopControlLoop();

    private:
        const uint8_t self_id;
        const float K_att;   
        const float K_rep;
        const float D_safe;
        const float V_max;     
        std::atomic<bool> mission_active{false}; 
        std::vector<NeighborInfo*> neighbors;
        Position current_position;
        uint8_t hops_from_base_station;
        
        void computeAttractiveForces(const float K_att,const NeighborInfo& neighbor, Force& force);
        void computeRepulsiveForces(const float K_rep, const NeighborInfo& neighbor, Force& force);
        void computeVelocityCommand(const Force& force, const float V_max, VelocityCommand& cmd);
        void distributedPotentialFieldControlLoop(RadioInterface& radio, VelocityActuator& velocity_actuator);
};