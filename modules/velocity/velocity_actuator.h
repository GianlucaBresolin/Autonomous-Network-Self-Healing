#pragma once
#include "../../interfaces/velocity_actuator_interface.h"

class VelocityActuator: public VelocityActuatorInterface {
    public:
        VelocityActuator();
        ~VelocityActuator() = default;

        void applyVelocityCommand(const VelocityCommand& cmd);
};