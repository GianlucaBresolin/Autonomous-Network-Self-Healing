#pragma once
#include "../../interfaces/velocity_actuator.h"
#include "velocity_command.h"

class VelocityActuator: public VelocityActuatorInterface {
    public:
        VelocityActuator();
        ~VelocityActuator() = default;

        void applyVelocityCommand(const VelocityCommandInterface* cmd) override;
};