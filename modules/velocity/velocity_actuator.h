#pragma once
#include "../../interfaces/velocity_actuator.h"
#include "velocity_command.h"

class VelocityActuator: public VelocityActuatorInterface<VelocityCommand> {
    public:
        VelocityActuator();
        ~VelocityActuator() = default;

        void applyVelocityCommand(const VelocityCommand& cmd) override;
};