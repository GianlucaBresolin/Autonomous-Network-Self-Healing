#pragma once 
#include "velocity_command.h"

class VelocityActuatorInterface {
    public:
        virtual ~VelocityActuatorInterface() = default;
        virtual void applyVelocityCommand(const VelocityCommandInterface* cmd) = 0;
};