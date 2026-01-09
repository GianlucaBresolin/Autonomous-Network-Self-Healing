#pragma once 

struct VelocityCommand {
    // to be defined
};

class VelocityActuatorInterface {
    public:
        virtual ~VelocityActuatorInterface() = default;
        virtual void applyVelocityCommand(const VelocityCommand& cmd) = 0;
};