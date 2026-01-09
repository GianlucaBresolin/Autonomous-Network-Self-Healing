#pragma once 

template <typename T> // VelocityCommand
class VelocityActuatorInterface {
    public:
        virtual ~VelocityActuatorInterface() = default;
        virtual void applyVelocityCommand(const T& cmd) = 0;
};