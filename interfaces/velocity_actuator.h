#pragma once 

#include "common/vector3D.h"

class VelocityActuatorInterface {
    public:
        virtual ~VelocityActuatorInterface() = default;
        virtual void applyVelocity(const Vector3D velocity) = 0;
};