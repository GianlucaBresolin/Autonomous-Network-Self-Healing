#include "platform/ns3/velocity_actuator/ns3_velocity_actuator.h"

Ns3VelocityActuator::Ns3VelocityActuator(
    CustomMobility* mobility
) : 
    mobility(mobility)
{ }

void Ns3VelocityActuator::applyVelocity(
    const Vector3D velocity
) {
    if (!mobility) {
        return;
    }
    mobility->setVelocity(velocity);
}