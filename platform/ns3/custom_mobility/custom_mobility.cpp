#include "platform/ns3/custom_mobility/custom_mobility.h"

CustomMobility::CustomMobility(
    ns3::Ptr<ns3::ConstantPositionMobilityModel> mobility
) : 
    mobility(mobility),
    previous_time_s(ns3::Simulator::Now().GetSeconds()),
    acceleration(0.0, 0.0, 0.0), // no initial acceleration
    velocity(0.0, 0.0, 0.0)  // no initial velocity
{
    if (this->mobility) {
        previous_position = this->mobility->GetPosition();
    }
}

void CustomMobility::setPosition(
    const double x,
    const double y, 
    const double z
) { 
    if (!mobility) {
        return;
    }

    mobility->SetPosition(ns3::Vector(x, y, z));
    previous_position = mobility->GetPosition();
    previous_time_s = clock();
}

void CustomMobility::update(){
    const clock_t now_s = clock();
    const double delta_t_s = (now_s - previous_time_s) / static_cast<double>(CLOCKS_PER_SEC);
    
    // update position and velocity based on:
    // - previous position
    // - previous velocity
    // - current acceleration
    // - time elapsed since last update
    double new_x, new_y, new_z;
    if (delta_t_s <= delta_t_max_velocity_s) {
        // we do not have reached max velocity
        // pos = pos0 + v_prev * delta_t + 0.5 * a * (delta_t)^2
        new_x = previous_position.x + velocity.x * delta_t_s + 0.5 * acceleration.x * std::pow(delta_t_s, 2);
        new_y = previous_position.y + velocity.y * delta_t_s + 0.5 * acceleration.y * std::pow(delta_t_s, 2);
        new_z = previous_position.z + velocity.z * delta_t_s + 0.5 * acceleration.z * std::pow(delta_t_s, 2);
        
        // update velocity: v = v0 + a * delta_t
        velocity.x += acceleration.x * delta_t_s;
        velocity.y += acceleration.y * delta_t_s;
        velocity.z += acceleration.z * delta_t_s;
    } else {
        // we have reached max velocity
        if (acceleration.x < 0) {
            new_x = previous_position.x + velocity.x * delta_t_s + 0.5 * acceleration.x * std::pow(delta_t_s, 2);

            velocity.x += acceleration.x * delta_t_s;
        } else {
            double delta_constant_velocity_s = delta_t_s - delta_t_max_velocity_s;
            // pos = pos0 + v_prev * delta_t_max_velocity + 0.5 * a * (delta_t_max_velocity)^2 + v_max * delta_constant_velocity
            new_x = previous_position.x + velocity.x * delta_t_max_velocity_s + 0.5 * acceleration.x * std::pow(delta_t_max_velocity_s, 2) + velocity.x * delta_constant_velocity_s;
            new_y = previous_position.y + velocity.y * delta_t_max_velocity_s + 0.5 * acceleration.y * std::pow(delta_t_max_velocity_s, 2) + velocity.y * delta_constant_velocity_s;
            new_z = previous_position.z + velocity.z * delta_t_max_velocity_s + 0.5 * acceleration.z * std::pow(delta_t_max_velocity_s, 2) + velocity.z * delta_constant_velocity_s;
            
            // update velocity: v = v0 + a * delta_t_max_velocity (its magnitude is
            // max_velocity)
            velocity.x += acceleration.x * delta_t_max_velocity_s;
            velocity.y += acceleration.y * delta_t_max_velocity_s;
            velocity.z += acceleration.z * delta_t_max_velocity_s;
        }
    }

    previous_time_s = now_s;
    mobility->SetPosition(ns3::Vector(new_x, new_y, new_z));
}

std::vector<double> CustomMobility::getPosition() {
    if (!mobility) {
        return {0.0, 0.0, 0.0};
    }
    
    // update position and velocity
    // update(); 

    // retrieve current position
    previous_position = mobility->GetPosition();
    return {
        previous_position.x, 
        previous_position.y, 
        previous_position.z
    };
}

void CustomMobility::updateVelocity(const Vector3D new_acceleration, const double max_velocity) {
    // update position and velocity
    update();

    // set new acceleration 
    acceleration = new_acceleration;

    if (acceleration.module() != 1e-6) {
        // set new delta_time_max_velocity_s = (max_velocity - current_velocity) / acceleration
        delta_t_max_velocity_s = (max_velocity - velocity.module()) / acceleration.module();
    } else {
        delta_t_max_velocity_s = 0.0;
    }
}