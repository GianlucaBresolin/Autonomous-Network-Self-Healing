#include "platform/ns3/custom_mobility/custom_mobility.h"
#include <iostream>
#include <limits>
#include <cmath>

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
    previous_time_s = ns3::Simulator::Now().GetSeconds();
}

void CustomMobility::update(){
    const double now_s = ns3::Simulator::Now().GetSeconds();
    const double delta_t_s = now_s - previous_time_s;
    
    // Skip update if no time has passed
    if (delta_t_s <= 0.0) {
        return;
    }
    
    // update position and velocity based on:
    // - previous position
    // - previous velocity
    // - current acceleration
    // - time elapsed since last update
    double new_x, new_y, new_z;
    
    if (delta_t_max_velocity_s <= 0.0 || delta_t_s <= delta_t_max_velocity_s || std::isinf(delta_t_max_velocity_s)) {
        // Case 1: We have not reached max velocity within this time step
        // or delta_t_max_velocity_s is invalid (no max velocity constraint)
        
        // pos = pos0 + v_prev * delta_t + 0.5 * a * (delta_t)^2
        new_x = previous_position.x + velocity.x * delta_t_s + 0.5 * acceleration.x * std::pow(delta_t_s, 2);
        new_y = previous_position.y + velocity.y * delta_t_s + 0.5 * acceleration.y * std::pow(delta_t_s, 2);
        new_z = previous_position.z + velocity.z * delta_t_s + 0.5 * acceleration.z * std::pow(delta_t_s, 2);
        
        // update velocity: v = v0 + a * delta_t
        velocity.x += acceleration.x * delta_t_s;
        velocity.y += acceleration.y * delta_t_s;
        velocity.z += acceleration.z * delta_t_s;
    } else {
        // Case 2: We reach max velocity during this time step
        const double delta_constant_velocity_s = delta_t_s - delta_t_max_velocity_s;
        
        // First, compute velocity at the transition point (when we hit max velocity)
        const double vx_at_max = velocity.x + acceleration.x * delta_t_max_velocity_s;
        const double vy_at_max = velocity.y + acceleration.y * delta_t_max_velocity_s;
        const double vz_at_max = velocity.z + acceleration.z * delta_t_max_velocity_s;

        // Position update:
        // Phase 1: accelerate from (pos, v) to transition point
        // Phase 2: coast at constant velocity (v_at_max) for remaining time
        new_x = previous_position.x 
                + velocity.x * delta_t_max_velocity_s 
                + 0.5 * acceleration.x * std::pow(delta_t_max_velocity_s, 2) 
                + vx_at_max * delta_constant_velocity_s;
        new_y = previous_position.y 
                + velocity.y * delta_t_max_velocity_s 
                + 0.5 * acceleration.y * std::pow(delta_t_max_velocity_s, 2) 
                + vy_at_max * delta_constant_velocity_s;
        new_z = previous_position.z 
                + velocity.z * delta_t_max_velocity_s 
                + 0.5 * acceleration.z * std::pow(delta_t_max_velocity_s, 2) 
                + vz_at_max * delta_constant_velocity_s;
            
        // Update velocity to the max-velocity-clamped value
        velocity.x = vx_at_max;
        velocity.y = vy_at_max;
        velocity.z = vz_at_max;

        //std::cout << "[CustomMobility] t=" << now_s 
        //          << " reached max velocity: v=(" << velocity.x << "," << velocity.y << "," << velocity.z << ")"
        //          << " |v|=" << velocity.module()
        //          << " delta_t_max_velocity_s=" << delta_t_max_velocity_s << std::endl;
    }
    
    // Clamp velocity magnitude to max_velocity (safety check for floating-point errors)
    if (max_velocity > 0.0 && velocity.module() > max_velocity) {
        velocity = velocity.unit_vector() * max_velocity;
    }

    previous_time_s = now_s;
    previous_position = ns3::Vector(new_x, new_y, new_z);
    mobility->SetPosition(previous_position);
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

void CustomMobility::updateVelocity(const Vector3D new_acceleration, const double new_max_velocity) {
    // First, apply any pending motion from the previous acceleration
    update();

    // Store the new acceleration and max velocity
    acceleration = new_acceleration;
    max_velocity = new_max_velocity;
    delta_t_max_velocity_s = 0.0;
    
    // If no acceleration or no max velocity constraint, skip time-to-max calculation
    if (acceleration.module() == 0.0 || max_velocity <= 0.0) {
        return;
    }
    
    // Check if we're already at or above max velocity
    const double current_speed = velocity.module();
    if (current_speed >= max_velocity) {
        // Already at max velocity - clamp and set delta_t to 0
        velocity = velocity.unit_vector() * max_velocity;
        delta_t_max_velocity_s = 0.0;
        return;
    }
    
    // Calculate time until we reach max velocity
    // We need to solve: |v0 + a*t|^2 = v_max^2
    // Expanding: (v0.x + a.x*t)^2 + (v0.y + a.y*t)^2 + (v0.z + a.z*t)^2 = v_max^2
    // This gives: A*t^2 + B*t + C = 0
    // where:
    //   A = |a|^2
    //   B = 2 * (v0 · a)
    //   C = |v0|^2 - v_max^2
    
    const double A = std::pow(acceleration.x, 2) + std::pow(acceleration.y, 2) + std::pow(acceleration.z, 2);
    const double B = 2.0 * (velocity.x * acceleration.x + velocity.y * acceleration.y + velocity.z * acceleration.z);
    const double C = std::pow(velocity.x, 2) + std::pow(velocity.y, 2) + std::pow(velocity.z, 2) - std::pow(max_velocity, 2);
    
    if (A == 0.0) {
        // Zero acceleration, we'll never reach max velocity through acceleration
        delta_t_max_velocity_s = std::numeric_limits<double>::infinity();
        return;
    }
    
    const double discriminant = B * B - 4.0 * A * C;
    
    if (discriminant < 0.0) {
        // No real solution - acceleration won't bring us to max velocity
        // (this can happen if acceleration is perpendicular to velocity)
        delta_t_max_velocity_s = std::numeric_limits<double>::infinity();
        return;
    }
    
    const double sqrt_discriminant = std::sqrt(discriminant);
    const double t1 = (-B + sqrt_discriminant) / (2.0 * A);
    const double t2 = (-B - sqrt_discriminant) / (2.0 * A);
    
    // We want the smallest positive root
    // The two roots represent when speed increases to max_velocity and when it decreases back
    // (if we're accelerating through the origin in velocity space)
    
    if (t1 >= 0.0 && t2 >= 0.0) {
        // Both roots are positive - take the smaller one
        // This is when we first reach max velocity
        delta_t_max_velocity_s = std::min(t1, t2);
    } else if (t1 >= 0.0) {
        delta_t_max_velocity_s = t1;
    } else if (t2 >= 0.0) {
        delta_t_max_velocity_s = t2;
    } else {
        // Both roots are negative - we've already passed max velocity
        // or we're decelerating away from it
        delta_t_max_velocity_s = std::numeric_limits<double>::infinity();
    }
}