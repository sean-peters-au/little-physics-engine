/**
 * @file sleep.cpp
 * @brief Implementation of sleep state management system
 */

#include "systems/sleep.hpp"
#include "entities/entity_components.hpp"
#include "core/profile.hpp"

#include <cmath>
#include <iostream>

namespace Systems {

SleepSystem::SleepSystem() {
    // Initialize with default configurations
}

void SleepSystem::update(entt::registry &registry) {
    PROFILE_SCOPE("SleepSystem");

    auto view = registry.view<Components::Velocity, Components::ParticlePhase, Components::Mass, Components::Sleep>();
    for (auto [entity, vel, phase, mass, sleep] : view.each()) {
        if (phase.phase != Components::Phase::Solid && phase.phase != Components::Phase::Liquid && phase.phase != Components::Phase::Gas) {
            continue; // Only sleep typical bodies
        }
        if (registry.all_of<Components::Boundary>(entity)) {
            continue; // Don't sleep boundaries
        }

        // Compute linear speed
        double const speed = std::sqrt(vel.x*vel.x + vel.y*vel.y);

        // Check angular velocity if exists
        double angularSpeed = 0.0;
        if (registry.all_of<Components::AngularVelocity>(entity)) {
            auto &angVel = registry.get<Components::AngularVelocity>(entity);
            angularSpeed = std::fabs(angVel.omega);
        }

        if (speed < specificConfig.velocityThreshold) {
            // Candidate for sleeping
            if (!sleep.asleep) {
                sleep.sleepCounter++;
                if (sleep.sleepCounter > specificConfig.framesBeforeSleep) {
                    sleep.asleep = true;
                }
            }
        } else {
            // Entity is active, reset sleep counter
            sleep.sleepCounter = 0;
            sleep.asleep = false;
        }
        
        // If asleep, set velocity, angular velocity to zero
        if (sleep.asleep) {
            vel.x = 0;
            vel.y = 0;
            
            // Zero out angular velocity if present
            if (registry.all_of<Components::AngularVelocity>(entity)) {
                auto &angVel = registry.get<Components::AngularVelocity>(entity);
                angVel.omega = 0;
            }
        }
    }
}

} // namespace Systems