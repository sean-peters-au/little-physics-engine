#include "systems/sleep.hpp"
#include "entities/entity_components.hpp"
#include "core/profile.hpp"

#include <cmath>
#include <iostream>

namespace Systems {

void SleepSystem::update(entt::registry &registry) {
    PROFILE_SCOPE("SleepSystem");

    // View of all entities that might sleep
    auto view = registry.view<
        Components::Velocity,
        Components::ParticlePhase,
        Components::Mass,
        Components::Sleep
    >();
    for (auto [entity, vel, phase, mass, sleep] : view.each()) {
        // Skip if phase is not a typical body
        if (phase.phase != Components::Phase::Solid
            && phase.phase != Components::Phase::Liquid
            && phase.phase != Components::Phase::Gas)
        {
            continue;
        }
        // Skip boundaries
        if (registry.any_of<Components::Boundary>(entity)) {
            continue;
        }

        // Compute linear speed
        double const speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);

        // Check angular velocity if the entity has it
        double angularSpeed = 0.0;
        if (registry.any_of<Components::AngularVelocity>(entity)) {
            auto &angVel = registry.get<Components::AngularVelocity>(entity);
            angularSpeed = std::fabs(angVel.omega);
        }

        // If both linear and angular speeds are below thresholds, increment counter
        if (speed < specificConfig.linearSleepThreshold &&
            angularSpeed < specificConfig.angularSleepThreshold)
        {
            if (!sleep.asleep) {
                ++sleep.sleepCounter;
                if (sleep.sleepCounter > specificConfig.framesBeforeSleep) {
                    sleep.asleep = true;
                }
            }
        } else {
            // Reset counter if above threshold
            sleep.sleepCounter = 0;
            sleep.asleep = false;
        }
        
        // If asleep, zero out motion
        if (sleep.asleep) {
            vel.x = 0.0;
            vel.y = 0.0;
            // Zero angular velocity if present
            if (registry.any_of<Components::AngularVelocity>(entity)) {
                auto &angVel = registry.get<Components::AngularVelocity>(entity);
                angVel.omega = 0.0;
            }
        }
    }
}

} // namespace Systems