#include "nbody/systems/dampening.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include <cmath>

namespace Systems {

void DampeningSystem::update(entt::registry &registry) {
    // We’ll apply linear and angular damping for non-sleeping objects
    auto view = registry.view<Components::Position, Components::Velocity>();

    for (auto [entity, pos, vel] : view.each()) {
        bool asleep = false;
        if (registry.any_of<Components::Sleep>(entity)) {
            auto &sleepC = registry.get<Components::Sleep>(entity);
            asleep = sleepC.asleep;
        }

        if (!asleep) {
            // Apply linear damping
            vel.x *= 0.99;
            vel.y *= 0.99;

            // If there's angular velocity, apply angular damping
            if (registry.any_of<Components::AngularVelocity>(entity)) {
                auto &angVel = registry.get<Components::AngularVelocity>(entity);
                angVel.omega *= 0.99;
                registry.replace<Components::AngularVelocity>(entity, angVel);
            }

            // Store the updated velocity
            registry.replace<Components::Velocity>(entity, vel);
        }
    }
}

} // namespace Systems 