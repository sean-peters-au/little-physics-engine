/**
 * @file dampening.cpp
 * @brief Implementation of velocity dampening system
 */

#include "core/profile.hpp"
#include "systems/dampening.hpp"
#include "entities/entity_components.hpp"
#include "core/constants.hpp"
#include <cmath>

namespace Systems {

DampeningSystem::DampeningSystem() {
    // Initialize with default configurations
}

void DampeningSystem::update(entt::registry &registry) {
    PROFILE_SCOPE("DampeningSystem");

    // We'll apply linear and angular damping for non-sleeping objects
    auto view = registry.view<Components::Position, Components::Velocity>();

    for (auto [entity, pos, vel] : view.each()) {
        // Apply linear damping
        vel.x *= specificConfig.linearDamping;
        vel.y *= specificConfig.linearDamping;

        // If there's angular velocity, apply angular damping
        if (registry.any_of<Components::AngularVelocity>(entity)) {
            auto &angVel = registry.get<Components::AngularVelocity>(entity);
            // Use the same damping factor for angular velocity
            angVel.omega *= specificConfig.linearDamping;
            registry.replace<Components::AngularVelocity>(entity, angVel);
        }

        // Store the updated velocity
        registry.replace<Components::Velocity>(entity, vel);
    }
}

} // namespace Systems 