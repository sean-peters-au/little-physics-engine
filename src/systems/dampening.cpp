/**
 * @file dampening.cpp
 * @brief Implementation of velocity dampening system
 */

#include "nbody/core/profile.hpp"
#include "nbody/systems/dampening.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include <cmath>

namespace Systems {

DampeningSystem::DampeningSystem() {
    // Initialize with default configurations
}

void DampeningSystem::setSystemConfig(const SystemConfig& config) {
    sysConfig = config;
}

void DampeningSystem::setDampeningConfig(const DampeningConfig& config) {
    dampeningConfig = config;
}

void DampeningSystem::update(entt::registry &registry) {
    PROFILE_SCOPE("DampeningSystem");

    // We'll apply linear and angular damping for non-sleeping objects
    auto view = registry.view<Components::Position, Components::Velocity>();

    for (auto [entity, pos, vel] : view.each()) {
        // Apply linear damping
        vel.x *= dampeningConfig.linearDampingFactor;
        vel.y *= dampeningConfig.linearDampingFactor;

        // If there's angular velocity, apply angular damping
        if (registry.any_of<Components::AngularVelocity>(entity)) {
            auto &angVel = registry.get<Components::AngularVelocity>(entity);
            angVel.omega *= dampeningConfig.angularDampingFactor;
            registry.replace<Components::AngularVelocity>(entity, angVel);
        }

        // Store the updated velocity
        registry.replace<Components::Velocity>(entity, vel);
    }
}

} // namespace Systems 