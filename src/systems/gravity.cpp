/**
 * @file gravity.cpp
 * @brief Implementation of basic gravitational force system
 */

#include "nbody/core/profile.hpp"
#include "nbody/systems/gravity.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sim.hpp"
#include "nbody/core/constants.hpp"
#include <cmath>

namespace Systems {

BasicGravitySystem::BasicGravitySystem() {
    // Initialize with default configurations
}

void BasicGravitySystem::setSystemConfig(const SystemConfig& config) {
    sysConfig = config;
}

void BasicGravitySystem::setGravityConfig(const GravityConfig& config) {
    gravityConfig = config;
}

void BasicGravitySystem::update(entt::registry& registry) {
    PROFILE_SCOPE("BasicGravitySystem");

    // Get current simulator state for time scaling
    const auto& state = registry.get<Components::SimulatorState>(
        registry.view<Components::SimulatorState>().front()
    );

    // Get the gravitational acceleration from config
    double gravity = gravityConfig.gravitationalAcceleration;

    // Calculate time step with all scaling factors applied
    double const dt = sysConfig.SecondsPerTick * 
                state.baseTimeAcceleration * 
                state.timeScale;

    // Create a view that automatically excludes Boundary entities
    auto view = registry.view<Components::ParticlePhase, Components::Velocity, Components::Mass>(
        entt::exclude<Components::Boundary>
    );
    
    // Update velocities for entities not tagged as a Boundary
    for (auto [entity, phase, vel, mass] : view.each()) {
        vel.y += gravity * dt;
    }
}

} // namespace Systems