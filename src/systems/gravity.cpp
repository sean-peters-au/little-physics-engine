/**
 * @file gravity.cpp
 * @brief Implementation of basic gravitational force system
 */

#include "core/profile.hpp"
#include "systems/gravity.hpp"
#include "entities/entity_components.hpp"
#include "entities/sim_components.hpp"
#include "core/constants.hpp"
#include <cmath>

namespace Systems {

BasicGravitySystem::BasicGravitySystem() {
    // Initialize with default configurations
}

void BasicGravitySystem::update(entt::registry& registry) {
    PROFILE_SCOPE("BasicGravitySystem");

    // Get current simulator state for time scaling
    const auto& state = registry.get<Components::SimulatorState>(
        registry.view<Components::SimulatorState>().front()
    );

    // Get the gravitational acceleration from config
    double gravity = specificConfig.gravitationalAcceleration;

    // Calculate time step with all scaling factors applied
    double const dt = sysConfig.SecondsPerTick * 
                state.baseTimeAcceleration * 
                state.timeScale;

    // Create a view that automatically excludes Boundary entities
    auto view = registry.view<Components::ParticlePhase, Components::Velocity, Components::Mass>(
        entt::exclude<Components::Boundary>
    );

    // Check for planetary mass ONLY if the threshold is positive
    bool applyGravity = true;
    if (specificConfig.planetaryMassThreshold > 0.0) {
        for (auto entity : view) { // Use simpler view iteration for check
            if (view.get<Components::Mass>(entity).value >= specificConfig.planetaryMassThreshold) {
                applyGravity = false;
                break;  // Found a planetary body, stop checking
            }
        }
    }

    // If gravity should be applied, iterate ONCE to apply it.
    if (applyGravity) {
        for (auto [entity, phase, vel, mass] : view.each()) { // Iterate again, this time getting components
            vel.y += gravity * dt;
            // No need to check mass again here
        }
    }
}

} // namespace Systems