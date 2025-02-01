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

void BasicGravitySystem::update(entt::registry& registry) {
    PROFILE_SCOPE("BasicGravitySystem");

    // Get current simulator state for time scaling
    const auto& state = registry.get<Components::SimulatorState>(
        registry.view<Components::SimulatorState>().front()
    );

    // Standard gravitational acceleration (9.8 m/s²)
    constexpr double GRAVITY = 9.8;

    // Calculate time step with all scaling factors applied
    double const dt = SimulatorConstants::SecondsPerTick * 
                state.baseTimeAcceleration * 
                state.timeScale;

    // Update velocities for all entities with required components
    auto view = registry.view<Components::ParticlePhase, 
                            Components::Velocity, 
                            Components::Mass>();
    
    for (auto [entity, phase, vel, mass] : view.each()) {
        if (registry.any_of<Components::Boundary>(entity)) {
            continue;
        }
        // Apply gravitational acceleration in negative y direction
        vel.y += GRAVITY * dt;
    }
}

} // namespace Systems