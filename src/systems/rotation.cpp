/**
 * @file rotation.cpp
 * @brief Implementation of rotational motion system
 */

#include "nbody/core/profile.hpp"
#include "nbody/systems/rotation.hpp"
#include "nbody/entities/entity_components.hpp"
#include "nbody/components/sim.hpp"
#include "nbody/core/constants.hpp"

namespace Systems {

RotationSystem::RotationSystem() {
    // Initialize with default configurations
}

void RotationSystem::update(entt::registry &registry) {
    PROFILE_SCOPE("RotationSystem");
    // Get simulator state
    const auto& state = registry.get<Components::SimulatorState>(
        registry.view<Components::SimulatorState>().front()
    );

    // Time step in real seconds using simulator state
    double const dt = sysConfig.SecondsPerTick * 
                state.baseTimeAcceleration * state.timeScale;

    auto view = registry.view<Components::AngularPosition, Components::AngularVelocity>();
    for (auto [entity, angPos, angVel] : view.each()) {
        if (registry.any_of<Components::Boundary>(entity)) {
            continue; // Don't rotate boundaries
        }

        // Update angular position
        angPos.angle += angVel.omega * dt;

        // Angular damping (if configured)
        if (specificConfig.angularDamping < 1.0) {
            angVel.omega *= specificConfig.angularDamping;
        }

        // Clamp angular velocity (if configured)
        if (specificConfig.maxAngularSpeed > 0) {
            if (angVel.omega > specificConfig.maxAngularSpeed) {
                angVel.omega = specificConfig.maxAngularSpeed;
            }
            if (angVel.omega < -specificConfig.maxAngularSpeed) {
                angVel.omega = -specificConfig.maxAngularSpeed;
            }
        }

        // Normalize angle to [0, 2π)
        if (angPos.angle > 2.0 * SimulatorConstants::Pi) {
            angPos.angle -= 2.0 * SimulatorConstants::Pi;
        } else if (angPos.angle < 0) {
            angPos.angle += 2.0 * SimulatorConstants::Pi;
        }
    }
}

} // namespace Systems