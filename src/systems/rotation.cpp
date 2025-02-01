/**
 * @file rotation.cpp
 * @brief Implementation of rotational motion system
 */

#include "nbody/core/profile.hpp"
#include "nbody/systems/rotation.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sim.hpp"
#include "nbody/core/constants.hpp"

namespace Systems {

void RotationSystem::update(entt::registry &registry) {
    PROFILE_SCOPE("RotationSystem");
    // Get simulator state
    const auto& state = registry.get<Components::SimulatorState>(
        registry.view<Components::SimulatorState>().front()
    );

    // Time step in real seconds using simulator state
    double const dt = SimulatorConstants::SecondsPerTick * 
                state.baseTimeAcceleration * state.timeScale;

    auto view = registry.view<Components::AngularPosition, Components::AngularVelocity>();
    for (auto [entity, angPos, angVel] : view.each()) {
        if (registry.any_of<Components::Boundary>(entity)) {
            continue; // Don't rotate boundaries
        }

        // Update angular position
        angPos.angle += angVel.omega * dt;

        // Angular damping
        double const angularDamping = 0.98;
        angVel.omega *= angularDamping;

        // (Optional) Clamp angular velocity
        double const maxAngularSpeed = 20.0;
        if (angVel.omega > maxAngularSpeed) { angVel.omega = maxAngularSpeed;
}
        if (angVel.omega < -maxAngularSpeed) { angVel.omega = -maxAngularSpeed;
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