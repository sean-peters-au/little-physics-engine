#include "nbody/systems/rotation.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"

namespace Systems {

void RotationSystem::update(entt::registry &registry) {
    double dt = SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration;

    auto view = registry.view<Components::AngularPosition, Components::AngularVelocity>();
    for (auto [entity, angPos, angVel] : view.each()) {
        // Update angular position
        angPos.angle += angVel.omega * dt;

        // Angular damping
        double angularDamping = 0.98; // reduces spin by 2% each frame
        angVel.omega *= angularDamping;

        // (Optional) Clamp angular velocity
        double maxAngularSpeed = 20.0;
        if (angVel.omega > maxAngularSpeed) angVel.omega = maxAngularSpeed;
        if (angVel.omega < -maxAngularSpeed) angVel.omega = -maxAngularSpeed;

        // Normalize angle to [0, 2π)
        if (angPos.angle > 2.0 * SimulatorConstants::Pi)
            angPos.angle -= 2.0 * SimulatorConstants::Pi;
        else if (angPos.angle < 0)
            angPos.angle += 2.0 * SimulatorConstants::Pi;
    }
}

} // namespace Systems