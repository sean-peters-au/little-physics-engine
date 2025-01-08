#include "nbody/systems/movement.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/core/profile.hpp"
#include <cmath>

namespace Systems {

void MovementSystem::update(entt::registry &registry) {
    PROFILE_SCOPE("MovementSystem");

    // Time step in real seconds (with all time scaling)
    double dt = SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration;

    // View of entities with Position and Velocity
    auto view = registry.view<Components::Position, Components::Velocity>();

    for (auto [entity, pos, vel] : view.each()) {
        // Check whether entity is asleep
        if (registry.any_of<Components::Boundary>(entity)) {
            continue; // Don't move boundaries
        }

        // Update position
        pos.x += vel.x * dt;
        pos.y += vel.y * dt;

        // Update components in registry
        registry.replace<Components::Position>(entity, pos);
        registry.replace<Components::Velocity>(entity, vel);
    }
}

} // namespace Systems 