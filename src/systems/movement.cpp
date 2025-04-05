#include "systems/movement.hpp"
#include "entities/entity_components.hpp"
#include "core/constants.hpp"
#include "core/profile.hpp"
#include <cmath>

namespace Systems {

MovementSystem::MovementSystem() {
    // Initialize with default configurations
}

void MovementSystem::update(entt::registry &registry) {
    PROFILE_SCOPE("MovementSystem");

    // Time step in real seconds (with all time scaling)
    double const dt = sysConfig.SecondsPerTick * sysConfig.TimeAcceleration;

    // View of entities with Position and Velocity, excluding Boundaries and ParticlePhase
    auto view = registry.view<Components::Position, Components::Velocity>(entt::exclude<Components::Boundary, Components::ParticlePhase>);

    for (auto [entity, pos, vel] : view.each()) {
        // Update position for solids (or any remaining entities)
        pos.x += vel.x * dt;
        pos.y += vel.y * dt;

        // Update components in registry
        registry.replace<Components::Position>(entity, pos);
        registry.replace<Components::Velocity>(entity, vel);
    }
}

} // namespace Systems 