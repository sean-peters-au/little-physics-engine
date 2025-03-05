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

    // View of entities with Position and Velocity
    auto view = registry.view<Components::Position, Components::Velocity>();

    for (auto [entity, pos, vel] : view.each()) {
        // Skip boundaries
        if (registry.any_of<Components::Boundary>(entity)) {
            continue; // Don't move boundaries
        }
        
        // Hack: Only move solids; skip if ParticlePhase exists and is not Solid
        if (registry.any_of<Components::ParticlePhase>(entity)) {
            if (registry.get<Components::ParticlePhase>(entity).phase == Components::Phase::Liquid) {
                continue; // Skip fluids (or any non-solid phase)
            }
        }

        // Update position for solids
        pos.x += vel.x * dt;
        pos.y += vel.y * dt;

        // Update components in registry
        registry.replace<Components::Position>(entity, pos);
        registry.replace<Components::Velocity>(entity, vel);
    }
}

} // namespace Systems 