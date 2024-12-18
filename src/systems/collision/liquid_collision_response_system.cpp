#include "nbody/systems/collision/liquid_collision_response_system.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/systems/collision/collision_data.hpp"

void Systems::LiquidCollisionResponseSystem::update(entt::registry &registry, CollisionManifold &manifold) {
    for (auto &col : manifold.collisions) {
        auto &phaseA = registry.get<Components::ParticlePhase>(col.a);
        auto &phaseB = registry.get<Components::ParticlePhase>(col.b);

        // If both or one is liquid
        if (phaseA.phase == Components::Phase::Liquid || phaseB.phase == Components::Phase::Liquid) {
            // Adjust positions or velocities in a way consistent with fluid simulation
            // For SPH, might add repulsive force or just rely on SPH solver.
        }
    }
}