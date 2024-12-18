#include "nbody/systems/collision/gas_collision_response_system.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/systems/collision/collision_data.hpp"

void Systems::GasCollisionResponseSystem::update(entt::registry &registry, CollisionManifold &manifold) {
    for (auto &col : manifold.collisions) {
        auto &phaseA = registry.get<Components::ParticlePhase>(col.a);
        auto &phaseB = registry.get<Components::ParticlePhase>(col.b);

        if (phaseA.phase == Components::Phase::Gas || phaseB.phase == Components::Phase::Gas) {
            // Possibly exchange velocities, randomize directions, or apply a simplified gas collision model.
        }
    }
}