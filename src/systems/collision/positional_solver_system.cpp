#include "nbody/systems/collision/positional_solver_system.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/math/vector_math.hpp"
#include <algorithm>
#include <cmath>

/**
 * This positional solver does not apply impulses; it only adjusts positions
 * to reduce penetration. Each iteration moves objects along the collision 
 * normal to separate them further.
 * 
 * By running multiple iterations, we gradually push objects apart until 
 * the penetration is very small.
 */
void Systems::PositionalSolverSystem::update(entt::registry &registry, CollisionManifold &manifold, 
                                             int iterations, double baumgarte, double slop) {
    for (int iter = 0; iter < iterations; ++iter) {
        for (auto &col : manifold.collisions) {
            // If entities not valid anymore, skip
            if (!registry.valid(col.a) || !registry.valid(col.b)) continue;

            // Retrieve components
            if (!registry.all_of<Components::Position, Components::Mass, Components::ParticlePhase>(col.a)) continue;
            if (!registry.all_of<Components::Position, Components::Mass, Components::ParticlePhase>(col.b)) continue;

            auto posA = registry.get<Components::Position>(col.a);
            auto &massA = registry.get<Components::Mass>(col.a);

            auto posB = registry.get<Components::Position>(col.b);
            auto &massB = registry.get<Components::Mass>(col.b);

            auto &phaseA = registry.get<Components::ParticlePhase>(col.a);
            auto &phaseB = registry.get<Components::ParticlePhase>(col.b);

            // Only solve positional for solids (or at least one solid)
            if (phaseA.phase != Components::Phase::Solid && phaseB.phase != Components::Phase::Solid)
                continue;

            double invMassA = (massA.value > 1e-12) ? 1.0 / massA.value : 0.0;
            double invMassB = (massB.value > 1e-12) ? 1.0 / massB.value : 0.0;
            double invSum = invMassA + invMassB;
            if (invSum < 1e-12) continue; // no correction if both infinite mass or zero mass

            double penetration = col.penetration;
            if (penetration <= slop) continue; // no correction needed

            // Apply positional correction based on baumgarte stabilization
            double corr = (penetration - slop) * baumgarte / invSum;
            Vector n = col.normal; // direction of correction

            double dxA = n.x * corr * invMassA;
            double dyA = n.y * corr * invMassA;
            double dxB = n.x * corr * invMassB;
            double dyB = n.y * corr * invMassB;

            posA.x -= dxA;
            posA.y -= dyA;
            posB.x += dxB;
            posB.y += dyB;

            // Store back updated positions
            registry.replace<Components::Position>(col.a, posA);
            registry.replace<Components::Position>(col.b, posB);
        }
    }
}