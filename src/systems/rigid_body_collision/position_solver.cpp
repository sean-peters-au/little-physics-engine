/**
 * @file position_solver.cpp
 * @brief Implementation of position-based penetration correction
 */

#include "nbody/systems/rigid_body_collision/position_solver.hpp"
#include <algorithm>
#include <entt/entt.hpp>
#include "nbody/components/basic.hpp"
#include "nbody/core/profile.hpp"

const double BAUMGARTE = 0.5;
const double SLOP = 0.001;
const double ITERATIONS = 3;

namespace RigidBodyCollision {

void PositionSolver::positionalSolver(
    entt::registry &registry,
    const CollisionManifold &manifold
)
{
    PROFILE_SCOPE("PositionSolver");

    // We iterate multiple times to allow stacking corrections to converge
    for (int i = 0; i < ITERATIONS; i++) {
        // Now each 'c' in manifold.collisions might represent a single contact
        // and we might have multiple for the same pair.
        for (auto &c : manifold.collisions) {
            if (!registry.valid(c.a) || !registry.valid(c.b)) continue;

            // Ensure entities have the required components
            if (!registry.all_of<Components::Position, Components::Mass, Components::ParticlePhase>(c.a)) 
                continue;
            if (!registry.all_of<Components::Position, Components::Mass, Components::ParticlePhase>(c.b)) 
                continue;

            // Only correct positions for solid phase
            auto &phaseA = registry.get<Components::ParticlePhase>(c.a);
            auto &phaseB = registry.get<Components::ParticlePhase>(c.b);
            if (phaseA.phase != Components::Phase::Solid &&
                phaseB.phase != Components::Phase::Solid) {
                continue;
            }

            auto &massA = registry.get<Components::Mass>(c.a);
            auto &massB = registry.get<Components::Mass>(c.b);
            auto posA   = registry.get<Components::Position>(c.a);
            auto posB   = registry.get<Components::Position>(c.b);

            double invMassA = (massA.value > 1e29) ? 0.0 : (1.0 / massA.value);
            double invMassB = (massB.value > 1e29) ? 0.0 : (1.0 / massB.value);
            double invSum   = invMassA + invMassB;
            if (invSum < 1e-12) continue;  // both infinite => no correction

            double pen = c.penetration;
            if (pen <= SLOP) continue;

            // Apply standard Baumgarte approach
            double corr = std::max(pen - SLOP, 0.0) * BAUMGARTE / invSum;
            auto n = c.normal;  // normal from A->B

            // We'll push bodies out along 'n'
            // but if you wanted to push along the actual contactPoint,
            // you'd use that for torque or advanced approaches, etc.
            posA.x -= n.x * corr * invMassA;
            posA.y -= n.y * corr * invMassA;
            posB.x += n.x * corr * invMassB;
            posB.y += n.y * corr * invMassB;

            registry.replace<Components::Position>(c.a, posA);
            registry.replace<Components::Position>(c.b, posB);
        }
    }
}

} // namespace RigidBodyCollision