/**
 * @file position_solver.cpp
 * @brief Implementation of position-based penetration correction
 */

#include "nbody/systems/rigid_body_collision/position_solver.hpp"
#include <algorithm>
#include <entt/entt.hpp>
#include "nbody/components/basic.hpp"
#include "nbody/core/profile.hpp"

namespace RigidBodyCollision {

void PositionSolver::positionalSolver(entt::registry &registry,
                                    const CollisionManifold &manifold,
                                    int iterations,
                                    double baumgarte,
                                    double slop)
{
    PROFILE_SCOPE("PositionSolver");

    // Multiple iterations help with stability and stacking
    for (int i = 0; i < iterations; i++) {
        for (auto &c : manifold.collisions) {
            // Skip invalid or deleted entities
            if (!registry.valid(c.a) || !registry.valid(c.b)) continue;

            // Ensure entities have required components
            if (!registry.all_of<Components::Position, Components::Mass, 
                               Components::ParticlePhase>(c.a)) continue;
            if (!registry.all_of<Components::Position, Components::Mass, 
                               Components::ParticlePhase>(c.b)) continue;

            // Only correct positions for solid phase particles
            auto &phaseA = registry.get<Components::ParticlePhase>(c.a);
            auto &phaseB = registry.get<Components::ParticlePhase>(c.b);
            if (phaseA.phase != Components::Phase::Solid && 
                phaseB.phase != Components::Phase::Solid) {
                continue;
            }

            // Get physics components
            auto &massA = registry.get<Components::Mass>(c.a);
            auto &massB = registry.get<Components::Mass>(c.b);
            auto posA = registry.get<Components::Position>(c.a);
            auto posB = registry.get<Components::Position>(c.b);

            // Handle infinite mass bodies
            double invMassA = (massA.value > 1e29) ? 0.0 : (1.0 / massA.value);
            double invMassB = (massB.value > 1e29) ? 0.0 : (1.0 / massB.value);
            double invSum = invMassA + invMassB;
            if (invSum < 1e-12) continue;  // Both bodies are static

            // Apply position correction only if penetration exceeds slop
            double pen = c.penetration;
            if (pen <= slop) continue;

            // Calculate correction magnitude based on penetration
            double corr = std::max(pen - slop, 0.0) * baumgarte / invSum;
            auto n = c.normal;

            // Apply mass-weighted position adjustments
            posA.x -= n.x * corr * invMassA;
            posA.y -= n.y * corr * invMassA;
            posB.x += n.x * corr * invMassB;
            posB.y += n.y * corr * invMassB;

            // Update positions in registry
            registry.replace<Components::Position>(c.a, posA);
            registry.replace<Components::Position>(c.b, posB);
        }
    }
}

} // namespace RigidBodyCollision