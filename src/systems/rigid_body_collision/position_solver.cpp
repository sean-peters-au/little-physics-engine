#include <algorithm>
#include <cmath>

#include "nbody/systems/rigid_body_collision/position_solver.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"

namespace RigidBodyCollision
{

void PositionSolver::positionalSolver(entt::registry &registry,
                                      const CollisionManifold &manifold,
                                      int iterations,
                                      double baumgarte,
                                      double slop)
{
    for (int i=0; i<iterations; i++) {
        for (auto &col : manifold.collisions) {
            if (!registry.valid(col.a) || !registry.valid(col.b)) continue;
            if (!registry.all_of<Components::Position, Components::Mass, Components::ParticlePhase>(col.a)) continue;
            if (!registry.all_of<Components::Position, Components::Mass, Components::ParticlePhase>(col.b)) continue;

            auto &phaseA = registry.get<Components::ParticlePhase>(col.a);
            auto &phaseB = registry.get<Components::ParticlePhase>(col.b);
            if (phaseA.phase != Components::Phase::Solid && phaseB.phase != Components::Phase::Solid) {
                continue;
            }

            auto &massA = registry.get<Components::Mass>(col.a);
            auto &massB = registry.get<Components::Mass>(col.b);
            auto posA = registry.get<Components::Position>(col.a);
            auto posB = registry.get<Components::Position>(col.b);

            double invMassA = (massA.value>1e29)? 0.0 : ((massA.value>1e-12)? 1.0/massA.value:0.0);
            double invMassB = (massB.value>1e29)? 0.0 : ((massB.value>1e-12)? 1.0/massB.value:0.0);
            double invSum = invMassA + invMassB;
            if (invSum<1e-12) continue;

            double pen = col.penetration;
            if (pen <= slop) continue;

            double corr = std::max(pen - slop, 0.0) * baumgarte / invSum;
            auto n = col.normal;

            posA.x -= n.x * corr * invMassA;
            posA.y -= n.y * corr * invMassA;
            posB.x += n.x * corr * invMassB;
            posB.y += n.y * corr * invMassB;

            registry.replace<Components::Position>(col.a, posA);
            registry.replace<Components::Position>(col.b, posB);
        }
    }
}

} // namespace RigidBodyCollision