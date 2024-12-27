#include "nbody/systems/collision/positional_solver_system.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/math/vector_math.hpp"
#include <algorithm>
#include <cmath>

void Systems::PositionalSolverSystem::update(entt::registry &registry, CollisionManifold &manifold, 
                                             int iterations, double baumgarte, double slop) {
    for (int iter = 0; iter < iterations; ++iter) {
        for (auto &col : manifold.collisions) {
            if (!registry.valid(col.a) || !registry.valid(col.b)) continue;

            if (!registry.all_of<Components::Position, Components::Mass, Components::ParticlePhase>(col.a)) continue;
            if (!registry.all_of<Components::Position, Components::Mass, Components::ParticlePhase>(col.b)) continue;

            auto posA = registry.get<Components::Position>(col.a);
            auto &massA = registry.get<Components::Mass>(col.a);

            auto posB = registry.get<Components::Position>(col.b);
            auto &massB = registry.get<Components::Mass>(col.b);

            auto &phaseA = registry.get<Components::ParticlePhase>(col.a);
            auto &phaseB = registry.get<Components::ParticlePhase>(col.b);

            if (phaseA.phase != Components::Phase::Solid && phaseB.phase != Components::Phase::Solid)
                continue;

            double invMassA = (massA.value > 1e29) ? 0.0 : ((massA.value > 1e-12) ? 1.0 / massA.value : 0.0);
            double invMassB = (massB.value > 1e29) ? 0.0 : ((massB.value > 1e-12) ? 1.0 / massB.value : 0.0);

            double invSum = invMassA + invMassB;
            if (invSum < 1e-12) continue;

            double penetration = col.penetration;
            if (penetration <= slop) continue;

            double corr = (penetration - slop) * baumgarte / invSum;
            Vector n = col.normal;

            double dxA = n.x * corr * invMassA;
            double dyA = n.y * corr * invMassA;
            double dxB = n.x * corr * invMassB;
            double dyB = n.y * corr * invMassB;

            posA.x -= dxA;
            posA.y -= dyA;
            posB.x += dxB;
            posB.y += dyB;

            registry.replace<Components::Position>(col.a, posA);
            registry.replace<Components::Position>(col.b, posB);
        }
    }
}