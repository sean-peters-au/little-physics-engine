// position_solver.cpp
#include "nbody/systems/rigid_body_collision/position_solver.hpp"
#include <algorithm>
#include <entt/entt.hpp>
#include "nbody/components/basic.hpp"
#include "nbody/core/profile.hpp"
#include <cmath>

namespace RigidBodyCollision {

/**
 * A smaller BAUMGARTE factor for position correction often helps stability
 * if there are multiple contact points. You can experiment with 0.1–0.8.
 */
static constexpr double BAUMGARTE = 0.2;
static constexpr double SLOP = 0.001;
static constexpr int POS_SOLVER_ITERATIONS = 5;

void PositionSolver::positionalSolver(
    entt::registry &registry,
    const CollisionManifold &manifold
)
{
    PROFILE_SCOPE("PositionSolver");

    // We do a few sequential iterations over all contacts in the manifold.
    // This approach is similar to the sequential impulse "split impulse" method
    // used in many 2D physics engines (like Box2D).
    for (int iter = 0; iter < POS_SOLVER_ITERATIONS; ++iter)
    {
        for (auto &c : manifold.collisions)
        {
            if (!registry.valid(c.a) || !registry.valid(c.b))
                continue;

            // Make sure both have position, mass, etc.
            if (!registry.all_of<Components::Position, Components::Mass, Components::ParticlePhase>(c.a)
             || !registry.all_of<Components::Position, Components::Mass, Components::ParticlePhase>(c.b))
            {
                continue;
            }

            // We only do position-correction if at least one object is SOLID
            auto &phaseA = registry.get<Components::ParticlePhase>(c.a).phase;
            auto &phaseB = registry.get<Components::ParticlePhase>(c.b).phase;
            if (phaseA != Components::Phase::Solid && phaseB != Components::Phase::Solid)
                continue;

            // Retrieve data
            auto &massA = registry.get<Components::Mass>(c.a).value;
            auto &massB = registry.get<Components::Mass>(c.b).value;

            // Angular data is optional: if an entity lacks AngularPosition/Inertia, treat as no rotation
            double angleA = 0.0, angleB = 0.0;
            bool canRotateA = false, canRotateB = false;
            double inertiaA = 0.0, inertiaB = 0.0;

            if (registry.any_of<Components::AngularPosition>(c.a))
            {
                angleA = registry.get<Components::AngularPosition>(c.a).angle;
            }
            if (registry.any_of<Components::Inertia>(c.a))
            {
                inertiaA = registry.get<Components::Inertia>(c.a).I;
                if (inertiaA < 1e29) {
                    canRotateA = true;
                }
            }

            if (registry.any_of<Components::AngularPosition>(c.b))
            {
                angleB = registry.get<Components::AngularPosition>(c.b).angle;
            }
            if (registry.any_of<Components::Inertia>(c.b))
            {
                inertiaB = registry.get<Components::Inertia>(c.b).I;
                if (inertiaB < 1e29) {
                    canRotateB = true;
                }
            }

            auto posA = registry.get<Components::Position>(c.a);
            auto posB = registry.get<Components::Position>(c.b);

            // Compute inverse mass
            double invMassA = (massA > 1e29) ? 0.0 : (1.0 / massA);
            double invMassB = (massB > 1e29) ? 0.0 : (1.0 / massB);

            // For infinite inertia, set invInertia = 0
            double invIA = (canRotateA && inertiaA > 1e-12) ? (1.0 / inertiaA) : 0.0;
            double invIB = (canRotateB && inertiaB > 1e-12) ? (1.0 / inertiaB) : 0.0;

            // Check if penetration is large enough to warrant correction
            double pen = c.penetration - SLOP;
            if (pen <= 0.0)
                continue;

            double correctionMag = BAUMGARTE * pen;

            // We want to move object A "back" along the normal and object B "forward"
            // along the normal so that they separate. The contact normal points from A to B.
            // We'll also incorporate the lever arms (rA, rB) for angular correction.
            auto normal = c.normal.normalized(); 
            // Contact point rA, rB in *world space*. 
            // rA = contact - posA
            // rB = contact - posB
            // (since posA.x, posA.y is the body's origin)
            Vector rA(c.contactPoint.x - posA.x, c.contactPoint.y - posA.y);
            Vector rB(c.contactPoint.x - posB.x, c.contactPoint.y - posB.y);

            // For position corrections, the "effective mass" is:
            //
            //   K = (invMassA + invMassB)
            //       + (rA x n)^2 * invIA
            //       + (rB x n)^2 * invIB
            //
            // If K is near zero, we do no correction.
            double rACrossN = rA.cross(normal);
            double rBCrossN = rB.cross(normal);
            double denom = invMassA + invMassB
                         + (rACrossN * rACrossN) * invIA
                         + (rBCrossN * rBCrossN) * invIB;

            if (denom < 1e-12)
                continue;

            double scalar = correctionMag / denom;

            // The final linear correction
            Vector correction = normal * scalar;

            // Apply to posA (negative direction)
            posA.x -= correction.x * invMassA;
            posA.y -= correction.y * invMassA;

            // Angular correction: angleA -= (rA cross n) * scalar * invIA
            if (canRotateA) {
                double rotA = rACrossN * scalar * invIA;
                angleA -= rotA;
            }

            // Apply to posB (positive direction)
            posB.x += correction.x * invMassB;
            posB.y += correction.y * invMassB;

            // Angular correction: angleB += (rB cross n) * scalar * invIB
            if (canRotateB) {
                double rotB = rBCrossN * scalar * invIB;
                angleB += rotB;
            }

            // Store back in registry
            registry.replace<Components::Position>(c.a, posA);
            if (registry.any_of<Components::AngularPosition>(c.a)) {
                auto &apA = registry.get<Components::AngularPosition>(c.a);
                apA.angle = angleA;
                registry.replace<Components::AngularPosition>(c.a, apA);
            }

            registry.replace<Components::Position>(c.b, posB);
            if (registry.any_of<Components::AngularPosition>(c.b)) {
                auto &apB = registry.get<Components::AngularPosition>(c.b);
                apB.angle = angleB;
                registry.replace<Components::AngularPosition>(c.b, apB);
            }
        }
    }
}

} // namespace RigidBodyCollision