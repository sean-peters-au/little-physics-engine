#include "nbody/systems/collision/solid_collision_response_system.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/math/vector_math.hpp"
#include <algorithm>
#include <cmath>

void Systems::SolidCollisionResponseSystem::update(entt::registry &registry, CollisionManifold &manifold) {
    for (auto &col : manifold.collisions) {
        // Check if either entity is asleep
        bool asleepA = false;
        bool asleepB = false;
        if (registry.any_of<Components::Sleep>(col.a)) {
            auto &sleepA = registry.get<Components::Sleep>(col.a);
            asleepA = sleepA.asleep;
        }
        if (registry.any_of<Components::Sleep>(col.b)) {
            auto &sleepB = registry.get<Components::Sleep>(col.b);
            asleepB = sleepB.asleep;
        }

        if (asleepA && asleepB) {
            // Both asleep, skip collision response
            continue;
        }

        auto &phaseA = registry.get<Components::ParticlePhase>(col.a);
        auto &phaseB = registry.get<Components::ParticlePhase>(col.b);
        if (phaseA.phase != Components::Phase::Solid && phaseB.phase != Components::Phase::Solid) {
            continue;
        }

        // Extract positions and velocities as Positions and Vectors
        auto posA = registry.get<Components::Position>(col.a);
        auto velA = registry.get<Components::Velocity>(col.a); // Velocity is a Vector
        auto &massA = registry.get<Components::Mass>(col.a);

        auto posB = registry.get<Components::Position>(col.b);
        auto velB = registry.get<Components::Velocity>(col.b);
        auto &massB = registry.get<Components::Mass>(col.b);

        double invMassA = 1.0/massA.value;
        double invMassB = 1.0/massB.value;
        double invSum = invMassA+invMassB;

        // Normal and penetration from collision
        Vector n = col.normal; // Already a Vector
        double penetration = col.penetration;

        // Compute relative velocity
        Vector relativeVel = velB - velA;
        double normalSpeed = relativeVel.dotProduct(n);

        // Restitution
        double restitution = 0.5; // scenario-based?
        if (std::fabs(normalSpeed) < 0.5) restitution *= 0.5;

        if (normalSpeed < 0) {
            double j = -(1+restitution)*normalSpeed / invSum;
            double frictionCoeff = 0.3;

            // Apply normal impulse
            Vector impulse = n * j;
            velA = velA - (impulse * invMassA);
            velB = velB + (impulse * invMassB);

            // Positional correction (baumgarte)
            double baumgarte = 0.2;
            double slop = 0.001;
            double corr = std::max(penetration - slop,0.0)*baumgarte/invSum;

            // Move posA and posB: Convert vector correction into Position adjustments
            Position corrA = Position(n.x*(corr*invMassA), n.y*(corr*invMassA));
            Position corrB = Position(n.x*(corr*invMassB), n.y*(corr*invMassB));

            posA = Position(posA.x - corrA.x, posA.y - corrA.y);
            posB = Position(posB.x + corrB.x, posB.y + corrB.y);

            // Friction: tangent direction
            Vector t(-n.y, n.x);
            double tangentSpeed = relativeVel.dotProduct(t);
            double jt = -tangentSpeed/(invSum);
            double maxFriction = j*frictionCoeff;
            jt = std::clamp(jt,-maxFriction,maxFriction);

            Vector frictionImpulse = t * jt;
            velA = velA - (frictionImpulse * invMassA);
            velB = velB + (frictionImpulse * invMassB);

            // Angular impulses if angular components exist
            if (registry.all_of<Components::AngularVelocity,Components::Inertia>(col.a) &&
                registry.all_of<Components::AngularVelocity,Components::Inertia>(col.b)) {
                auto &angVelA = registry.get<Components::AngularVelocity>(col.a);
                auto &I_A = registry.get<Components::Inertia>(col.a);
                auto &angVelB = registry.get<Components::AngularVelocity>(col.b);
                auto &I_B = registry.get<Components::Inertia>(col.b);

                // Compute lever arms:
                // Convert posA, posB to Vector for math
                Vector pA(posA.x, posA.y);
                Vector pB(posB.x, posB.y);
                Vector c(col.contactPoint.x, col.contactPoint.y);

                Vector rA = c - pA;
                Vector rB = c - pB;

                // Angular impulse from normal:
                double angularImpulseA = rA.cross(impulse);
                double angularImpulseB = rB.cross(impulse)*(-1);
                angVelA.omega += angularImpulseA / I_A.I;
                angVelB.omega += angularImpulseB / I_B.I;

                // Angular impulse from friction:
                double fAngA = rA.cross(frictionImpulse);
                double fAngB = rB.cross(frictionImpulse)*(-1);
                angVelA.omega += fAngA / I_A.I;
                angVelB.omega += fAngB / I_B.I;

                // Angular damping:
                double angularDamp=0.98;
                angVelA.omega *= angularDamp;
                angVelB.omega *= angularDamp;
            }
        }

        // Store back updated pos and vel
        registry.replace<Components::Position>(col.a, posA);
        registry.replace<Components::Position>(col.b, posB);
        registry.replace<Components::Velocity>(col.a, velA);
        registry.replace<Components::Velocity>(col.b, velB);
    }
}