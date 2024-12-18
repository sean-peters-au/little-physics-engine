#include "nbody/systems/collision/solid_collision_response_system.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/math/vector_math.hpp"
#include <algorithm>
#include <cmath>

void Systems::SolidCollisionResponseSystem::update(entt::registry &registry, CollisionManifold &manifold) {
    const double baumgarte = 0.5;
    const double slop = 0.001;
    const double angularDamp = 0.98;

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

        auto posA = registry.get<Components::Position>(col.a);
        auto velA = registry.get<Components::Velocity>(col.a);
        auto &massA = registry.get<Components::Mass>(col.a);

        auto posB = registry.get<Components::Position>(col.b);
        auto velB = registry.get<Components::Velocity>(col.b);
        auto &massB = registry.get<Components::Mass>(col.b);

        double invMassA = 1.0/massA.value;
        double invMassB = 1.0/massB.value;
        double invSum = invMassA+invMassB;

        Vector n = col.normal;
        double penetration = col.penetration;

        Vector relativeVel = velB - velA;
        double normalSpeed = relativeVel.dotProduct(n);

        double restitution = 0.0; // Consider no restitution for stable stacking
        if (std::fabs(normalSpeed) < 0.5) restitution *= 0.5;

        double j = 0.0;
        if (normalSpeed < 0) {
            j = -(1.0+restitution)*normalSpeed / invSum;
        }

        // Apply normal impulse
        Vector impulse = n * j;
        velA = velA - (impulse * invMassA);
        velB = velB + (impulse * invMassB);

        // Positional correction
        double corr = std::max(penetration - slop,0.0)*baumgarte/invSum;
        Position corrA(n.x*(corr*invMassA), n.y*(corr*invMassA));
        Position corrB(n.x*(corr*invMassB), n.y*(corr*invMassB));
        posA = Position(posA.x - corrA.x, posA.y - corrA.y);
        posB = Position(posB.x + corrB.x, posB.y + corrB.y);

        // Compute friction
        // Retrieve materials (if any). If not present, assume defaults.
        double staticFrictionA=0.5, dynamicFrictionA=0.3;
        double staticFrictionB=0.5, dynamicFrictionB=0.3;
        if (registry.any_of<Components::Material>(col.a)) {
            auto &matA = registry.get<Components::Material>(col.a);
            staticFrictionA = matA.staticFriction;
            dynamicFrictionA = matA.dynamicFriction;
        }
        if (registry.any_of<Components::Material>(col.b)) {
            auto &matB = registry.get<Components::Material>(col.b);
            staticFrictionB = matB.staticFriction;
            dynamicFrictionB = matB.dynamicFriction;
        }

        // Combine friction (just average for now)
        double combinedStaticFriction = (staticFrictionA + staticFrictionB)*0.5;
        double combinedDynamicFriction = (dynamicFrictionA + dynamicFrictionB)*0.5;

        // Tangent direction
        Vector t = relativeVel - (n * (relativeVel.dotProduct(n)));
        double tLen = t.length();
        if (tLen > EPSILON) {
            t = t / tLen;
        } else {
            t = Vector(0,0);
        }

        double tangentSpeed = relativeVel.dotProduct(t);
        // If tangent speed is very small, use static friction
        double jt = -tangentSpeed/(invSum);
        double frictionImpulseMagnitude = std::fabs(jt);

        if (frictionImpulseMagnitude < j*combinedStaticFriction) {
            // Static friction can stop relative motion completely
            jt = -tangentSpeed/(invSum);
        } else {
            // Use dynamic friction
            jt = -j * combinedDynamicFriction * (jt>0?1:-1);
        }

        Vector frictionImpulse = t * jt;
        velA = velA - (frictionImpulse * invMassA);
        velB = velB + (frictionImpulse * invMassB);

        // Angular impulses if needed
        if (registry.all_of<Components::AngularVelocity,Components::Inertia>(col.a) &&
            registry.all_of<Components::AngularVelocity,Components::Inertia>(col.b)) {
            auto &angVelA = registry.get<Components::AngularVelocity>(col.a);
            auto &I_A = registry.get<Components::Inertia>(col.a);
            auto &angVelB = registry.get<Components::AngularVelocity>(col.b);
            auto &I_B = registry.get<Components::Inertia>(col.b);

            Vector pA(posA.x, posA.y);
            Vector pB(posB.x, posB.y);
            Vector c(col.contactPoint.x, col.contactPoint.y);

            Vector rA = c - pA;
            Vector rB = c - pB;

            double angularImpulseA = rA.cross(impulse);
            double angularImpulseB = rB.cross(impulse)*(-1);
            angVelA.omega += angularImpulseA / I_A.I;
            angVelB.omega += angularImpulseB / I_B.I;

            double fAngA = rA.cross(frictionImpulse);
            double fAngB = rB.cross(frictionImpulse)*(-1);
            angVelA.omega += fAngA / I_A.I;
            angVelB.omega += fAngB / I_B.I;

            angVelA.omega *= angularDamp;
            angVelB.omega *= angularDamp;
            registry.replace<Components::AngularVelocity>(col.a, angVelA);
            registry.replace<Components::AngularVelocity>(col.b, angVelB);
        }

        registry.replace<Components::Position>(col.a, posA);
        registry.replace<Components::Position>(col.b, posB);
        registry.replace<Components::Velocity>(col.a, velA);
        registry.replace<Components::Velocity>(col.b, velB);
    }
}