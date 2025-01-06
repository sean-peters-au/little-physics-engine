#include <cmath>

#include <entt/entt.hpp>

#include "nbody/systems/rigid_body_collision/contact_solver.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/math/vector_math.hpp"
#include "nbody/systems/rigid_body_collision/collision_data.hpp"

namespace RigidBodyCollision
{

void ContactSolver::solveContactConstraints(entt::registry &registry,
                                            const ContactManager &manager,
                                            double /*baumgarte*/,
                                            double /*slop*/)
{
    const double angularDamp = 0.98;
    const double gravity = 9.8;

    // Get all collisions
    const auto &manifold = manager.getCurrentManifold();

    for (auto &col : manifold.collisions) {
        if (!registry.valid(col.a) || !registry.valid(col.b)) continue;

        bool asleepA = false, asleepB = false;
        if (registry.any_of<Components::Sleep>(col.a)) {
            asleepA = registry.get<Components::Sleep>(col.a).asleep;
        }
        if (registry.any_of<Components::Sleep>(col.b)) {
            asleepB = registry.get<Components::Sleep>(col.b).asleep;
        }
        if (asleepA && asleepB) continue;

        auto &phaseA = registry.get<Components::ParticlePhase>(col.a);
        auto &phaseB = registry.get<Components::ParticlePhase>(col.b);
        if (phaseA.phase != Components::Phase::Solid && phaseB.phase != Components::Phase::Solid)
            continue;

        auto posA  = registry.get<Components::Position>(col.a);
        auto velA  = registry.get<Components::Velocity>(col.a);
        auto &massA = registry.get<Components::Mass>(col.a);

        auto posB  = registry.get<Components::Position>(col.b);
        auto velB  = registry.get<Components::Velocity>(col.b);
        auto &massB = registry.get<Components::Mass>(col.b);

        double invMassA = (massA.value > 1e29)? 0.0 : 1.0/massA.value;
        double invMassB = (massB.value > 1e29)? 0.0 : 1.0/massB.value;
        double invSum   = invMassA + invMassB;
        if (invSum < 1e-12) continue;

        Vector n = col.normal;
        double penetration = col.penetration;

        // Normal impulse
        Vector relVel = velB - velA;
        double normalSpeed = relVel.dotProduct(n);

        double restitution = 0.0;
        double jColl = 0.0;
        if (normalSpeed < 0) {
            jColl = -(1.0 + restitution)* normalSpeed / invSum;
        }

        // “resting normal” hack (like mg-based friction anchor)
        double restingN = 0.0;
        if (penetration > 0.0 && std::fabs(normalSpeed) < 0.1) {
            double combinedMass = massA.value + massB.value;
            Vector gravityDir(0.0, 1.0);
            double gravityComponent = std::fabs(gravityDir.dotProduct(n));
            restingN = combinedMass * gravity * gravityComponent;
        }

        double jTotalNormal = jColl + restingN;
        if (jTotalNormal < 0) jTotalNormal = 0;

        Vector impulseNormal = n * jColl;
        velA = velA - (impulseNormal * invMassA);
        velB = velB + (impulseNormal * invMassB);

        // friction
        double staticFrictionA  = 0.5, dynamicFrictionA  = 0.3;
        double staticFrictionB  = 0.5, dynamicFrictionB  = 0.3;
        if (registry.any_of<Components::Material>(col.a)) {
            auto &mA = registry.get<Components::Material>(col.a);
            staticFrictionA  = mA.staticFriction;
            dynamicFrictionA = mA.dynamicFriction;
        }
        if (registry.any_of<Components::Material>(col.b)) {
            auto &mB = registry.get<Components::Material>(col.b);
            staticFrictionB  = mB.staticFriction;
            dynamicFrictionB = mB.dynamicFriction;
        }
        double combinedStaticFriction  = 0.5*(staticFrictionA + staticFrictionB);
        double combinedDynamicFriction = 0.5*(dynamicFrictionA + dynamicFrictionB);

        double jRef = jTotalNormal;
        Vector tangent = relVel - n*(relVel.dotProduct(n));
        double tLen = tangent.length();
        if (tLen > 1e-9) {
            tangent = tangent / tLen;
        } else {
            tangent = Vector(0,0);
        }

        double tangentialSpeed = relVel.dotProduct(tangent);
        double jt = -tangentialSpeed / invSum;
        double frictionImpulseMag = std::fabs(jt);

        double frictionLimit = jRef * combinedStaticFriction;
        if (frictionImpulseMag < frictionLimit) {
            // static friction
            jt = -tangentialSpeed / invSum;
        } else {
            double dynFric = jRef * combinedDynamicFriction;
            jt = -(jt > 0 ? dynFric : -dynFric);
        }

        Vector frictionImpulse = tangent * jt;
        velA = velA - (frictionImpulse * invMassA);
        velB = velB + (frictionImpulse * invMassB);

        // Angular impulses
        if (registry.all_of<Components::AngularVelocity, Components::Inertia>(col.a) &&
            registry.all_of<Components::AngularVelocity, Components::Inertia>(col.b)) {

            auto &angA = registry.get<Components::AngularVelocity>(col.a);
            auto &I_A  = registry.get<Components::Inertia>(col.a);
            auto &angB = registry.get<Components::AngularVelocity>(col.b);
            auto &I_B  = registry.get<Components::Inertia>(col.b);

            Vector c = col.contactPoint;  // world contact
            Vector pA(posA.x, posA.y);
            Vector pB(posB.x, posB.y);

            Vector rA = c - pA;
            Vector rB = c - pB;

            double angImpA = rA.cross(impulseNormal);
            double angImpB = rB.cross(impulseNormal);
            angA.omega += angImpA / I_A.I;
            angB.omega += angImpB / I_B.I;

            double fAngA = rA.cross(frictionImpulse);
            double fAngB = rB.cross(frictionImpulse);
            angA.omega += fAngA / I_A.I;
            angB.omega += fAngB / I_B.I;

            // damping
            angA.omega *= angularDamp;
            angB.omega *= angularDamp;
            registry.replace<Components::AngularVelocity>(col.a, angA);
            registry.replace<Components::AngularVelocity>(col.b, angB);
        }

        // store updated velocities
        registry.replace<Components::Velocity>(col.a, velA);
        registry.replace<Components::Velocity>(col.b, velB);
    } // end for collisions
}

} // namespace RigidBodyCollision