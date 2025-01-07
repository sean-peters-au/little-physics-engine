/**
 * @file contact_solver.cpp
 * @brief Implementation of velocity-based collision constraint solver
 *
 * Implements an iterative impulse solver that resolves collisions through
 * velocity corrections, handling both normal response and friction effects.
 *
 * Debugging statements have been added to help trace unexpected large impulses.
 */

#include "nbody/systems/rigid_body_collision/contact_solver.hpp"
#include <iostream>
#include <cmath>
#include <entt/entt.hpp>
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/core/profile.hpp"

static const double Gravity              = 9.81;
static const double RollingFrictionCoeff = 0.01;  // torque-based rolling friction
static const double StaticFrictionCoeff  = 0.7;   // static friction for resting contacts
static const double StaticVelocityThresh = 0.01;  // threshold for zero contact velocity
static const double RollingVelocityThresh= 0.05;  // threshold for small angular velocity

namespace RigidBodyCollision {

/**
 * @brief Checks if an entity has effectively infinite mass.
 */
static bool isInfiniteMass(const entt::registry &registry, entt::entity e)
{
    if (!registry.valid(e)) return false;
    if (!registry.all_of<Components::Mass>(e)) return false;
    const auto &m = registry.get<Components::Mass>(e);
    return (m.value > 1e29);
}

/**
 * @brief Check if an entity has valid Inertia and AngularVelocity, and I > ~0.
 */
static bool canRotate(entt::registry &registry, entt::entity e)
{
    if (!registry.all_of<Components::AngularVelocity, Components::Inertia>(e)) {
        return false;
    }
    const auto &I = registry.get<Components::Inertia>(e);
    return (I.I > 1e-12); // skip if effectively zero
}

/**
 * @brief Applies an angular impulse (torque) if canRotate(...) is true.
 * 
 * @param sign   Usually +1 or -1.
 * @param factor Additional scale factor for partial impulses (e.g. warm start).
 */
static void applyAngularImpulse(entt::registry &registry,
                                entt::entity entity,
                                const Vector &r,
                                const Vector &impulse,
                                double sign,
                                double factor,
                                double angularDamping)
{
    if (!canRotate(registry, entity)) {
        return;
    }
    auto &angVel  = registry.get<Components::AngularVelocity>(entity);
    auto &inertia = registry.get<Components::Inertia>(entity);

    double crossTerm = r.cross(impulse) / inertia.I;
    angVel.omega += sign * crossTerm * factor;
    angVel.omega *= angularDamping;
    registry.replace<Components::AngularVelocity>(entity, angVel);
}

/**
 * @brief For "resting" contacts, apply static friction or rolling friction 
 *        based on the object weight rather than collision impulse.
 * 
 * This function is called in the main solver loop after collision-based impulses.
 * If body A is on infinite-mass body B (like ground) or vice versa, 
 * compute friction from mg. 
 */
static void applyStaticAndRollingFriction(entt::registry &registry,
                                          ContactRef &c,
                                          double angularDamping)
{
    // Check if exactly one body is infinite mass (our "ground")
    bool aInf = isInfiniteMass(registry, c.a);
    bool bInf = isInfiniteMass(registry, c.b);
    if (!aInf && !bInf) return;
    if (aInf && bInf)   return;  // both infinite => no friction needed

    // Decide which is the finite body, which is ground
    entt::entity body   = aInf ? c.b : c.a;
    entt::entity ground = aInf ? c.a : c.b;

    // Ensure contact normal is sufficiently vertical (floor-like)
    // Here, we treat dot(n, up)=± as "vertical enough", but you can tweak threshold
    Vector up(0.0, 1.0);
    double dotUp = c.normal.dotProduct(up);
    // Example: if dotUp < 0.7, or dotUp is negative, we might consider it a wall
    // Here we want normal ~ down, so if dotUp < -0.7 => it's likely ground
    if (dotUp > -0.7) return; 

    // Retrieve relevant components once
    auto mass   = registry.get<Components::Mass>(body);
    auto vel    = registry.get<Components::Velocity>(body);
    auto pos    = registry.get<Components::Position>(body);
    auto &angVel= registry.get<Components::AngularVelocity>(body); // reference if we modify

    // Basic normal force (mass * g)
    double normalForce = mass.value * Gravity;

    // Contact offset from body center
    Vector rBody = Vector(c.contactPoint.x - pos.x,
                          c.contactPoint.y - pos.y);

    // Compute velocity at the contact point (linear + rotational)
    Vector velBodyAtContact = vel;
    if (canRotate(registry, body)) {
        velBodyAtContact += Vector(-angVel.omega * rBody.y,
                                    angVel.omega * rBody.x);
    }

    double speedAtContact = velBodyAtContact.length();

    // Static friction if linear speed is below a threshold
    if (speedAtContact < StaticVelocityThresh)
    {
        double staticFrictionLimit = StaticFrictionCoeff * normalForce;
        if (speedAtContact > 1e-9) {
            Vector dir = velBodyAtContact / speedAtContact;
            double impulseMag = mass.value * speedAtContact;
            if (impulseMag > staticFrictionLimit) {
                impulseMag = staticFrictionLimit;
            }
            Vector frictionImpulse = dir * (-impulseMag);

            // Apply friction to linear velocity
            auto newVel = vel + frictionImpulse * (1.0 / mass.value);
            registry.replace<Components::Velocity>(body, newVel);

            // Apply torque
            applyAngularImpulse(registry, body, rBody, frictionImpulse,
                                +1.0, 1.0, angularDamping);
        }
        return; // done if static friction is enough
    }

    // Rolling friction if the body is spinning above threshold
    if (canRotate(registry, body)) {
        // For example, measure spin directly from angVel
        double w = std::fabs(angVel.omega);
        if (w > RollingVelocityThresh) {
            double radiusApprox = 0.5; // or derive from bounding box
            double torqueMag = RollingFrictionCoeff * normalForce * radiusApprox;
            double sign = (angVel.omega > 0.0) ? -1.0 : +1.0;

            auto &inertia = registry.get<Components::Inertia>(body);
            double dOmega = torqueMag / inertia.I;
            angVel.omega += (sign * dOmega);
            angVel.omega *= angularDamping;
            registry.replace<Components::AngularVelocity>(body, angVel);
        }
    }
}

/**
 * @brief Part of warm-start: apply partial normal + friction impulses to linear & angular velocity.
 */
static void warmStartContact(entt::registry &registry,
                             ContactRef &c,
                             double frictionCoeff,
                             double angularDamping)
{
    if (!registry.valid(c.a) || !registry.valid(c.b)) {
        return;
    }

    // Grab mass and compute inverse mass
    auto &massA = registry.get<Components::Mass>(c.a);
    auto &massB = registry.get<Components::Mass>(c.b);
    double invMassA = (massA.value > 1e29) ? 0.0 : (1.0 / massA.value);
    double invMassB = (massB.value > 1e29) ? 0.0 : (1.0 / massB.value);

    // Retrieve linear velocities
    auto velA = registry.get<Components::Velocity>(c.a);
    auto velB = registry.get<Components::Velocity>(c.b);

    // Warm-start factor is the fraction of accumulated impulses we apply right away
    const double warmStartFactor = 0.2;

    // Partial normal impulse
    Vector Pn = c.normal * (c.normalImpulseAccum * warmStartFactor);
    velA -= (Pn * invMassA);
    velB += (Pn * invMassB);

    // Clamp friction to Coulomb limit
    double maxFriction = frictionCoeff * std::fabs(c.normalImpulseAccum);
    double tangentImpulse = c.tangentImpulseAccum;
    if (std::fabs(tangentImpulse) > maxFriction) {
        tangentImpulse = (tangentImpulse > 0.0) ? maxFriction : -maxFriction;
        c.tangentImpulseAccum = tangentImpulse;
    }

    // Tangential impulse
    Vector t(-c.normal.y, c.normal.x);
    Vector Pf = t * tangentImpulse;
    velA -= (Pf * invMassA);
    velB += (Pf * invMassB);

    // Write back updated linear velocities
    registry.replace<Components::Velocity>(c.a, velA);
    registry.replace<Components::Velocity>(c.b, velB);

    // Apply partial angular impulses
    auto posA = registry.get<Components::Position>(c.a);
    auto posB = registry.get<Components::Position>(c.b);
    Vector rA(c.contactPoint.x - posA.x, c.contactPoint.y - posA.y);
    Vector rB(c.contactPoint.x - posB.x, c.contactPoint.y - posB.y);

    // For body A, subtract normal and tangential impulses
    applyAngularImpulse(registry, c.a, rA, Pn, -1.0, warmStartFactor, angularDamping);
    applyAngularImpulse(registry, c.a, rA, Pf, -1.0, warmStartFactor, angularDamping);

    // For body B, add normal and tangential impulses
    applyAngularImpulse(registry, c.b, rB, Pn, +1.0, warmStartFactor, angularDamping);
    applyAngularImpulse(registry, c.b, rB, Pf, +1.0, warmStartFactor, angularDamping);
}

/**
 * @brief Solve normal impulse exactly like the old code, including the "replace + add" steps.
 */
static void solveNormalImpulse(entt::registry &registry,
                               ContactRef &c,
                               double invMassA,
                               double invMassB,
                               double invSum,
                               double angularDamping)
{
    // Get velocities
    auto velA = registry.get<Components::Velocity>(c.a);
    auto velB = registry.get<Components::Velocity>(c.b);

    Vector n = c.normal;
    Vector relVel = velB - velA;
    double vn = relVel.dotProduct(n);

    // Calculate normal impulse
    double jn = -vn / invSum;
    jn = std::max(0.0, jn);

    // Exactly replicate old code's "Replace (don't add)" then double assignment:
    //     c.normalImpulseAccum = jn;
    //     oldNormalImpulse     = c.normalImpulseAccum;
    //     newNormalImpulse     = oldNormalImpulse + jn;
    //     c.normalImpulseAccum = newNormalImpulse;
    //     dJn = newNormalImpulse - oldNormalImpulse;
    //     Pn = n * dJn;
    //
    // That effectively sets c.normalImpulseAccum = 2 * jn, but the impulse applied is just jn.
    c.normalImpulseAccum = jn;
    double oldNormalImpulse = c.normalImpulseAccum;  // = jn
    double newNormalImpulse = oldNormalImpulse + jn; // = 2 * jn
    c.normalImpulseAccum    = newNormalImpulse;      // c.normalImpulseAccum = 2 * jn
    double dJn = newNormalImpulse - oldNormalImpulse; // = jn

    Vector Pn = n * dJn; // the actual impulse to apply (jn)

    // Apply to linear velocity
    velA -= Pn * invMassA;
    velB += Pn * invMassB;

    // Write back velocities
    registry.replace<Components::Velocity>(c.a, velA);
    registry.replace<Components::Velocity>(c.b, velB);

    // Apply angular if valid
    auto posA = registry.get<Components::Position>(c.a);
    auto posB = registry.get<Components::Position>(c.b);
    Vector rA(c.contactPoint.x - posA.x, c.contactPoint.y - posA.y);
    Vector rB(c.contactPoint.x - posB.x, c.contactPoint.y - posB.y);

    // Body A gets negative impulse, B gets positive
    applyAngularImpulse(registry, c.a, rA, Pn, -1.0, 1.0, angularDamping);
    applyAngularImpulse(registry, c.b, rB, Pn, +1.0, 1.0, angularDamping);
}

/**
 * @brief Solve friction impulse, including old code's approach with velocities at contact point.
 */
static void solveFrictionImpulse(entt::registry &registry,
                                 ContactRef &c,
                                 double invMassA,
                                 double invMassB,
                                 double invSum,
                                 double frictionCoeff,
                                 double angularDamping)
{
    // 1) Grab the *actual* linear velocities from the registry
    auto velA = registry.get<Components::Velocity>(c.a);
    auto velB = registry.get<Components::Velocity>(c.b);

    // 2) Build "contact velocities" by adding rotational contributions in a local variable.
    auto posA = registry.get<Components::Position>(c.a);
    auto posB = registry.get<Components::Position>(c.b);

    Vector rA(c.contactPoint.x - posA.x, c.contactPoint.y - posA.y);
    Vector rB(c.contactPoint.x - posB.x, c.contactPoint.y - posB.y);

    Vector velA_contact = velA; // local copy for friction calc
    if (canRotate(registry, c.a)) {
        double wA = registry.get<Components::AngularVelocity>(c.a).omega;
        velA_contact += Vector(-wA * rA.y, wA * rA.x);
    }
    Vector velB_contact = velB; // local copy for friction calc
    if (canRotate(registry, c.b)) {
        double wB = registry.get<Components::AngularVelocity>(c.b).omega;
        velB_contact += Vector(-wB * rB.y, wB * rB.x);
    }

    // 3) Compute the relative velocity *at the contact* and isolate tangential velocity
    Vector n = c.normal;
    Vector relVelContact = velB_contact - velA_contact;
    Vector vt = relVelContact - (n * relVelContact.dotProduct(n));
    double vtLen = vt.length();
    if (vtLen < 1e-9) {
        return; // no tangential velocity => no friction
    }

    // 4) Tangential direction & effective mass (linear + rotational)
    Vector tDir = vt / vtLen;
    double angularMass = 0.0;

    if (canRotate(registry, c.a)) {
        auto &IA = registry.get<Components::Inertia>(c.a);
        double raCrossT = rA.cross(tDir);
        angularMass += (raCrossT * raCrossT) / IA.I;
    }
    if (canRotate(registry, c.b)) {
        auto &IB = registry.get<Components::Inertia>(c.b);
        double rbCrossT = rB.cross(tDir);
        angularMass += (rbCrossT * rbCrossT) / IB.I;
    }

    double effectiveMass = invSum + angularMass;
    if (effectiveMass < 1e-12) {
        return;
    }

    // 5) Compute the friction impulse along the tangent
    double jt = -(relVelContact.dotProduct(tDir)) / effectiveMass;

    // 6) Clamp friction impulse by the Coulomb limit => frictionCoeff * normalImpulseAccum
    double frictionLimit = frictionCoeff * std::fabs(c.normalImpulseAccum);
    double oldTangentImpulse = c.tangentImpulseAccum;
    double targetTangentImpulse = oldTangentImpulse + jt;
    double newTangentImpulse = std::max(-frictionLimit,
                                        std::min(frictionLimit, targetTangentImpulse));
    c.tangentImpulseAccum = newTangentImpulse;

    double dJt = newTangentImpulse - oldTangentImpulse;
    Vector Pf = tDir * dJt;  // friction impulse vector

    // 7) Apply that friction impulse to the *actual* linear velocities
    velA -= Pf * invMassA;
    velB += Pf * invMassB;

    registry.replace<Components::Velocity>(c.a, velA);
    registry.replace<Components::Velocity>(c.b, velB);

    // 8) Apply friction torque to angular velocities
    applyAngularImpulse(registry, c.a, rA, Pf, -1.0, 1.0, angularDamping);
    applyAngularImpulse(registry, c.b, rB, Pf, +1.0, 1.0, angularDamping);
}

/**
 * @brief Primary public function to solve all contact constraints, preserving original math.
 */
void ContactSolver::solveContactConstraints(entt::registry &registry,
                                            ContactManager &manager,
                                            double /*baumgarte*/,
                                            double /*slop*/)
{
    PROFILE_SCOPE("ContactSolver");
    const double angularDamping = 0.98;
    const double frictionCoeff  = 0.9; // global friction demo

    // Retrieve contact list (with warm-start impulses) from the manager
    auto &contacts = manager.getContactsForSolver();

    //--------------------------------------------------------------------------
    // Warm-Start Phase
    //--------------------------------------------------------------------------
    for (auto &c : contacts) {
        warmStartContact(registry, c, frictionCoeff, angularDamping);
    }

    //--------------------------------------------------------------------------
    // Main Solver Iterations
    //--------------------------------------------------------------------------
    const int solverIterations = 10;
    for (int iter = 0; iter < solverIterations; iter++) {
        for (auto &c : contacts) {
            if (!registry.valid(c.a) || !registry.valid(c.b)) {
                continue;
            }

            // Basic mass info
            auto &massA = registry.get<Components::Mass>(c.a);
            auto &massB = registry.get<Components::Mass>(c.b);
            double invMassA = (massA.value > 1e29) ? 0.0 : (1.0 / massA.value);
            double invMassB = (massB.value > 1e29) ? 0.0 : (1.0 / massB.value);
            double invSum   = invMassA + invMassB;
            if (invSum < 1e-12) {
                // both bodies effectively infinite => skip
                continue;
            }

            // 1) Solve Normal Impulse (identical to old code)
            solveNormalImpulse(registry, c, invMassA, invMassB, invSum, angularDamping);

            // 2) Solve Friction Impulse (same approach as old code)
            // solveFrictionImpulse(registry, c, invMassA, invMassB, invSum, frictionCoeff, angularDamping);

            applyStaticAndRollingFriction(registry, c, angularDamping);
        }
    }

    //--------------------------------------------------------------------------
    // Final: store impulses for next-frame warm start
    //--------------------------------------------------------------------------
    manager.applySolverResults(contacts);
}

} // namespace RigidBodyCollision