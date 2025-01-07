/**
 * @file contact_solver.cpp
 * @brief Implementation of velocity-based collision constraint solver
 *
 * Implements an iterative impulse solver that resolves collisions through
 * velocity corrections, handling both normal response and friction effects,
 * with integrated mg-based friction fallback and rolling friction.
 */

#include "nbody/systems/rigid_body_collision/contact_solver.hpp"
#include <iostream>
#include <cmath>
#include <entt/entt.hpp>
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/core/profile.hpp"

static const double Gravity              = 9.81;
static const double RollingFrictionCoeff = 0.01; 
static const double StaticFrictionCoeff  = 0.7;   
static const double StaticVelocityThresh = 0.01;  
static const double RollingVelocityThresh= 0.05;  

namespace RigidBodyCollision {

static bool isInfiniteMass(const entt::registry &registry, entt::entity e)
{
    if (!registry.valid(e)) return false;
    if (!registry.all_of<Components::Mass>(e)) return false;
    const auto &m = registry.get<Components::Mass>(e);
    return (m.value > 1e29);
}

static bool canRotate(entt::registry &registry, entt::entity e)
{
    if (!registry.all_of<Components::AngularVelocity, Components::Inertia>(e)) return false;
    const auto &I = registry.get<Components::Inertia>(e);
    return (I.I > 1e-12);
}

static void applyAngularImpulse(entt::registry &registry,
                                entt::entity entity,
                                const Vector &r,
                                const Vector &impulse,
                                double sign,
                                double factor,
                                double angularDamping)
{
    std::cout << "applyAngularImpulse" << std::endl;
    if (!canRotate(registry, entity)) return;
    auto &angVel  = registry.get<Components::AngularVelocity>(entity);
    auto &inertia = registry.get<Components::Inertia>(entity);

    std::cout << "r: " << r.x << ", " << r.y << std::endl;
    std::cout << "impulse: " << impulse.x << ", " << impulse.y << std::endl;
    double crossTerm = r.cross(impulse) / inertia.I;
    std::cout << "crossTerm: " << crossTerm << std::endl;
    std::cout << "sign: " << sign << std::endl;
    std::cout << "factor: " << factor << std::endl;
    std::cout << "angularDamping: " << angularDamping << std::endl;
    angVel.omega += sign * crossTerm * factor;
    angVel.omega *= angularDamping;
    std::cout << "angVel.omega: " << angVel.omega << std::endl;

    registry.replace<Components::AngularVelocity>(entity, angVel);
}

static void warmStartContact(entt::registry &registry,
                             ContactRef &c,
                             double frictionCoeff,
                             double angularDamping)
{
    if (!registry.valid(c.a) || !registry.valid(c.b)) {
        return;
    }

    auto &massA = registry.get<Components::Mass>(c.a);
    auto &massB = registry.get<Components::Mass>(c.b);
    double invMassA = (massA.value > 1e29) ? 0.0 : (1.0 / massA.value);
    double invMassB = (massB.value > 1e29) ? 0.0 : (1.0 / massB.value);

    auto velA = registry.get<Components::Velocity>(c.a);
    auto velB = registry.get<Components::Velocity>(c.b);

    const double warmStartFactor = 0.2;

    Vector Pn = c.normal * (c.normalImpulseAccum * warmStartFactor);
    velA -= (Pn * invMassA);
    velB += (Pn * invMassB);

    double maxFriction = frictionCoeff * std::fabs(c.normalImpulseAccum);
    double tangentImpulse = c.tangentImpulseAccum;
    if (std::fabs(tangentImpulse) > maxFriction) {
        tangentImpulse = (tangentImpulse > 0.0) ? maxFriction : -maxFriction;
        c.tangentImpulseAccum = tangentImpulse;
    }

    Vector t(-c.normal.y, c.normal.x);
    Vector Pf = t * tangentImpulse;
    velA -= (Pf * invMassA);
    velB += (Pf * invMassB);

    registry.replace<Components::Velocity>(c.a, velA);
    registry.replace<Components::Velocity>(c.b, velB);

    auto posA = registry.get<Components::Position>(c.a);
    auto posB = registry.get<Components::Position>(c.b);
    Vector rA(c.contactPoint.x - posA.x, c.contactPoint.y - posA.y);
    Vector rB(c.contactPoint.x - posB.x, c.contactPoint.y - posB.y);

    applyAngularImpulse(registry, c.a, rA, Pn, -1.0, warmStartFactor, angularDamping);
    applyAngularImpulse(registry, c.a, rA, Pf, -1.0, warmStartFactor, angularDamping);
    applyAngularImpulse(registry, c.b, rB, Pn, +1.0, warmStartFactor, angularDamping);
    applyAngularImpulse(registry, c.b, rB, Pf, +1.0, warmStartFactor, angularDamping);
}

static void solveNormalImpulse(entt::registry &registry,
                               ContactRef &c,
                               double invMassA,
                               double invMassB,
                               double invSum,
                               double angularDamping)
{
    auto velA = registry.get<Components::Velocity>(c.a);
    auto velB = registry.get<Components::Velocity>(c.b);

    Vector n = c.normal;
    Vector relVel = velB - velA;
    double vn = relVel.dotProduct(n);

    double jn = -vn / invSum;
    jn = std::max(0.0, jn);

    // Accumulate impulse for this iteration
    double oldNormalImpulse = c.normalImpulseAccum;
    double newNormalImpulse = oldNormalImpulse + jn;
    c.normalImpulseAccum = newNormalImpulse;
    
    double dJn = newNormalImpulse - oldNormalImpulse;
    Vector Pn = n * dJn;

    // Apply impulse
    velA -= Pn * invMassA;
    velB += Pn * invMassB;

    registry.replace<Components::Velocity>(c.a, velA);
    registry.replace<Components::Velocity>(c.b, velB);
}

static void solveFrictionImpulse(entt::registry &registry,
                                 ContactRef &c,
                                 double invMassA,
                                 double invMassB,
                                 double invSum,
                                 double frictionCoeff,
                                 double angularDamping)
{
    auto velA = registry.get<Components::Velocity>(c.a);
    auto velB = registry.get<Components::Velocity>(c.b);

    auto posA = registry.get<Components::Position>(c.a);
    auto posB = registry.get<Components::Position>(c.b);

    Vector rA(c.contactPoint.x - posA.x, c.contactPoint.y - posA.y);
    Vector rB(c.contactPoint.x - posB.x, c.contactPoint.y - posB.y);

    // Build local contact velocities
    Vector velA_contact = velA;
    if (canRotate(registry, c.a)) {
        double wA = registry.get<Components::AngularVelocity>(c.a).omega;
        velA_contact += Vector(-wA * rA.y, wA * rA.x);
    }
    Vector velB_contact = velB;
    if (canRotate(registry, c.b)) {
        double wB = registry.get<Components::AngularVelocity>(c.b).omega;
        velB_contact += Vector(-wB * rB.y, wB * rB.x);
    }

    std::cout << "velA_contact: " << velA_contact.x << ", " << velA_contact.y << std::endl;
    std::cout << "velB_contact: " << velB_contact.x << ", " << velB_contact.y << std::endl;

    Vector n = c.normal;
    std::cout << "n: " << n.x << ", " << n.y << std::endl;
    Vector relVelContact = velB_contact - velA_contact;
    Vector vt = relVelContact - n * relVelContact.dotProduct(n);
    double vtLen = vt.length();
    if (vtLen < 1e-9) {
        std::cout << "No tangential motion => no friction" << std::endl;
        return;
    }
    Vector tDir = vt / vtLen;

    // Effective mass (linear + rotational)
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
        std::cout << "Both infinite or near zero mass => skip" << std::endl;
        return;
    }

    // Ideal friction impulse
    double jt = -(relVelContact.dotProduct(tDir)) / effectiveMass;
    std::cout << "jt: " << jt << std::endl;

    // Base friction limit from collision impulse
    double frictionLimit = frictionCoeff * std::fabs(c.normalImpulseAccum);
    std::cout << "frictionLimit: " << frictionLimit << std::endl;

    // mg-based fallback logic (only if exactly one body is infinite & contact is truly supporting)
    bool infA = isInfiniteMass(registry, c.a);
    bool infB = isInfiniteMass(registry, c.b);
    std::cout << "infA: " << infA << ", infB: " << infB << std::endl;

    if ((infA ^ infB)) {
        Vector up(0.0, 1.0);
        double dotUp = std::fabs(n.dotProduct(up));
        // Get the finite body
        entt::entity finiteBody = infA ? c.b : c.a;
        double massVal = registry.get<Components::Mass>(finiteBody).value;
        double mg = massVal * Gravity;

        // If normalImpulse is truly supporting a big fraction of mg, apply fallback
        // E.g. if normalImpulse > 0.1 * mg => it's pinned enough to floor.
        if (dotUp > 0.7 && frictionLimit < 1e-9 && vtLen > 1e-9) {
            double normalImpulseVal = std::fabs(c.normalImpulseAccum);
            if (normalImpulseVal > 0.1 * mg) {
                std::cout << "mg-based fallback" << std::endl;
                double normalForce = mg;
                frictionLimit = frictionCoeff * normalForce;
            } else {
                std::cout << "Skipping mg-based fallback; normalImpulse too small." << std::endl;
            }
        }
    }

    // Also clamp friction by how much is needed to bring vt to zero
    double neededImpulse = effectiveMass * vtLen;
    std::cout << "neededImpulse: " << neededImpulse << std::endl;
    frictionLimit = std::min(frictionLimit, neededImpulse);
    std::cout << "frictionLimit: " << frictionLimit << std::endl;

    double oldTangentImpulse = c.tangentImpulseAccum;
    double targetTangentImpulse = oldTangentImpulse + jt;
    double newTangentImpulse = std::max(-frictionLimit,
                                        std::min(frictionLimit, targetTangentImpulse));
    c.tangentImpulseAccum = newTangentImpulse;
    std::cout << "newTangentImpulse: " << newTangentImpulse << std::endl;
    double dJt = newTangentImpulse - oldTangentImpulse;
    std::cout << "dJt: " << dJt << std::endl;
    Vector Pf = tDir * dJt;
    std::cout << "Pf: " << Pf.x << ", " << Pf.y << std::endl;

    // If Pf dot vt > 0 => friction is adding velocity => zero it
    if (vt.dotProduct(Pf) > 1e-12) {
        std::cout << "Pf·vt > 0 => friction adds velocity => zero it" << std::endl;
        Pf = Vector(0,0);
        c.tangentImpulseAccum = oldTangentImpulse;
        dJt = 0.0;
    }

    velA -= Pf * invMassA;
    velB += Pf * invMassB;
    std::cout << "velA: " << velA.x << ", " << velA.y << std::endl;
    std::cout << "velB: " << velB.x << ", " << velB.y << std::endl;

    registry.replace<Components::Velocity>(c.a, velA);
    registry.replace<Components::Velocity>(c.b, velB);

    applyAngularImpulse(registry, c.a, rA, Pf, -1.0, 1.0, angularDamping);
    applyAngularImpulse(registry, c.b, rB, Pf, +1.0, 1.0, angularDamping);

    // Rolling friction with sign clamp
    if (canRotate(registry, c.a)) {
        std::cout << "canRotate(registry, c.a)" << std::endl;
        double wA = registry.get<Components::AngularVelocity>(c.a).omega;
        if (std::fabs(wA) > RollingVelocityThresh) {
            std::cout << "wA > RollingVelocityThresh" << std::endl;
            auto &inertiaA = registry.get<Components::Inertia>(c.a);
            auto &angVelA  = registry.get<Components::AngularVelocity>(c.a);
            double sign = (wA > 0.0) ? -1.0 : +1.0;
            double dOmega = (RollingFrictionCoeff * registry.get<Components::Mass>(c.a).value * Gravity * 0.5)
                            / inertiaA.I;

            // If applying sign*dOmega would overshoot zero, clamp to zero
            if (std::fabs(dOmega) > std::fabs(wA)) {
                dOmega = wA * (-1.0); 
            }

            std::cout << "angVelA.omega += sign * dOmega: " << (sign * dOmega) << std::endl;
            angVelA.omega += sign * dOmega;
            angVelA.omega *= angularDamping;
            std::cout << "angVelA.omega: " << angVelA.omega << std::endl;
            registry.replace<Components::AngularVelocity>(c.a, angVelA);
        }
    }
    if (canRotate(registry, c.b)) {
        std::cout << "canRotate(registry, c.b)" << std::endl;
        double wB = registry.get<Components::AngularVelocity>(c.b).omega;
        if (std::fabs(wB) > RollingVelocityThresh) {
            std::cout << "wB > RollingVelocityThresh" << std::endl;
            auto &inertiaB = registry.get<Components::Inertia>(c.b);
            auto &angVelB  = registry.get<Components::AngularVelocity>(c.b);
            double sign = (wB > 0.0) ? -1.0 : +1.0;
            double dOmega = (RollingFrictionCoeff * registry.get<Components::Mass>(c.b).value * Gravity * 0.5)
                            / inertiaB.I;

            // If applying sign*dOmega would overshoot zero, clamp to zero
            if (std::fabs(dOmega) > std::fabs(wB)) {
                dOmega = wB * (-1.0);
            }

            std::cout << "angVelB.omega += sign * dOmega: " << (sign * dOmega) << std::endl;
            angVelB.omega += sign * dOmega;
            angVelB.omega *= angularDamping;
            std::cout << "angVelB.omega: " << angVelB.omega << std::endl;
            registry.replace<Components::AngularVelocity>(c.b, angVelB);
        }
    }
}

void ContactSolver::solveContactConstraints(entt::registry &registry,
                                            ContactManager &manager,
                                            double /*baumgarte*/,
                                            double /*slop*/)
{
    PROFILE_SCOPE("ContactSolver");
    const double angularDamping = 0.98;
    const double frictionCoeff  = 0.9; 

    auto &contacts = manager.getContactsForSolver();

    // Reset accumulated impulses at start of physics step
    for (auto &c : contacts) {
        c.normalImpulseAccum = 0.0;
        c.tangentImpulseAccum = 0.0;
    }

    // Warm start using previous frame's impulses
    for (auto &c : contacts) {
        warmStartContact(registry, c, frictionCoeff, angularDamping);
    }

    // Solve iterations
    const int solverIterations = 10;
    for (int iter = 0; iter < solverIterations; iter++) {
        for (auto &c : contacts) {
            if (!registry.valid(c.a) || !registry.valid(c.b)) {
                continue;
            }

            auto &massA = registry.get<Components::Mass>(c.a);
            auto &massB = registry.get<Components::Mass>(c.b);
            double invMassA = (massA.value > 1e29) ? 0.0 : (1.0 / massA.value);
            double invMassB = (massB.value > 1e29) ? 0.0 : (1.0 / massB.value);
            double invSum   = invMassA + invMassB;
            if (invSum < 1e-12) {
                continue;
            }

            // Store impulses for this iteration
            double oldNormalImpulse = c.normalImpulseAccum;
            solveNormalImpulse(registry, c, invMassA, invMassB, invSum, angularDamping);
            
            // Use total accumulated normal impulse for friction calculation
            solveFrictionImpulse(registry, c, invMassA, invMassB, invSum, frictionCoeff, angularDamping);
        }
    }

    // Store final impulses for next frame's warm start
    manager.applySolverResults(contacts);
}

} // namespace RigidBodyCollision