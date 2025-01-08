/**
 * @file contact_solver.cpp
 * @brief Implementation of a more complete iterative solver with position correction,
 *        friction clamping, and rolling friction tweaks to reduce indefinite rotation.
 */

#include "nbody/systems/rigid_body_collision/contact_solver.hpp"
#include <iostream>
#include <cmath>
#include <entt/entt.hpp>
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/core/profile.hpp"
#include "nbody/systems/rigid_body_collision/contact_manager.hpp"

namespace RigidBodyCollision {

// --------------------- Tuning -------------------------
static const double Gravity               = 9.81;
static const double GlobalFrictionCoeff   = 0.9;  // tangential friction
static const double RollingFrictionCoeff  = 0.005;// smaller to avoid big flips
static const double RollingCutoff         = 0.05; // if |w| < 0.05, no rolling friction
static const double AngularDamping        = 0.98; // damp spin each iteration
static const double BaumgarteFactor       = 0.2;  // position correction factor
static const double PenetrationSlop       = 0.001;// small overlap tolerance
static const double SleepThresholdLin     = 0.05; // if velocity < 0.05 for frames => set 0
static const double SleepThresholdAng     = 0.05; // if angular velocity < 0.05 => set 0
static const int    SleepFramesNeeded     = 60;   // frames below threshold => sleep
// ------------------------------------------------------

/** Checks if mass is effectively infinite */
static bool isInfiniteMass(const entt::registry &registry, entt::entity e)
{
    if (!registry.valid(e)) return false;
    if (!registry.all_of<Components::Mass>(e)) return false;
    auto &m = registry.get<Components::Mass>(e);
    return (m.value > 1e29);
}

/** Checks if body can rotate (has AngularVelocity + Inertia) and inertia > 0 */
static bool canRotate(entt::registry &registry, entt::entity e)
{
    if (!registry.all_of<Components::AngularVelocity, Components::Inertia>(e)) {
        return false;
    }
    auto &I = registry.get<Components::Inertia>(e);
    return (I.I > 1e-12);
}

/**
 * @brief apply an angular impulse to entity
 */
static void applyAngularImpulse(entt::registry &registry,
                                entt::entity entity,
                                const Vector &r,
                                const Vector &impulse,
                                double sign,
                                double factor,
                                double angularDamping)
{
    if (!canRotate(registry, entity)) return;

    auto &angVel  = registry.get<Components::AngularVelocity>(entity);
    auto &inertia = registry.get<Components::Inertia>(entity);

    double crossVal = r.cross(impulse) / inertia.I;
    double oldOmega = angVel.omega;

    angVel.omega += sign * crossVal * factor;
    angVel.omega *= angularDamping;

    std::cout << "[applyAngularImpulse] e=" << (int)entity
              << " cross=" << crossVal
              << " oldOmega=" << oldOmega
              << " newOmega=" << angVel.omega
              << std::endl;

    registry.replace<Components::AngularVelocity>(entity, angVel);
}

/**
 * @brief Rolling friction that bleeds off spin without flipping sign
 */
static void applyRollingFriction(entt::registry &registry,
                                 entt::entity e)
{
    if (!canRotate(registry, e)) return;
    auto &angVel = registry.get<Components::AngularVelocity>(e);
    auto &inertia= registry.get<Components::Inertia>(e);
    auto &mass  = registry.get<Components::Mass>(e);
    if (mass.value > 1e29) return; // infinite mass => skip

    double w = angVel.omega;
    if (std::fabs(w) < RollingCutoff) {
        return; // no rolling friction if nearly zero
    }
    double sign   = (w > 0.0) ? -1.0 : +1.0;
    double torque = RollingFrictionCoeff * mass.value * Gravity * 0.5;
    double alpha  = torque / inertia.I;

    // clamp so we never flip sign in one iteration
    if (std::fabs(alpha) > std::fabs(w)) {
        alpha = w * (-1.0);
    }
    double oldW = w;
    angVel.omega += sign * alpha;
    angVel.omega *= AngularDamping;

    std::cout << "[rollingFriction] e=" << (int)e
              << " oldW=" << oldW
              << " alpha=" << alpha
              << " newW=" << angVel.omega
              << std::endl;

    registry.replace<Components::AngularVelocity>(e, angVel);
}

/**
 * @brief Warm-start with partial impulses from last frame
 */
static void warmStartContact(entt::registry &registry,
                             ContactRef &c,
                             double frictionCoeff)
{
    if (!registry.valid(c.a) || !registry.valid(c.b)) return;

    auto velA = registry.get<Components::Velocity>(c.a);
    auto velB = registry.get<Components::Velocity>(c.b);
    double massA = registry.get<Components::Mass>(c.a).value;
    double massB = registry.get<Components::Mass>(c.b).value;
    double invA  = (massA > 1e29) ? 0.0 : (1.0 / massA);
    double invB  = (massB > 1e29) ? 0.0 : (1.0 / massB);

    auto posA = registry.get<Components::Position>(c.a);
    auto posB = registry.get<Components::Position>(c.b);

    Vector rA(c.contactPoint.x - posA.x, c.contactPoint.y - posA.y);
    Vector rB(c.contactPoint.x - posB.x, c.contactPoint.y - posB.y);

    double warmFactor = 0.2;
    Vector Pn = c.normal * (c.normalImpulseAccum * warmFactor);
    velA -= Pn * invA;
    velB += Pn * invB;

    double frictionLimit = frictionCoeff * std::fabs(c.normalImpulseAccum);
    double oldTangent    = c.tangentImpulseAccum;

    if (std::fabs(oldTangent) > frictionLimit) {
        oldTangent = (oldTangent > 0.0) ? frictionLimit : -frictionLimit;
        c.tangentImpulseAccum = oldTangent;
    }
    Vector tDir(-c.normal.y, c.normal.x);
    Vector Pf = tDir * (oldTangent * warmFactor);

    velA -= Pf * invA;
    velB += Pf * invB;

    registry.replace<Components::Velocity>(c.a, velA);
    registry.replace<Components::Velocity>(c.b, velB);

    applyAngularImpulse(registry, c.a, rA, Pn, -1.0, warmFactor, AngularDamping);
    applyAngularImpulse(registry, c.a, rA, Pf, -1.0, warmFactor, AngularDamping);
    applyAngularImpulse(registry, c.b, rB, Pn, +1.0, warmFactor, AngularDamping);
    applyAngularImpulse(registry, c.b, rB, Pf, +1.0, warmFactor, AngularDamping);

    std::cout << "[warmStartContact] eA=" << (int)c.a
              << " eB=" << (int)c.b
              << " normalImpulse=" << c.normalImpulseAccum
              << " tangentImpulse=" << c.tangentImpulseAccum
              << std::endl;
}

/**
 * @brief Solve normal impulse (includes torque for off-center hits)
 */
static void solveNormalConstraint(entt::registry &registry,
                                  ContactRef &c,
                                  double frictionCoeff)
{
    auto velA = registry.get<Components::Velocity>(c.a);
    auto velB = registry.get<Components::Velocity>(c.b);

    double massA = registry.get<Components::Mass>(c.a).value;
    double massB = registry.get<Components::Mass>(c.b).value;
    double invA  = (massA > 1e29) ? 0.0 : (1.0 / massA);
    double invB  = (massB > 1e29) ? 0.0 : (1.0 / massB);

    auto posA = registry.get<Components::Position>(c.a);
    auto posB = registry.get<Components::Position>(c.b);

    double wA = 0.0, wB = 0.0;
    if (canRotate(registry, c.a)) {
        wA = registry.get<Components::AngularVelocity>(c.a).omega;
    }
    if (canRotate(registry, c.b)) {
        wB = registry.get<Components::AngularVelocity>(c.b).omega;
    }

    Vector rA(c.contactPoint.x - posA.x, c.contactPoint.y - posA.y);
    Vector rB(c.contactPoint.x - posB.x, c.contactPoint.y - posB.y);

    Vector velA_contact = velA + Vector(-wA*rA.y, wA*rA.x);
    Vector velB_contact = velB + Vector(-wB*rB.y, wB*rB.x);

    Vector n = c.normal;
    Vector relVel = velB_contact - velA_contact;
    double vn = relVel.dotProduct(n);

    // add rotational mass
    double angularMass = 0.0;
    if (canRotate(registry, c.a)) {
        double raCrossN = rA.cross(n);
        double I_A      = registry.get<Components::Inertia>(c.a).I;
        angularMass += (raCrossN * raCrossN) / I_A;
    }
    if (canRotate(registry, c.b)) {
        double rbCrossN = rB.cross(n);
        double I_B      = registry.get<Components::Inertia>(c.b).I;
        angularMass += (rbCrossN * rbCrossN) / I_B;
    }

    double normalMass = invA + invB + angularMass;
    if (normalMass < 1e-12) {
        return; // both infinite => skip
    }

    double jn = -vn / normalMass;
    if (jn < 0.0) {
        jn = 0.0;
    }

    double oldImpulse = c.normalImpulseAccum;
    double newImpulse = oldImpulse + jn;
    if (newImpulse < 0.0) {
        newImpulse = 0.0;
    }
    c.normalImpulseAccum = newImpulse;

    double dJn = newImpulse - oldImpulse;
    if (std::fabs(dJn) < 1e-12) {
        return; 
    }
    Vector Pn = n * dJn;

    velA -= Pn * invA;
    velB += Pn * invB;

    registry.replace<Components::Velocity>(c.a, velA);
    registry.replace<Components::Velocity>(c.b, velB);

    applyAngularImpulse(registry, c.a, rA, Pn, -1.0, 1.0, AngularDamping);
    applyAngularImpulse(registry, c.b, rB, Pn, +1.0, 1.0, AngularDamping);

    std::cout << "[solveNormal] eA=" << (int)c.a
              << " eB=" << (int)c.b
              << " oldN=" << oldImpulse
              << " newN=" << newImpulse
              << " dJn=" << dJn
              << " vn=" << vn
              << std::endl;
}

/**
 * @brief Solve friction impulse using normalImpulse as limit
 */
static void solveFrictionConstraint(entt::registry &registry,
                                    ContactRef &c,
                                    double frictionCoeff)
{
    auto velA = registry.get<Components::Velocity>(c.a);
    auto velB = registry.get<Components::Velocity>(c.b);

    double massA = registry.get<Components::Mass>(c.a).value;
    double massB = registry.get<Components::Mass>(c.b).value;
    double invA  = (massA > 1e29) ? 0.0 : (1.0 / massA);
    double invB  = (massB > 1e29) ? 0.0 : (1.0 / massB);

    auto posA = registry.get<Components::Position>(c.a);
    auto posB = registry.get<Components::Position>(c.b);

    double wA = canRotate(registry, c.a)
        ? registry.get<Components::AngularVelocity>(c.a).omega : 0.0;
    double wB = canRotate(registry, c.b)
        ? registry.get<Components::AngularVelocity>(c.b).omega : 0.0;

    Vector rA(c.contactPoint.x - posA.x, c.contactPoint.y - posA.y);
    Vector rB(c.contactPoint.x - posB.x, c.contactPoint.y - posB.y);

    Vector velA_contact = velA + Vector(-wA*rA.y, wA*rA.x);
    Vector velB_contact = velB + Vector(-wB*rB.y, wB*rB.x);

    Vector rel = velB_contact - velA_contact;
    Vector n = c.normal;
    double dotN = rel.dotProduct(n);

    // tangential velocity
    Vector vt = rel - (n * dotN);
    double vtLen = vt.length();
    if (vtLen < 1e-9) {
        return;
    }

    Vector tDir = vt / vtLen;
    // friction effective mass
    double angularMass = 0.0;
    if (canRotate(registry, c.a)) {
        double raCrossT = rA.cross(tDir);
        double I_A = registry.get<Components::Inertia>(c.a).I;
        angularMass += (raCrossT*raCrossT)/I_A;
    }
    if (canRotate(registry, c.b)) {
        double rbCrossT = rB.cross(tDir);
        double I_B = registry.get<Components::Inertia>(c.b).I;
        angularMass += (rbCrossT*rbCrossT)/I_B;
    }
    double frictionMass = invA + invB + angularMass;
    if (frictionMass < 1e-12) {
        return;
    }

    double jt = -(rel.dotProduct(tDir))/frictionMass;

    double limit = frictionCoeff * std::fabs(c.normalImpulseAccum);
    double oldTangent = c.tangentImpulseAccum;
    double newTangent = oldTangent + jt;
    if (newTangent > limit) {
        newTangent = limit;
    } else if (newTangent < -limit) {
        newTangent = -limit;
    }
    c.tangentImpulseAccum = newTangent;

    double dJt = newTangent - oldTangent;
    if (std::fabs(dJt)<1e-12) {
        return;
    }
    Vector Pf = tDir * dJt;

    velA -= Pf * invA;
    velB += Pf * invB;
    registry.replace<Components::Velocity>(c.a, velA);
    registry.replace<Components::Velocity>(c.b, velB);

    applyAngularImpulse(registry, c.a, rA, Pf, -1.0, 1.0, AngularDamping);
    applyAngularImpulse(registry, c.b, rB, Pf, +1.0, 1.0, AngularDamping);

    std::cout << "[solveFriction] eA="<< (int)c.a
              << " eB="<< (int)c.b
              << " oldTangent="<< oldTangent
              << " newTangent="<< newTangent
              << " vtLen="<< vtLen
              << " frictionLimit="<< limit
              << std::endl;
}

/**
 * @brief Basic position-based correction to reduce overlap each iteration (Baumgarte).
 */
static void solvePosition(entt::registry &registry, ContactRef &c)
{
    // If penetration is small, skip
    if (c.penetration < PenetrationSlop) {
        return;
    }

    // get masses
    double mA = registry.get<Components::Mass>(c.a).value;
    double mB = registry.get<Components::Mass>(c.b).value;
    double invA = (mA > 1e29) ? 0.0 : (1.0 / mA);
    double invB = (mB > 1e29) ? 0.0 : (1.0 / mB);
    double invSum = invA + invB;
    if (invSum<1e-12) {
        return; // both infinite => no correction
    }
    // portion
    double correction = (c.penetration - PenetrationSlop) * BaumgarteFactor / invSum;

    Vector corr = c.normal * correction;

    auto posA = registry.get<Components::Position>(c.a);
    auto posB = registry.get<Components::Position>(c.b);

    posA.x -= corr.x * invA;
    posA.y -= corr.y * invA;
    posB.x += corr.x * invB;
    posB.y += corr.y * invB;

    registry.replace<Components::Position>(c.a, posA);
    registry.replace<Components::Position>(c.b, posB);

    std::cout << "[solvePosition] eA="<< (int)c.a
              << " eB="<< (int)c.b
              << " penetration="<< c.penetration
              << " correction="<< correction
              << std::endl;
}

/**
 * @brief Minor "sleep" approach: if velocity/omega < threshold for X frames => zero them out
 */
static void checkSleep(entt::registry &registry, ContactRef &c)
{
    // For both A & B if they're finite mass
    auto trySleep = [&](entt::entity e)
    {
        if (!registry.valid(e)) return;
        if (!registry.all_of<Components::Sleep, Components::Velocity, Components::Mass>(e)) return;
        auto &slp = registry.get<Components::Sleep>(e);
        auto &m   = registry.get<Components::Mass>(e);
        if (m.value>1e29) return; // infinite mass => no sleep

        auto vel = registry.get<Components::Velocity>(e);
        double speed = vel.length();
        double w=0.0;
        if (canRotate(registry, e)) {
            w = std::fabs(registry.get<Components::AngularVelocity>(e).omega);
        }
        if (speed< SleepThresholdLin && w< SleepThresholdAng) {
            slp.sleepCounter++;
            if (slp.sleepCounter>= SleepFramesNeeded) {
                // zero them out
                registry.replace<Components::Velocity>(e, Components::Velocity(0.0,0.0));
                if (canRotate(registry, e)) {
                    auto &angV = registry.get<Components::AngularVelocity>(e);
                    angV.omega=0.0;
                    registry.replace<Components::AngularVelocity>(e, angV);
                }
                std::cout<<"[sleep] entity="<<(int)e<<" is now sleeping\n";
            }
        } else {
            slp.sleepCounter=0;
        }
    };
    trySleep(c.a);
    trySleep(c.b);
}

//---------------------------------------------------------------
// The main public function
//---------------------------------------------------------------
void ContactSolver::solveContactConstraints(entt::registry &registry,
                                            ContactManager &manager,
                                            double /*baumgarte*/,
                                            double /*slop*/)
{
    // First, clear any existing contact visualization entities
    auto oldContacts = registry.view<RigidBodyCollision::ContactRef>();
    registry.destroy(oldContacts.begin(), oldContacts.end());

    // Retrieve current contact set
    auto &contacts = manager.getContactsForSolver();

    // Create visualization entities for each contact
    for (const auto &c : contacts) {
        auto contactEntity = registry.create();
        registry.emplace<RigidBodyCollision::ContactRef>(contactEntity, c);
    }

    // Warm Start
    for (auto &c : contacts) {
        warmStartContact(registry, c, GlobalFrictionCoeff);
    }

    // multiple velocity solver iterations
    const int velocityIterations = 3;
    for (int iter=0; iter<velocityIterations; iter++) {
        std::cout<<"=== Velocity Solver Iter "<<iter<<" ===\n";
        for (auto &c : contacts) {
            if (!registry.valid(c.a) || !registry.valid(c.b)) {
                continue;
            }
            solveNormalConstraint(registry, c, GlobalFrictionCoeff);
            solveFrictionConstraint(registry, c, GlobalFrictionCoeff);
        }
    }

    // Rolling friction pass after velocity
    for (auto &c : contacts) {
        if (registry.valid(c.a) && !isInfiniteMass(registry, c.a)) {
            applyRollingFriction(registry, c.a);
        }
        if (registry.valid(c.b) && !isInfiniteMass(registry, c.b)) {
            applyRollingFriction(registry, c.b);
        }
    }

    // Position correction pass (reduce overlap)
    const int positionIterations= 3;
    for (int i=0; i<positionIterations; i++) {
        std::cout<<"=== Position Solver Iter "<<i<<" ===\n";
        for (auto &c : contacts) {
            if (!registry.valid(c.a) || !registry.valid(c.b)) {
                continue;
            }
            // apply a simple Baumgarte correction
            solvePosition(registry, c);

        }
    }

    // Sleep checks
    for (auto &c : contacts) {
        checkSleep(registry, c);
    }

    // store final impulses for next frame
    manager.applySolverResults(contacts);

    std::cout<<"[solveContactConstraints] Done. Impulses stored.\n";
}

} // namespace RigidBodyCollision