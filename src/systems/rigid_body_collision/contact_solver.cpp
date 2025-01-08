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

#define ENABLE_CONTACT_SOLVER_DEBUG 0
#define DEBUG(x) do { if (ENABLE_CONTACT_SOLVER_DEBUG) { std::cout << x; } } while(0)

namespace RigidBodyCollision {

// --------------------- Tuning -------------------------
static const double Gravity               = 9.81;
static const double GlobalFrictionCoeff   = 0.5;
static const double RollingFrictionCoeff  = 0.00001;
static const double RollingCutoff         = 0.00001;
static const double AngularDamping        = 0.995;
//-------------------------------------------------------

static bool isInfiniteMass(const entt::registry &registry, entt::entity e)
{
    if (!registry.valid(e)) return false;
    if (!registry.all_of<Components::Mass>(e)) return false;
    auto &m = registry.get<Components::Mass>(e);
    return (m.value > 1e29);
}

static bool canRotate(entt::registry &registry, entt::entity e)
{
    if (!registry.all_of<Components::AngularVelocity, Components::Inertia>(e)) {
        return false;
    }
    auto &I = registry.get<Components::Inertia>(e);
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
    if (!canRotate(registry, entity)) return;

    auto &angVel  = registry.get<Components::AngularVelocity>(entity);
    auto &inertia = registry.get<Components::Inertia>(entity);

    double crossVal = r.cross(impulse) / inertia.I;
    double oldOmega = angVel.omega;

    angVel.omega += sign * crossVal * factor;
    angVel.omega *= angularDamping;

    DEBUG(std::string("[applyAngularImpulse] e= ") + std::to_string((int)entity) + "\n");
    DEBUG(std::string(" cross= ") + std::to_string(crossVal) + "\n");
    DEBUG(std::string(" oldOmega= ") + std::to_string(oldOmega) + "\n");
    DEBUG(std::string(" newOmega= ") + std::to_string(angVel.omega) + "\n");
    DEBUG("\n");

    registry.replace<Components::AngularVelocity>(entity, angVel);
}

static void applyRollingFriction(entt::registry &registry,
                                 entt::entity e)
{
    if (!canRotate(registry, e)) return;
    auto &angVel = registry.get<Components::AngularVelocity>(e);
    auto &inertia= registry.get<Components::Inertia>(e);
    auto &mass  = registry.get<Components::Mass>(e);
    if (mass.value > 1e29) return; 

    double w = angVel.omega;
    if (std::fabs(w) < RollingCutoff) {
        return;
    }
    double sign   = (w > 0.0) ? -1.0 : +1.0;
    double torque = RollingFrictionCoeff * mass.value * Gravity * 0.5;
    double alpha  = torque / inertia.I;

    if (std::fabs(alpha) > std::fabs(w)) {
        alpha = w * (-1.0);
    }
    double oldW = w;
    angVel.omega += sign * alpha;
    angVel.omega *= AngularDamping;

    DEBUG(std::string("[rollingFriction] e=") + std::to_string((int)e) + "\n");
    DEBUG(std::string(" oldW=") + std::to_string(oldW) + "\n");
    DEBUG(std::string(" alpha=") + std::to_string(alpha) + "\n");
    DEBUG(std::string(" newW=") + std::to_string(angVel.omega) + "\n");
    DEBUG("\n");

    registry.replace<Components::AngularVelocity>(e, angVel);
}

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

    DEBUG(std::string("[warmStartContact] eA=") + std::to_string((int)c.a) + "\n");
    DEBUG(std::string(" eB=") + std::to_string((int)c.b) + "\n");
    DEBUG(std::string(" normalImpulse=") + std::to_string(c.normalImpulseAccum) + "\n");
    DEBUG(std::string(" tangentImpulse=") + std::to_string(c.tangentImpulseAccum) + "\n");
    DEBUG("\n");
}

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
        return;
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

    DEBUG(std::string("[solveNormal] eA=") + std::to_string((int)c.a) + "\n");
    DEBUG(std::string(" eB=") + std::to_string((int)c.b) + "\n");
    DEBUG(std::string(" oldN=") + std::to_string(oldImpulse) + "\n");
    DEBUG(std::string(" newN=") + std::to_string(newImpulse) + "\n");
    DEBUG(std::string(" dJn=") + std::to_string(dJn) + "\n");
    DEBUG(std::string(" vn=") + std::to_string(vn) + "\n");
    DEBUG("\n");
}

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

    Vector vt = rel - (n * dotN);
    double vtLen = vt.length();
    if (vtLen < 1e-9) {
        return;
    }

    Vector tDir = vt / vtLen;
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

    DEBUG(std::string("[solveFriction] eA=") + std::to_string((int)c.a) + "\n");
    DEBUG(std::string(" eB=") + std::to_string((int)c.b) + "\n");
    DEBUG(std::string(" oldTangent=") + std::to_string(oldTangent) + "\n");
    DEBUG(std::string(" newTangent=") + std::to_string(newTangent) + "\n");
    DEBUG(std::string(" vtLen=") + std::to_string(vtLen) + "\n");
    DEBUG(std::string(" frictionLimit=") + std::to_string(limit) + "\n");
    DEBUG("\n");
}

static void checkSleep(entt::registry &registry, ContactRef &c)
{
    auto trySleep = [&](entt::entity e)
    {
        if (!registry.valid(e)) return;
        if (!registry.all_of<Components::Sleep, Components::Velocity, Components::Mass>(e)) return;
        auto &slp = registry.get<Components::Sleep>(e);
        auto &m   = registry.get<Components::Mass>(e);
        if (m.value>1e29) return;

        auto vel = registry.get<Components::Velocity>(e);
        double speed = vel.length();
        double w=0.0;
        if (canRotate(registry, e)) {
            w = std::fabs(registry.get<Components::AngularVelocity>(e).omega);
        }
    };
    trySleep(c.a);
    trySleep(c.b);
}

//---------------------------------------------------------------
// The main public function
//---------------------------------------------------------------
void ContactSolver::solveContactConstraints(
    entt::registry &registry,
    ContactManager &manager
)
{
    // Destroy old contact-visual entities
    auto oldContacts = registry.view<RigidBodyCollision::ContactRef>();
    registry.destroy(oldContacts.begin(), oldContacts.end());

    // Now retrieve the manifold data
    auto &manifolds = manager.getManifoldsForSolver();

    // Create debug visualization entities for each contact in each manifold
    for (auto &m : manifolds) {
        for (auto &c : m.contacts) {
            auto contactEntity = registry.create();
            registry.emplace<RigidBodyCollision::ContactRef>(contactEntity, c);
        }
    }

    // Warm Start
    for (auto &m : manifolds) {
        for (auto &c : m.contacts) {
            warmStartContact(registry, c, GlobalFrictionCoeff);
        }
    }

    // multiple velocity solver iterations :)
    const int velocityIterations = 10;
    for (int iter=0; iter<velocityIterations; iter++) {
        DEBUG(std::string("=== Velocity Solver Iter ") + std::to_string(iter) + " ===\n");
        for (auto &m : manifolds) {
            for (auto &c : m.contacts) {
                if (!registry.valid(c.a) || !registry.valid(c.b)) {
                    continue;
                }
                solveNormalConstraint(registry, c, GlobalFrictionCoeff);
                solveFrictionConstraint(registry, c, GlobalFrictionCoeff);
            }
        }
    }

    // Rolling friction pass
    for (auto &m : manifolds) {
        for (auto &c : m.contacts) {
            if (registry.valid(c.a) && !isInfiniteMass(registry, c.a)) {
                applyRollingFriction(registry, c.a);
            }
            if (registry.valid(c.b) && !isInfiniteMass(registry, c.b)) {
                applyRollingFriction(registry, c.b);
            }
        }
    }

    // Sleep checks (purely velocity-based)
    for (auto &m : manifolds) {
        for (auto &c : m.contacts) {
            checkSleep(registry, c);
        }
    }

    // store final impulses for next frame
    manager.applySolverResults(manifolds);

    DEBUG("Done. Impulses stored.\n");
}

} // namespace RigidBodyCollision