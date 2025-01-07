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
#include "nbody/math/polygon.hpp"
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
    if (!canRotate(registry, entity)) return;
    
    // Don't even process if impulse is zero
    if (impulse.length() < 1e-10) {
        std::cout << "Skipping zero impulse" << std::endl;
        return;
    }
    
    auto &angVel  = registry.get<Components::AngularVelocity>(entity);
    auto &inertia = registry.get<Components::Inertia>(entity);

    std::cout << "r: " << r.x << ", " << r.y << std::endl;
    std::cout << "impulse: " << impulse.x << ", " << impulse.y << std::endl;
    double crossTerm = r.cross(impulse) / inertia.I;
    std::cout << "crossTerm: " << crossTerm << std::endl;
    std::cout << "sign: " << sign << std::endl;
    std::cout << "factor: " << factor << std::endl;
    std::cout << "angularDamping: " << angularDamping << std::endl;
    
    if (std::fabs(crossTerm) > 1e-10) {
        std::cout << "before omega: " << angVel.omega << std::endl;
        angVel.omega += sign * crossTerm * factor;
        angVel.omega *= angularDamping;
        std::cout << "after omega: " << angVel.omega << std::endl;
        registry.replace<Components::AngularVelocity>(entity, angVel);
    }
}

static bool isGroundContact(const entt::registry &registry, const ContactRef &c) {
    Vector up(0.0, 1.0);
    double dotUp = c.normal.dotProduct(up);
    return (isInfiniteMass(registry, c.a) ^ isInfiniteMass(registry, c.b)) && 
           (dotUp < -0.7);
}

static entt::entity getMovingBody(const entt::registry &registry, const ContactRef &c) {
    return isInfiniteMass(registry, c.a) ? c.b : c.a;
}

static Vector getContactToCOM(const entt::registry &registry, 
                            const ContactRef &c,
                            entt::entity movingBody) {
    auto pos = registry.get<Components::Position>(movingBody);
    return Vector(c.contactPoint.x - pos.x, c.contactPoint.y - pos.y);
}

static double calculateStabilityMetric(const Vector &r, const Vector &normal) {
    Vector comProjection = r - normal * r.dotProduct(normal);
    return comProjection.dotProduct(normal.rotate());
}

static double calculateTippingTorque(const entt::registry &registry,
                                   entt::entity movingBody,
                                   const Vector &contactPoint,
                                   const Vector &r,
                                   const Vector &normal) {
    double massVal = registry.get<Components::Mass>(movingBody).value;
    double angle = registry.get<Components::AngularPosition>(movingBody).angle;
    
    // Vector pointing down (gravity direction)
    Vector gravityDir(0.0, -1.0);
    Vector gravityForce = gravityDir * (massVal * Gravity);
    
    // Project COM onto ground plane
    Vector comProjection = r - normal * r.dotProduct(normal);
    double stabilityOnEdge = comProjection.dotProduct(normal.rotate());
    
    // If COM is nearly over the contact point, no torque
    if (std::fabs(stabilityOnEdge) < 0.01) {
        std::cout << "  COM nearly over contact - no torque" << std::endl;
        return 0.0;
    }
    
    // Calculate torque ONLY from horizontal offset
    // This is key: vertical offset shouldn't contribute to tipping!
    double horizontalLeverArm = stabilityOnEdge;
    double torque = massVal * Gravity * horizontalLeverArm;
    
    std::cout << "Tipping calculation for entity " << (uint32_t)movingBody << ":" << std::endl;
    std::cout << "  contact point: " << contactPoint.x << ", " << contactPoint.y << std::endl;
    std::cout << "  COM offset r: " << r.x << ", " << r.y << std::endl;
    std::cout << "  horizontal lever arm: " << horizontalLeverArm << std::endl;
    std::cout << "  mass: " << massVal << std::endl;
    std::cout << "  current angle: " << angle << " rad" << std::endl;
    std::cout << "  base torque: " << torque << std::endl;
    
    return torque;
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
                               double invSum)
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

    // Only apply impulses if they're significant
    if (std::fabs(dJn) > 1e-10) {
        velA -= Pn * invMassA;
        velB += Pn * invMassB;

        registry.replace<Components::Velocity>(c.a, velA);
        registry.replace<Components::Velocity>(c.b, velB);
    }
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

    // Build contact point velocities
    Vector velA_contact = velA;
    Vector velB_contact = velB;
    if (canRotate(registry, c.a)) {
        double wA = registry.get<Components::AngularVelocity>(c.a).omega;
        velA_contact += Vector(-wA * rA.y, wA * rA.x);
    }
    if (canRotate(registry, c.b)) {
        double wB = registry.get<Components::AngularVelocity>(c.b).omega;
        velB_contact += Vector(-wB * rB.y, wB * rB.x);
    }

    Vector n = c.normal;
    Vector relVelContact = velB_contact - velA_contact;
    Vector vt = relVelContact - n * relVelContact.dotProduct(n);
    double vtLen = vt.length();

    // Calculate effective mass for friction
    double angularMass = 0.0;
    if (canRotate(registry, c.a)) {
        auto &IA = registry.get<Components::Inertia>(c.a);
        double raCrossT = rA.cross(vt.normalized());
        angularMass += (raCrossT * raCrossT) / IA.I;
    }
    if (canRotate(registry, c.b)) {
        auto &IB = registry.get<Components::Inertia>(c.b);
        double rbCrossT = rB.cross(vt.normalized());
        angularMass += (rbCrossT * rbCrossT) / IB.I;
    }
    double effectiveMass = invSum + angularMass;

    // Calculate friction impulse
    double jt = 0.0;
    if (vtLen > 1e-9) {
        Vector tDir = vt / vtLen;
        jt = -(relVelContact.dotProduct(tDir)) / effectiveMass;
    }

    // Calculate friction limit
    double frictionLimit = frictionCoeff * std::fabs(c.normalImpulseAccum);
    
    // For ground contacts, ensure minimum friction from mg
    if (isGroundContact(registry, c)) {
        auto movingBody = getMovingBody(registry, c);
        double massVal = registry.get<Components::Mass>(movingBody).value;
        double mg = massVal * Gravity;
        frictionLimit = std::max(frictionLimit, frictionCoeff * mg);
    }

    // Apply impulse with limits
    double oldTangentImpulse = c.tangentImpulseAccum;
    double newTangentImpulse = oldTangentImpulse + jt;
    newTangentImpulse = std::max(-frictionLimit, 
                                std::min(frictionLimit, newTangentImpulse));
    
    c.tangentImpulseAccum = newTangentImpulse;
    double dJt = newTangentImpulse - oldTangentImpulse;
    Vector Pf = vt.normalized() * dJt;

    // Only apply angular impulses if we actually have a friction force
    if (std::fabs(dJt) > 1e-10) {
        Vector Pf = vt.normalized() * dJt;
        
        // Apply to linear velocity
        velA -= Pf * invMassA;
        velB += Pf * invMassB;

        registry.replace<Components::Velocity>(c.a, velA);
        registry.replace<Components::Velocity>(c.b, velB);

        // Apply to angular velocity only if we have a real impulse
        std::cout << "\nApplying friction impulse to entity " << (uint32_t)c.a << ":" << std::endl;
        applyAngularImpulse(registry, c.a, rA, Pf, -1.0, 1.0, angularDamping);
        
        std::cout << "\nApplying friction impulse to entity " << (uint32_t)c.b << ":" << std::endl;
        applyAngularImpulse(registry, c.b, rB, Pf, +1.0, 1.0, angularDamping);
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

    // ONCE PER FRAME: Calculate tipping forces
    std::vector<std::pair<entt::entity, double>> tippingForces;
    for (auto &c : contacts) {
        if (!isGroundContact(registry, c)) {
            std::cout << "Not ground contact, skipping..." << std::endl;
            continue;
        }
        
        auto movingBody = getMovingBody(registry, c);
        if (!canRotate(registry, movingBody)) {
            std::cout << "Body cannot rotate, skipping..." << std::endl;
            continue;
        }

        Vector r = getContactToCOM(registry, c, movingBody);
        double stabilityMetric = calculateStabilityMetric(r, c.normal);
        
        std::cout << "\nStability check for entity " << (uint32_t)movingBody << ":" << std::endl;
        std::cout << "  r: " << r.x << ", " << r.y << std::endl;
        std::cout << "  normal: " << c.normal.x << ", " << c.normal.y << std::endl;
        std::cout << "  stabilityMetric: " << stabilityMetric << std::endl;
        
        if (std::fabs(stabilityMetric) > 1e-9) {
            double torque = calculateTippingTorque(registry, movingBody, 
                                                 c.contactPoint,  // Pass actual contact point
                                                 r, c.normal);
            double dt = SimulatorConstants::SecondsPerTick;
            double I = registry.get<Components::Inertia>(movingBody).I;
            double dOmega = (torque / I) * dt;
            
            // Limit maximum angular velocity
            auto &currentAngVel = registry.get<Components::AngularVelocity>(movingBody);
            double maxOmega = 2.0 * M_PI;  // Max 1 rotation per second
            if (std::fabs(currentAngVel.omega + dOmega) > maxOmega) {
                dOmega = (dOmega > 0 ? 1 : -1) * (maxOmega - std::fabs(currentAngVel.omega));
            }
            
            std::cout << "  Angular velocity change:" << std::endl;
            std::cout << "    inertia: " << I << std::endl;
            std::cout << "    dt: " << dt << std::endl;
            std::cout << "    current omega: " << currentAngVel.omega << std::endl;
            std::cout << "    dOmega: " << dOmega << std::endl;
            std::cout << "    new omega will be: " << (currentAngVel.omega + dOmega) << std::endl;
            
            tippingForces.push_back({movingBody, dOmega});
        }
    }

    // Reset accumulated impulses
    for (auto &c : contacts) {
        c.normalImpulseAccum = 0.0;
        c.tangentImpulseAccum = 0.0;
    }

    // Warm start
    std::cout << "\nWarm starting contacts..." << std::endl;
    for (auto &c : contacts) {
        warmStartContact(registry, c, frictionCoeff, angularDamping);
    }

    // Solve iterations
    const int solverIterations = 10;
    for (int iter = 0; iter < solverIterations; iter++) {
        std::cout << "\nSolver iteration " << iter << std::endl;
        for (auto &c : contacts) {
            if (!registry.valid(c.a) || !registry.valid(c.b)) continue;

            auto &massA = registry.get<Components::Mass>(c.a);
            auto &massB = registry.get<Components::Mass>(c.b);
            double invMassA = (massA.value > 1e29) ? 0.0 : (1.0 / massA.value);
            double invMassB = (massB.value > 1e29) ? 0.0 : (1.0 / massB.value);
            double invSum   = invMassA + invMassB;
            if (invSum < 1e-12) continue;

            solveNormalImpulse(registry, c, invMassA, invMassB, invSum);
            solveFrictionImpulse(registry, c, invMassA, invMassB, invSum, frictionCoeff, angularDamping);
        }
    }

    // ONCE PER FRAME: Apply tipping forces
    std::cout << "\nApplying tipping forces..." << std::endl;
    for (const auto& [entity, dOmega] : tippingForces) {
        auto &angVel = registry.get<Components::AngularVelocity>(entity);
        std::cout << "  Entity " << (uint32_t)entity << ":" << std::endl;
        std::cout << "    before omega: " << angVel.omega << std::endl;
        angVel.omega += dOmega;
        std::cout << "    after omega: " << angVel.omega << std::endl;
        registry.replace<Components::AngularVelocity>(entity, angVel);
    }

    manager.applySolverResults(contacts);
}

} // namespace RigidBodyCollision