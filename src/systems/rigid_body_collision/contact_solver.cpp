/**
 * @file contact_solver.cpp
 * @brief Implementation of velocity-based collision constraint solver
 *
 * Implements an iterative impulse solver that resolves collisions through
 * velocity corrections, handling both normal response and friction effects.
 */

#include "nbody/systems/rigid_body_collision/contact_solver.hpp"
#include <iostream>
#include <cmath>
#include <entt/entt.hpp>
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/core/profile.hpp"

namespace RigidBodyCollision {

void ContactSolver::solveContactConstraints(entt::registry &registry,
                                            ContactManager &manager,
                                            double /*baumgarte*/,
                                            double /*slop*/)
{
    PROFILE_SCOPE("ContactSolver");
    const double angularDamping = 0.98;
    const double frictionCoeff  = 0.3;  // global friction for demonstration

    // Retrieve contacts (with warm-start impulses) from the manager
    auto &contacts = manager.getContactsForSolver();

    //--------------------------------------------------------------------------
    // Warm-Start Phase
    //--------------------------------------------------------------------------
    // Re-apply the previously accumulated impulses so we start each iteration
    // from a better initial condition. Also clamp friction to avoid blow-ups.
    for (auto &c : contacts) {
        if (!registry.valid(c.a) || !registry.valid(c.b)) {
            continue;
        }

        // Get physics components
        auto &massA = registry.get<Components::Mass>(c.a);
        auto &massB = registry.get<Components::Mass>(c.b);

        bool hasRotA = registry.all_of<Components::AngularVelocity, Components::Inertia>(c.a);
        bool hasRotB = registry.all_of<Components::AngularVelocity, Components::Inertia>(c.b);

        // Handle infinite mass
        double invMassA = (massA.value > 1e29) ? 0.0 : (1.0 / massA.value);
        double invMassB = (massB.value > 1e29) ? 0.0 : (1.0 / massB.value);

        auto velA = registry.get<Components::Velocity>(c.a);
        auto velB = registry.get<Components::Velocity>(c.b);

        // Normal impulse
        Vector Pn = c.normal * c.normalImpulseAccum;
        velA = velA - (Pn * invMassA);
        velB = velB + (Pn * invMassB);

        //---------------------------------------------------------------------------------------
        // Friction impulse must be clamped by the friction limit: mu * normalImpulseAccum
        // If c.tangentImpulseAccum is bigger than mu * c.normalImpulseAccum, clamp it.
        //---------------------------------------------------------------------------------------
        double maxFriction = frictionCoeff * std::fabs(c.normalImpulseAccum);
        double tangentImpulse = c.tangentImpulseAccum;
        if (std::fabs(tangentImpulse) > maxFriction) {
            tangentImpulse = (tangentImpulse > 0.0) ? maxFriction : -maxFriction;
            c.tangentImpulseAccum = tangentImpulse; // store back
        }

        // We'll apply friction in the direction perpendicular to c.normal used last iteration
        // A simple 2D perpendicular: t = { -ny, nx }
        // This can cause some error if the normal changed slightly,
        // but we already partially mitigate large changes by resetting impulses in contact_manager.
        Vector t(-c.normal.y, c.normal.x);
        Vector Pf = t * tangentImpulse;

        velA = velA - (Pf * invMassA);
        velB = velB + (Pf * invMassB);

        // Write back updated linear velocities (after warm start)
        registry.replace<Components::Velocity>(c.a, velA);
        registry.replace<Components::Velocity>(c.b, velB);

        // Warm start for angular velocity
        if (hasRotA) {
            auto &angA = registry.get<Components::AngularVelocity>(c.a);
            auto &I_A  = registry.get<Components::Inertia>(c.a);
            auto posA  = registry.get<Components::Position>(c.a);
            Vector rA = Vector(c.contactPoint.x - posA.x, c.contactPoint.y - posA.y);

            double crossN_A = rA.cross(Pn) / I_A.I;
            double crossT_A = rA.cross(Pf) / I_A.I;
            angA.omega -= (crossN_A + crossT_A);
            angA.omega *= angularDamping;
            registry.replace<Components::AngularVelocity>(c.a, angA);
        }

        if (hasRotB) {
            auto &angB = registry.get<Components::AngularVelocity>(c.b);
            auto &I_B  = registry.get<Components::Inertia>(c.b);
            auto posB  = registry.get<Components::Position>(c.b);
            Vector rB = Vector(c.contactPoint.x - posB.x, c.contactPoint.y - posB.y);

            double crossN_B = rB.cross(Pn) / I_B.I;
            double crossT_B = rB.cross(Pf) / I_B.I;
            angB.omega += (crossN_B + crossT_B);
            angB.omega *= angularDamping;
            registry.replace<Components::AngularVelocity>(c.b, angB);
        }
    }

    //--------------------------------------------------------------------------
    // Main Solver Iterations (same structure as before)
    //--------------------------------------------------------------------------
    const int solverIterations = 10;
    for (int iter = 0; iter < solverIterations; iter++) {
        for (auto &c : contacts) {
            if (!registry.valid(c.a) || !registry.valid(c.b)) {
                continue;
            }

            // Get physics components
            auto &massA = registry.get<Components::Mass>(c.a);
            auto &massB = registry.get<Components::Mass>(c.b);
            auto velA   = registry.get<Components::Velocity>(c.a);
            auto velB   = registry.get<Components::Velocity>(c.b);

            bool hasRotA = registry.all_of<Components::AngularVelocity, Components::Inertia>(c.a);
            bool hasRotB = registry.all_of<Components::AngularVelocity, Components::Inertia>(c.b);

            double invMassA = (massA.value > 1e29) ? 0.0 : (1.0 / massA.value);
            double invMassB = (massB.value > 1e29) ? 0.0 : (1.0 / massB.value);
            double invSum   = invMassA + invMassB;
            if (invSum < 1e-12) {
                continue;
            }

            // ----------------------------
            // 1) Normal Impulse
            // ----------------------------
            Vector n = c.normal;
            Vector relVel = velB - velA;
            double vn = relVel.dotProduct(n);

            // No restitution in this code
            double jn = -vn / invSum;
            // Prevent pulling if shapes are separating
            jn = std::max(0.0, jn);

            // Accumulate normal impulse
            double oldNormalImpulse = c.normalImpulseAccum;
            double newNormalImpulse = oldNormalImpulse + jn;
            c.normalImpulseAccum    = newNormalImpulse;

            // The actual impulse to apply is the increment
            double dJn = newNormalImpulse - oldNormalImpulse;
            Vector Pn = n * dJn;

            // Apply normal impulse to linear velocities
            velA = velA - (Pn * invMassA);
            velB = velB + (Pn * invMassB);

            // Store updated linear velocities so angular code can see them
            registry.replace<Components::Velocity>(c.a, velA);
            registry.replace<Components::Velocity>(c.b, velB);

            // Angular response to normal impulse
            if (hasRotA) {
                auto &angA = registry.get<Components::AngularVelocity>(c.a);
                auto &I_A  = registry.get<Components::Inertia>(c.a);
                auto posA  = registry.get<Components::Position>(c.a);
                Vector rA  = Vector(c.contactPoint.x - posA.x, c.contactPoint.y - posA.y);
                double crossA = rA.cross(Pn) / I_A.I;
                angA.omega -= crossA;
                angA.omega *= angularDamping;
                registry.replace<Components::AngularVelocity>(c.a, angA);
            }
            if (hasRotB) {
                auto &angB = registry.get<Components::AngularVelocity>(c.b);
                auto &I_B  = registry.get<Components::Inertia>(c.b);
                auto posB  = registry.get<Components::Position>(c.b);
                Vector rB  = Vector(c.contactPoint.x - posB.x, c.contactPoint.y - posB.y);
                double crossB = rB.cross(Pn) / I_B.I;
                angB.omega += crossB;
                angB.omega *= angularDamping;
                registry.replace<Components::AngularVelocity>(c.b, angB);
            }

            // ----------------------------
            // 2) Friction Impulse
            // ----------------------------
            velA = registry.get<Components::Velocity>(c.a);
            velB = registry.get<Components::Velocity>(c.b);

            Vector vA_contact = velA;
            Vector vB_contact = velB;
            Vector rA(0.0, 0.0), rB(0.0, 0.0);
            double wA = 0.0, wB = 0.0;

            if (hasRotA) {
                auto &angA = registry.get<Components::AngularVelocity>(c.a);
                auto posA  = registry.get<Components::Position>(c.a);
                rA = Vector(c.contactPoint.x - posA.x, c.contactPoint.y - posA.y);
                wA = angA.omega;
                vA_contact = vA_contact + Vector(-wA * rA.y, wA * rA.x);
            }
            if (hasRotB) {
                auto &angB = registry.get<Components::AngularVelocity>(c.b);
                auto posB  = registry.get<Components::Position>(c.b);
                rB = Vector(c.contactPoint.x - posB.x, c.contactPoint.y - posB.y);
                wB = angB.omega;
                vB_contact = vB_contact + Vector(-wB * rB.y, wB * rB.x);
            }

            // Tangential (friction) relative velocity at the contact
            Vector relVelContact = vB_contact - vA_contact;
            Vector vt = relVelContact - n * (relVelContact.dotProduct(n));
            double vtLen = vt.length();
            if (vtLen > 1e-9) {
                Vector tDir = vt / vtLen;

                double jt = -(relVelContact.dotProduct(tDir)) / invSum;
                double frictionLimit = frictionCoeff * std::fabs(c.normalImpulseAccum);

                // Add to the old friction impulse
                double oldTangentImpulse = c.tangentImpulseAccum;
                double targetTangentImpulse = oldTangentImpulse + jt;
                // Clamp friction to the Coulomb limit
                double newTangentImpulse = std::max(-frictionLimit,
                                                    std::min(frictionLimit, targetTangentImpulse));
                c.tangentImpulseAccum = newTangentImpulse;

                // The actual impulse to apply in this iteration
                double dJt = newTangentImpulse - oldTangentImpulse;
                Vector Pf = tDir * dJt;

                // Apply friction impulses
                velA = velA - (Pf * invMassA);
                velB = velB + (Pf * invMassB);

                // Angular friction
                if (hasRotA) {
                    auto &angA = registry.get<Components::AngularVelocity>(c.a);
                    auto &I_A  = registry.get<Components::Inertia>(c.a);
                    double crossA = rA.cross(Pf) / I_A.I;
                    angA.omega -= crossA;
                    angA.omega *= angularDamping;
                    registry.replace<Components::AngularVelocity>(c.a, angA);
                }
                if (hasRotB) {
                    auto &angB = registry.get<Components::AngularVelocity>(c.b);
                    auto &I_B  = registry.get<Components::Inertia>(c.b);
                    double crossB = rB.cross(Pf) / I_B.I;
                    angB.omega += crossB;
                    angB.omega *= angularDamping;
                    registry.replace<Components::AngularVelocity>(c.b, angB);
                }
            }

            // Store updated linear velocities
            registry.replace<Components::Velocity>(c.a, velA);
            registry.replace<Components::Velocity>(c.b, velB);
        }
    }

    //--------------------------------------------------------------------------
    // Write back final impulses so they can be used in future frames
    //--------------------------------------------------------------------------
    manager.applySolverResults(contacts);
}

} // namespace RigidBodyCollision