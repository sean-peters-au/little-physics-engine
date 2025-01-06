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
                                            const ContactManager &manager,
                                            double /*baumgarte*/,
                                            double /*slop*/)
{
    PROFILE_SCOPE("ContactSolver");
    const double angularDamping = 0.98;

    const auto &manifold = manager.getCurrentManifold();
    auto &collisions = manifold.collisions;
    if (collisions.empty()) {
        return;
    }

    const int solverIterations = 10;
    for (int iter = 0; iter < solverIterations; iter++) {
        for (auto &c : collisions) {
            if (!registry.valid(c.a) || !registry.valid(c.b)) {
                continue;
            }

            // Get physics components
            auto &massA = registry.get<Components::Mass>(c.a);
            auto &massB = registry.get<Components::Mass>(c.b);
            auto velA = registry.get<Components::Velocity>(c.a);
            auto velB = registry.get<Components::Velocity>(c.b);

            bool hasRotA = registry.all_of<Components::AngularVelocity, Components::Inertia>(c.a);
            bool hasRotB = registry.all_of<Components::AngularVelocity, Components::Inertia>(c.b);

            // Handle infinite mass bodies (invMass = 0)
            double invMassA = (massA.value > 1e29) ? 0.0 : (1.0 / massA.value);
            double invMassB = (massB.value > 1e29) ? 0.0 : (1.0 / massB.value);
            double invSum = invMassA + invMassB;
            if (invSum < 1e-12) {
                continue;
            }

            // ----------------------------
            // 1) Normal Impulse (unchanged)
            // ----------------------------
            Vector n = c.normal;
            Vector relVel = velB - velA;
            double vn = relVel.dotProduct(n);

            // No restitution in this code
            double jn = -vn / invSum;
            // Prevent pulling if shapes are separating
            jn = std::max(0.0, jn);

            // Apply normal impulse to linear velocities
            Vector Pn = n * jn;
            velA = velA - (Pn * invMassA);
            velB = velB + (Pn * invMassB);

            // Store updated linear velocities now, so angular code can see them
            registry.replace<Components::Velocity>(c.a, velA);
            registry.replace<Components::Velocity>(c.b, velB);

            // Angular response to normal impulse
            if (registry.all_of<Components::AngularVelocity, Components::Inertia>(c.a))
            {
                auto &angA = registry.get<Components::AngularVelocity>(c.a);
                auto &I_A  = registry.get<Components::Inertia>(c.a);
                auto posA = registry.get<Components::Position>(c.a);
                Vector rA = Vector(c.contactPoint.x - posA.x, c.contactPoint.y - posA.y);
                double crossA = rA.cross(Pn) / I_A.I;
                angA.omega -= crossA;
                angA.omega *= angularDamping;
                registry.replace<Components::AngularVelocity>(c.a, angA);
            }

            if (registry.all_of<Components::AngularVelocity, Components::Inertia>(c.b))
            {
                auto &angB = registry.get<Components::AngularVelocity>(c.b);
                auto &I_B  = registry.get<Components::Inertia>(c.b);
                auto posB = registry.get<Components::Position>(c.b);
                Vector rB = Vector(c.contactPoint.x - posB.x, c.contactPoint.y - posB.y);
                double crossB = rB.cross(Pn) / I_B.I;
                angB.omega += crossB;
                angB.omega *= angularDamping;
                registry.replace<Components::AngularVelocity>(c.b, angB);
            }

            // ----------------------------
            // 2) Friction Impulse (modified to use contact velocities)
            // ----------------------------
            // Re-fetch (in case the normal impulse changed them):
            velA = registry.get<Components::Velocity>(c.a);
            velB = registry.get<Components::Velocity>(c.b);

            // We'll compute the contact-point velocities for friction only:
            //   vA_contact = velA + (omegaA x rA)
            //   vB_contact = velB + (omegaB x rB)
            Vector vA_contact = velA;
            Vector vB_contact = velB;

            Vector rA(0.0, 0.0), rB(0.0, 0.0);
            double wA = 0.0, wB = 0.0;

            if (hasRotA) {
                auto &angA = registry.get<Components::AngularVelocity>(c.a);
                auto posA = registry.get<Components::Position>(c.a);
                rA = Vector(c.contactPoint.x - posA.x, c.contactPoint.y - posA.y);
                wA = angA.omega;
                // 2D cross(omegaA, rA) => Vector(-omega*ry, omega*rx)
                vA_contact = vA_contact + Vector(-wA * rA.y, wA * rA.x);
            }
            if (hasRotB) {
                auto &angB = registry.get<Components::AngularVelocity>(c.b);
                auto posB = registry.get<Components::Position>(c.b);
                rB = Vector(c.contactPoint.x - posB.x, c.contactPoint.y - posB.y);
                wB = angB.omega;
                vB_contact = vB_contact + Vector(-wB * rB.y, wB * rB.x);
            }

            // Tangential (friction) relative velocity at the contact
            Vector relVelContact = vB_contact - vA_contact;
            Vector vt = relVelContact - n * (relVelContact.dotProduct(n));
            double vtLen = vt.length();
            if (vtLen > 1e-9) {
                // Tangential direction
                Vector tDir = vt / vtLen;

                double jt = -(relVelContact.dotProduct(tDir)) / invSum;
                double mu = 0.3; // friction coefficient
                double frictionLimit = mu * jn;

                // Clamp friction to Coulomb limit
                double jtClamped = std::max(-frictionLimit, std::min(frictionLimit, jt));
                Vector Pf = tDir * jtClamped;

                // Apply friction impulses to linear velocities
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
        } // end for each collision
    } // end iterations
}

} // namespace RigidBodyCollision