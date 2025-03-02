/**
 * @file contact_solver.hpp
 * @brief Velocity-based constraint solver for rigid body collisions
 *
 * This module implements an iterative impulse-based solver that resolves
 * collision constraints through velocity corrections. It handles both normal
 * and friction impulses, with support for warm-starting from the contact manager.
 */

#pragma once

#include <entt/entt.hpp>
#include "nbody/systems/rigid/contact_manager.hpp"

namespace RigidBodyCollision {

/**
 * @struct ContactSolverConfig
 * @brief Configuration parameters specific to the contact solver
 */
struct ContactSolverConfig {
    // Number of velocity solver iterations
    int iterations = 10;
    
    // Friction coefficient
    float frictionCoeff = 0.5f;
};

/**
 * @brief Resolves collision constraints using velocity-based impulses
 *
 * Implements a velocity-only solver that:
 * - Applies normal impulses to prevent penetration
 * - Handles friction using a simplified Coulomb model
 * - Supports angular velocity for rotating bodies
 * - Uses multiple iterations for stability
 */
class ContactSolver {
public:
    /**
     * @brief Resolves all active collision constraints
     *
     * @param registry   ECS registry containing physics components
     * @param manager    Contact manager providing collision data and warm-start info
     * @param config     Contact solver configuration parameters
     */
    static void solveContactConstraints(
        entt::registry &registry,
        ContactManager &manager,
        const ContactSolverConfig &config = ContactSolverConfig()
    );
};

} // namespace RigidBodyCollision
