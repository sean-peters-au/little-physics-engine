/**
 * @file contact_solver.hpp
 * @brief Velocity-based constraint solver for rigid body collisions
 *
 * This module implements an iterative impulse-based solver that resolves
 * collision constraints through velocity corrections. It handles both normal
 * and friction impulses, with support for warm-starting from the contact manager.
 */

#ifndef CONTACT_SOLVER_HPP
#define CONTACT_SOLVER_HPP

#include <entt/entt.hpp>
#include "nbody/systems/rigid_body_collision/contact_manager.hpp"

namespace RigidBodyCollision {

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
     */
    static void solveContactConstraints(
        entt::registry &registry,
        ContactManager &manager
    );
};

} // namespace RigidBodyCollision

#endif