/**
 * @file rigid_body_collision.hpp
 * @brief Main collision detection and resolution system for rigid bodies
 *
 * This module orchestrates the complete collision pipeline:
 * 1. Broad-phase: Quick AABB-based filtering of potential collisions
 * 2. Narrow-phase: Exact collision detection using GJK/EPA
 * 3. Contact solving: Iterative velocity-based constraint resolution
 * 4. Position correction: Baumgarte stabilization for penetration
 */

#ifndef RIGID_BODY_COLLISION_SYSTEM_HPP
#define RIGID_BODY_COLLISION_SYSTEM_HPP

#include <entt/entt.hpp>

namespace Systems {

/**
 * @brief Main collision system coordinating detection and resolution
 *
 * Implements a complete collision handling pipeline that:
 * - Uses spatial partitioning for broad-phase filtering
 * - Performs exact collision tests with GJK/EPA
 * - Resolves collisions with an iterative impulse solver
 * - Corrects penetrations with position-based stabilization
 * - Supports both dynamic and static bodies
 * - Handles different particle phases (solid, fluid)
 */
class RigidBodyCollisionSystem {
public:
    /**
     * @brief Executes the complete collision pipeline
     * 
     * @param registry ECS registry containing physics components
     * @param solverIterations Number of velocity solver iterations (default: 5)
     * @param positionalSolverIterations Number of position correction passes (default: 2)
     * @param baumgarte Position correction strength factor (default: 0.5)
     * @param slop Penetration tolerance before correction (default: 0.001)
     * 
     * @note The system automatically breaks early if no collisions are detected
     *       during broad-phase to avoid unnecessary computation
     */
    static void update(entt::registry &registry,
                      int solverIterations = 5,
                      int positionalSolverIterations = 2,
                      double baumgarte = 0.5,
                      double slop = 0.001);
};

} // namespace Systems

#endif // RIGID_BODY_COLLISION_SYSTEM_HPP