/**
 * @file position_solver.hpp
 * @brief Position-based penetration correction for rigid body collisions
 *
 * This module implements a position-based correction pass that runs after
 * the velocity solver to eliminate any remaining penetration between bodies.
 * It uses a Baumgarte stabilization approach to smoothly separate overlapping
 * objects while respecting their mass ratios.
 */

#pragma once

#include <entt/entt.hpp>
#include "nbody/systems/rigid/collision_data.hpp"

namespace RigidBodyCollision {

/**
 * @brief Resolves remaining penetrations through position adjustments
 *
 * Implements a "split impulse" style position solver that:
 * - Runs after the velocity solver
 * - Directly adjusts positions to remove overlap
 * - Uses mass-weighting for physically correct separation
 * - Applies smoothing via Baumgarte factor
 * - Supports a penetration tolerance (slop)
 */
class PositionSolver {
public:
    /**
     * @brief Resolves penetrations using position-based correction
     * 
     * @param registry   ECS registry containing position and mass components
     * @param manifold   Collision manifold from narrow-phase
     * @param iterations Number of correction passes to apply
     * @param baumgarte  Correction strength factor (0-1)
     * @param slop       Penetration tolerance before correction
     * 
     * @note Only solid phase particles participate in position correction
     *       to allow fluid-like behavior for non-solid phases
     */
    static void positionalSolver(
        entt::registry &registry,
        const CollisionManifold &manifold
    );
};

} // namespace RigidBodyCollision
