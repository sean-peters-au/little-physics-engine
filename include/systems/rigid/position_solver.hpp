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
#include "systems/rigid/collision_data.hpp"

namespace RigidBodyCollision {

/**
 * @brief Position solver configuration parameters
 */
struct PositionSolverConfig {
    // Number of position correction iterations to perform.
    // More iterations yield a more accurate resolution of penetrations, but may impact performance.
    int iterations = 3;
    
    // Baumgarte stabilization factor (correction strength).
    // This factor (between 0 and 1) controls the incremental positional correction applied each iteration.
    // Higher values result in more aggressive corrections, which can improve convergence but may lead to instability if set too high.
    double baumgarte = 0.2;
    
    // Penetration tolerance (slop).
    // Corrections are applied only if the penetration depth exceeds this threshold to avoid excessive jittering from minor overlaps.
    double slop = 0.001;
};

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
     * @param config     Position solver configuration parameters
     * 
     * @note Only solid phase particles participate in position correction
     *       to allow fluid-like behavior for non-solid phases
     */
    static void positionalSolver(
        entt::registry &registry,
        const CollisionManifold &manifold,
        const PositionSolverConfig &config = PositionSolverConfig()
    );
};

} // namespace RigidBodyCollision
