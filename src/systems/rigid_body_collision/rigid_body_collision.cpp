/**
 * @file rigid_body_collision.cpp
 * @brief Implementation of the main collision system pipeline
 */

#include <vector>

#include "nbody/systems/rigid_body_collision/rigid_body_collision.hpp"
#include "nbody/systems/rigid_body_collision/broadphase.hpp"
#include "nbody/systems/rigid_body_collision/narrowphase.hpp"
#include "nbody/systems/rigid_body_collision/contact_manager.hpp"
#include "nbody/systems/rigid_body_collision/contact_solver.hpp"
#include "nbody/systems/rigid_body_collision/position_solver.hpp"
#include "nbody/systems/rigid_body_collision/collision_data.hpp"

namespace Systems
{

void RigidBodyCollisionSystem::update(entt::registry &registry,
                                      int solverIterations,
                                      int positionalSolverIterations,
                                      double baumgarte,
                                      double slop)
{
    using namespace RigidBodyCollision;

    // Create contact manager for this frame
    ContactManager manager;

    // Multiple velocity solver passes for stability
    for (int pass = 0; pass < solverIterations; pass++) {
        // 1) Broad-phase: Quick AABB-based filtering
        auto candidatePairs = broadPhase(registry);

        // 2) Narrow-phase: Exact collision detection with GJK/EPA
        auto manifold = narrowPhase(registry, candidatePairs);

        if (manifold.collisions.empty()) {
            break;  // Early exit if no collisions found
        }

        // 3) Update contact manager and solve velocity constraints
        manager.updateContacts(manifold);
        ContactSolver::solveContactConstraints(registry, manager, baumgarte, slop);
    }

    // 4) Additional position correction passes if requested
    if (positionalSolverIterations > 0) {
        // Final broad/narrow phase to catch any remaining overlaps
        auto candidatePairs = broadPhase(registry);
        auto manifold = narrowPhase(registry, candidatePairs);
        
        if (!manifold.collisions.empty()) {
            PositionSolver::positionalSolver(registry, manifold,
                                          positionalSolverIterations,
                                          baumgarte, slop);
        }
    }
}

} // namespace Systems