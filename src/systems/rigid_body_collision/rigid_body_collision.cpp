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

    // 1) Broad-phase: Quick AABB-based filtering (once per frame)
    auto candidatePairs = broadPhase(registry);

    // 2) Narrow-phase: Exact collision detection with GJK/EPA (once per frame)
    auto manifold = narrowPhase(registry, candidatePairs);
    if (manifold.collisions.empty()) {
        return;  // Early out if no collisions
    }

    // 3) Update contact manager (once), which sets up warm-start data
    ContactManager manager;
    manager.updateContacts(manifold);

    ContactSolver::solveContactConstraints(registry, manager, baumgarte, slop);

    // 5) (Optional) Position solver for any residual penetration
    PositionSolver::positionalSolver(registry, manifold, 
                                        positionalSolverIterations,
                                        baumgarte, slop);
}

} // namespace Systems