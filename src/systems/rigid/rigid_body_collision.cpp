/**
 * @file rigid_body_collision.cpp
 * @brief Implementation of the main collision system pipeline
 */

#include <vector>

#include "core/profile.hpp"
#include "systems/rigid/rigid_body_collision.hpp"
#include "systems/rigid/broadphase.hpp"
#include "systems/rigid/narrowphase.hpp"
#include "systems/rigid/contact_manager.hpp"
#include "systems/rigid/contact_solver.hpp"
#include "systems/rigid/position_solver.hpp"
#include "systems/rigid/collision_data.hpp"

namespace Systems
{

RigidBodyCollisionSystem::RigidBodyCollisionSystem() {
    // Initialize with default configurations
}

void RigidBodyCollisionSystem::update(entt::registry &registry)
{
    PROFILE_SCOPE("RigidBodyCollisionSystem");
    using namespace RigidBodyCollision;

    // 1) Broad-phase: Quick AABB-based filtering (once per frame)
    BroadphaseConfig bpConfig = BroadphaseConfig();
    auto candidatePairs = Broadphase::detectCollisions(registry, sysConfig, bpConfig);

    // 2) Narrow-phase: Exact collision detection with GJK/EPA (once per frame)
    auto manifold = narrowPhase(registry, candidatePairs);
    if (manifold.collisions.empty()) {
        return;  // Early out if no collisions
    }

    // 3) Update contact manager (once), which sets up warm-start data
    ContactManager manager;
    manager.updateContacts(manifold);

    // 4) Contact solver (iterative velocity constraints)
    ContactSolverConfig solverConfig;
    ContactSolver::solveContactConstraints(registry, manager, solverConfig);

    // 5) Position solver for any residual penetration
    PositionSolverConfig positionConfig;
    PositionSolver::positionalSolver(registry, manifold, positionConfig);
}

} // namespace Systems