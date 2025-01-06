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

    // We create a contact manager (later, you might store it persistently)
    ContactManager manager;

    // Multiple passes for velocity solver
    for (int pass = 0; pass < solverIterations; pass++) {
        // 1) Broad-phase
        auto candidatePairs = broadPhase(registry);

        // 2) Narrow-phase => find collisions
        auto manifold = narrowPhase(registry, candidatePairs);

        if (manifold.collisions.empty()) {
            break; // No collisions => done
        }

        // 3) Update contact manager (no-op for now), then solve
        manager.updateContacts(manifold);

        // 4) Velocity solver (split impulse)
        ContactSolver::solveContactConstraints(registry, manager, baumgarte, slop);
    }

    // 5) Extra positional passes if desired
    if (positionalSolverIterations > 0) {
        // do final broad-phase + narrow-phase for leftover overlaps
        auto candidatePairs = broadPhase(registry);
        auto manifold = narrowPhase(registry, candidatePairs);
        if (!manifold.collisions.empty()) {
            PositionSolver::positionalSolver(registry, manifold,
                                             positionalSolverIterations,
                                             baumgarte, slop);
        }
    }

    // Optionally manager.cleanupStaleContacts(registry);
}

} // namespace Systems