#ifndef RIGID_BODY_COLLISION_SYSTEM_HPP
#define RIGID_BODY_COLLISION_SYSTEM_HPP

#include <entt/entt.hpp>

namespace Systems {

/**
 * @class RigidBodyCollisionSystem
 * @brief Orchestrates broad-phase, narrow-phase, contact solving, and positional solver.
 *
 * Split into multiple internal modules under `rigid_body_collision_system/`.
 */
class RigidBodyCollisionSystem {
public:
    /**
     * @brief Run full collision detection & resolution.
     * @param registry The ECS registry
     * @param solverIterations Number of velocity solver iterations
     * @param positionalSolverIterations Number of positional solver passes
     * @param baumgarte Factor for positional correction
     * @param slop Penetration slop
     */
    static void update(entt::registry &registry,
                       int solverIterations = 5,
                       int positionalSolverIterations = 2,
                       double baumgarte = 0.5,
                       double slop = 0.001);
};

} // namespace Systems

#endif // RIGID_BODY_COLLISION_SYSTEM_HPP