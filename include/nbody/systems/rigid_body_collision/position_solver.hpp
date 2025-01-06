#ifndef POSITION_SOLVER_HPP
#define POSITION_SOLVER_HPP

#include <entt/entt.hpp>
#include "nbody/systems/rigid_body_collision/collision_data.hpp"

namespace RigidBodyCollision {

/**
 * @brief Correct leftover penetrations (split impulse style) with a positional pass.
 */
class PositionSolver {
public:
    /**
     * @brief Fix overlap using a simple baumgarte approach. 
     */
    static void positionalSolver(entt::registry &registry,
                                 const CollisionManifold &manifold,
                                 int iterations,
                                 double baumgarte,
                                 double slop);
};

} // namespace RigidBodyCollision

#endif // POSITION_SOLVER_HPP