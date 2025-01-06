#ifndef RIGID_BODY_COLLISION_HPP
#define RIGID_BODY_COLLISION_HPP

#include <entt/entt.hpp>
#include "nbody/systems/collision/collision_data.hpp"

/**
 * @file rigid_body_collision.hpp
 * @brief Rigid body collision system that handles broad-phase, narrow-phase, and
 *        solid collision responses (plus optional positional correction).
 */

namespace Systems {

    /**
     * @class RigidBodyCollisionSystem
     * @brief Provides a single interface to run all steps of rigid body collision:
     *        1) Broad-phase (collect collision candidates)
     *        2) Narrow-phase (GJK/EPA)
     *        3) Solid collision response
     *        4) Positional solver (optional, reduces persistent penetrations)
     */
    class RigidBodyCollisionSystem {
    public:
        /**
         * @brief Run the full rigid body collision sequence on the registry.
         * 
         * @param registry                 The main EnTT registry.
         * @param solverIterations         Number of iterations for the contact solver.
         * @param positionalSolverIterations Number of passes for positional correction.
         * @param baumgarte                Factor for positional correction.
         * @param slop                     Penetration slop to allow before correction.
         */
        static void update(entt::registry &registry,
                           int solverIterations = 5,
                           int positionalSolverIterations = 2,
                           double baumgarte = 0.1,
                           double slop = 0.001);
    };

} // namespace Systems

#endif