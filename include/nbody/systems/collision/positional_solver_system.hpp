#ifndef POSITIONAL_SOLVER_SYSTEM_HPP
#define POSITIONAL_SOLVER_SYSTEM_HPP

#include <entt/entt.hpp>
#include "nbody/systems/collision/collision_data.hpp"

/**
 * The PositionalSolverSystem runs after collision responses are applied,
 * performing additional passes of positional correction. This helps to reduce
 * lingering penetrations and thus reduce jitter.
 */
namespace Systems {
    class PositionalSolverSystem {
    public:
        // iterations: how many positional correction passes to run
        // baumgarte: factor for positional correction
        // slop: penetration slop
        static void update(entt::registry &registry, CollisionManifold &manifold, 
                           int iterations = 2, double baumgarte = 0.5, double slop = 0.001);
    };
}

#endif