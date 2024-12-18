#ifndef NARROW_PHASE_SYSTEM_HPP
#define NARROW_PHASE_SYSTEM_HPP

#include <entt/entt.hpp>
#include "nbody/components/basic.hpp"
#include "nbody/math/vector_math.hpp"
#include "nbody/systems/collision/collision_data.hpp"
#include <vector>

// We'll define a system that runs GJK/EPA and stores results
namespace Systems {
    class NarrowPhaseSystem {
    public:
        static void update(entt::registry &registry, 
                           const std::vector<CandidatePair> &candidatePairs,
                           CollisionManifold &manifold);
    };
}

#endif