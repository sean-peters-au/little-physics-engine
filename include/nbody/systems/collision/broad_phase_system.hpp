#ifndef BROAD_PHASE_SYSTEM_HPP
#define BROAD_PHASE_SYSTEM_HPP

#include <entt/entt.hpp>
#include <vector>
#include "nbody/components/basic.hpp"
#include "nbody/systems/collision/collision_data.hpp"

namespace Systems {
    class BroadPhaseSystem {
    public:
        // Update quadtree or spatial structure
        static void update(entt::registry &registry, std::vector<CandidatePair> &candidatePairs);
    };
}

#endif