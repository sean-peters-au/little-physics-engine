/**
 * @file collision_data.hpp
 * @brief Declaration of rigid collision data structures
 */

#pragma once

#include <vector>

#include <entt/entt.hpp>

#include "nbody/math/vector_math.hpp"
#include "nbody/components/basic.hpp"

// CandidatePair used by broad phase and narrow phase
struct CandidatePair {
    entt::entity eA;
    entt::entity eB;
};

// CollisionInfo used by narrow phase and response systems
struct CollisionInfo {
    entt::entity a;
    entt::entity b;
    Vector normal;
    double penetration;
    Vector contactPoint;
};

// CollisionManifold used by response systems
struct CollisionManifold {
    std::vector<CollisionInfo> collisions;
    void clear() { collisions.clear(); }
};
