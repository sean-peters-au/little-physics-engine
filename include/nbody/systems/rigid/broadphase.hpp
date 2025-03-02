/**
 * @file broadphase.hpp
 * @brief Broad-phase collision detection system using spatial partitioning
 *
 * This module implements a quadtree-based spatial partitioning system to efficiently
 * identify potential collision pairs between rigid bodies. It reduces the number of
 * detailed collision checks needed by filtering out pairs that cannot possibly collide
 * based on their axis-aligned bounding boxes (AABBs).
 */

#pragma once

#include <vector>
#include <memory>
#include <entt/entt.hpp>
#include "nbody/systems/i_system.hpp"
#include "nbody/systems/rigid/collision_data.hpp"

namespace RigidBodyCollision {

/**
 * @struct BroadphaseConfig
 * @brief Configuration parameters specific to the broadphase collision detection
 */
struct BroadphaseConfig {
    // Maximum objects per quadtree node before subdivision
    int quadtreeCapacity = 8;
    
    // Extra buffer size around universe for boundary objects
    double boundaryBuffer = 500.0;
};

/**
 * @class Broadphase
 * @brief Implements broad-phase collision detection using a quadtree
 * 
 * This utility class identifies potential collision pairs by partitioning space
 * using a quadtree and checking for AABB overlaps.
 */
class Broadphase {
public:
    /**
     * @brief Performs broad-phase collision detection and returns candidate pairs
     * @param registry EnTT registry containing entities and components
     * @param config System configuration parameters
     * @param bpConfig Broadphase specific configuration
     * @return Vector of entity pairs that potentially collide
     */
    static std::vector<CandidatePair> detectCollisions(
        entt::registry& registry,
        const SystemConfig& sysConfig,
        const BroadphaseConfig& bpConfig = BroadphaseConfig()
    );
};

} // namespace RigidBodyCollision
