/**
 * @file broadphase.hpp
 * @brief Broad-phase collision detection system using spatial partitioning
 *
 * This module implements a quadtree-based spatial partitioning system to efficiently
 * identify potential collision pairs between rigid bodies. It reduces the number of
 * detailed collision checks needed by filtering out pairs that cannot possibly collide
 * based on their axis-aligned bounding boxes (AABBs).
 */

#ifndef COLLISION_BROADPHASE_HPP
#define COLLISION_BROADPHASE_HPP

#include <vector>
#include <memory>
#include <entt/entt.hpp>
#include "nbody/systems/rigid_body_collision/collision_data.hpp"

namespace RigidBodyCollision {

/**
 * @brief Performs broad-phase collision detection on all rigid bodies
 * 
 * @param registry ECS registry containing position, mass, and particle phase components
 * @return Vector of entity pairs that potentially collide based on AABB overlap
 * 
 * @note The returned pairs are ordered such that the first entity ID is always
 *       less than the second to prevent duplicate checks
 */
std::vector<CandidatePair> broadPhase(entt::registry &registry);

} // namespace RigidBodyCollision

#endif // COLLISION_BROADPHASE_HPP