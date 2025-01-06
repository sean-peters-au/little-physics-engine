#ifndef COLLISION_BROADPHASE_HPP
#define COLLISION_BROADPHASE_HPP

#include <vector>
#include <memory>
#include <entt/entt.hpp>
#include "nbody/systems/rigid_body_collision/collision_data.hpp"

namespace RigidBodyCollision {

/**
 * @brief Broad-phase collision detection using a quadtree of bounding boxes.
 */
std::vector<CandidatePair> broadPhase(entt::registry &registry);

} // namespace RigidBodyCollision

#endif // COLLISION_BROADPHASE_HPP