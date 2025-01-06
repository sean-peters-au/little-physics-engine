#ifndef COLLISION_NARROWPHASE_HPP
#define COLLISION_NARROWPHASE_HPP

#include <vector>
#include <entt/entt.hpp>
#include "nbody/systems/rigid_body_collision/collision_data.hpp"

namespace RigidBodyCollision {

/**
 * @brief Runs GJK + EPA on candidate pairs, builds a manifold of collisions.
 */
CollisionManifold narrowPhase(entt::registry &registry,
                              const std::vector<CandidatePair> &pairs);

} // namespace RigidBodyCollision

#endif // COLLISION_NARROWPHASE_HPP