/**
 * @file narrowphase.hpp
 * @brief Detailed collision detection using GJK and EPA algorithms
 *
 * This module performs exact collision detection on pairs of shapes identified
 * by the broad-phase. It uses the Gilbert-Johnson-Keerthi (GJK) algorithm to
 * detect intersections and the Expanding Polytope Algorithm (EPA) to compute
 * contact information when collisions are found.
 */

#ifndef COLLISION_NARROWPHASE_HPP
#define COLLISION_NARROWPHASE_HPP

#include <vector>
#include <entt/entt.hpp>
#include "nbody/systems/rigid_body_collision/collision_data.hpp"

namespace RigidBodyCollision {

/**
 * @brief Performs detailed collision detection on candidate pairs
 * 
 * @param registry ECS registry containing shape and transform components
 * @param pairs Candidate collision pairs from broad-phase
 * @return CollisionManifold containing detailed contact information
 * 
 * For each candidate pair, this function:
 * 1. Extracts shape data from the ECS
 * 2. Runs GJK to detect intersection
 * 3. If intersecting, runs EPA to compute contact normal and depth
 * 4. Generates accurate contact points for the solver
 */
CollisionManifold narrowPhase(entt::registry &registry,
                             const std::vector<CandidatePair> &pairs);

} // namespace RigidBodyCollision

#endif // COLLISION_NARROWPHASE_HPP