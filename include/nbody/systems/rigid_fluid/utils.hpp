/**
 * @file utils.hpp
 * @brief Shared utilities for rigid-fluid interaction systems
 */

#pragma once

#include <vector>
#include <entt/entt.hpp>

#include "nbody/math/vector_math.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sph.hpp"

namespace Systems {
namespace RigidFluidUtils {

// Data structure for fluid particles
struct FluidParticle {
    entt::entity id;
    Components::Position pos;
    Components::Velocity vel;
    double mass;
    double density;
    double smoothingLength;
    double radius;   // For position solver

    FluidParticle() : id(entt::null), mass(0), density(0), smoothingLength(0), radius(0) {}
};

// Data structure for rigid bodies
struct RigidBody {
    entt::entity id;
    Components::Position* pos;
    Components::Velocity* vel;
    Components::AngularPosition* angPos;
    Components::AngularVelocity* angVel;
    Components::Mass mass;
    Components::Inertia inertia;
    Components::Shape shape;
    const PolygonShape* polygon;
    
    // AABB for broad phase
    double minX, minY, maxX, maxY;
};

// Collect all fluid particles with position & core properties
std::vector<FluidParticle> collectFluidParticles(entt::registry& registry);

// Collect all rigid bodies with full physical properties
std::vector<RigidBody> collectRigidBodies(entt::registry& registry);

// Transform polygon to world space
std::vector<Vector> getWorldSpacePolygon(const PolygonShape& poly, 
                                       const Vector& position, 
                                       double angle);

// Properly detect if a point is inside a polygon
bool isPointInPolygon(const Vector& point, const std::vector<Vector>& polygon);

// Find closest point on a polygon edge
Vector closestPointOnPolygon(const Vector& point, const std::vector<Vector>& polygon);

// Calculate normal pointing out from a polygon
Vector calculateOutwardNormal(const Vector& point, 
                             const Vector& boundaryPoint, 
                             const std::vector<Vector>& polygon);

} // namespace RigidFluidUtils
} // namespace Systems 