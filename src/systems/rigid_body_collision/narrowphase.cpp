/**
 * @file narrowphase.cpp
 * @brief Implementation of GJK/EPA-based detailed collision detection
 */

#include <cmath>
#include <vector>
#include <entt/entt.hpp>
#include <iostream>

#include "nbody/systems/rigid_body_collision/narrowphase.hpp"
#include "nbody/systems/rigid_body_collision/collision_data.hpp"
#include "nbody/algo/gjk.hpp"
#include "nbody/algo/epa.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/math/vector_math.hpp"
#include "nbody/math/polygon.hpp"
#include "nbody/core/profile.hpp"

namespace RigidBodyCollision {

/**
 * @brief Extracts shape data from an entity for GJK/EPA processing
 * 
 * Combines position, rotation, and shape information (circle or polygon)
 * into a unified format for collision detection algorithms.
 */
static ShapeData extractShapeData(entt::registry &reg, entt::entity e) {
    ShapeData sd;
    sd.pos = reg.get<Components::Position>(e);
    sd.angle = 0.0;
    if (reg.any_of<Components::AngularPosition>(e)) {
        sd.angle = reg.get<Components::AngularPosition>(e).angle;
    }
    if (reg.any_of<CircleShape>(e)) {
        sd.isCircle = true;
        sd.radius = reg.get<CircleShape>(e).radius;
    } else {
        sd.isCircle = false;
        sd.radius = 0.0;
        sd.poly = reg.get<PolygonShape>(e);
    }
    return sd;
}

/**
 * @brief Generates world-space vertices for a shape
 * 
 * For circles: Creates an octagonal approximation
 * For polygons: Transforms local vertices to world space
 */
static std::vector<Vector> getWorldVerts(const ShapeData &shape) {
    std::vector<Vector> verts;
    if (shape.isCircle) {
        // Approximate circle with 8-point polygon
        const int samples = 8;
        double step = (2.0 * M_PI) / samples;
        for (int i=0; i<samples; i++) {
            double angle = i * step + shape.angle;
            double cx = shape.pos.x + shape.radius * std::cos(angle);
            double cy = shape.pos.y + shape.radius * std::sin(angle);
            verts.push_back(Vector(cx, cy));
        }
    } else {
        // Transform polygon vertices to world space
        for (auto &v : shape.poly.vertices) {
            double rx = v.x * std::cos(shape.angle) - v.y * std::sin(shape.angle);
            double ry = v.x * std::sin(shape.angle) + v.y * std::cos(shape.angle);
            double wx = shape.pos.x + rx;
            double wy = shape.pos.y + ry;
            verts.push_back(Vector(wx, wy));
        }
    }
    return verts;
}

/**
 * @brief Computes accurate contact points for a collision
 * 
 * Uses supporting vertices along the collision normal to determine
 * the actual contact points between shapes.
 */
static void findAccurateContact(const ShapeData &A,
                              const ShapeData &B,
                              const Vector &normal,
                              double penetration,
                              Vector &contactPoint)  // Single output point
{
    // Get vertices in world space
    auto vertsA = getWorldVerts(A);
    auto vertsB = getWorldVerts(B);

    // Find the deepest point of B into A (this is where the collision is happening)
    double deepestProj = 1e30;
    Vector deepestPoint;

    for (const auto &v : vertsB) {
        double proj = v.dotProduct(normal);  // Project onto collision normal
        if (proj < deepestProj) {
            deepestProj = proj;
            deepestPoint = v;
        }
    }

    // The contact point is where B intersects with A
    contactPoint = deepestPoint;
}

/**
 * @brief Creates collision info from contact points
 * 
 * Builds the final collision information structure using the
 * computed contact points and collision normal.
 */
static CollisionInfo buildSingleContact(entt::entity eA,
                                     entt::entity eB,
                                     const ShapeData &A,
                                     const ShapeData &B,
                                     const Vector &n,
                                     double penetration)
{
    Vector contactPoint;
    findAccurateContact(A, B, n, penetration, contactPoint);
    
    CollisionInfo info;
    info.a = eA;
    info.b = eB;
    info.normal = n;
    info.penetration = penetration;
    info.contactPoint = contactPoint;
    return info;
}

CollisionManifold narrowPhase(entt::registry &registry,
                             const std::vector<CandidatePair> &pairs)
{
    PROFILE_SCOPE("NarrowPhase");

    CollisionManifold manifold;
    manifold.clear();

    // Process each candidate pair from broad-phase
    for (auto &cp : pairs) {
        if (!registry.valid(cp.eA) || !registry.valid(cp.eB)) continue;

        // Extract shape data for GJK/EPA
        auto sdA = extractShapeData(registry, cp.eA);
        auto sdB = extractShapeData(registry, cp.eB);

        // Run GJK to detect intersection
        Simplex simplex;
        if (GJKIntersect(sdA, sdB, simplex)) {
            // Run EPA to get contact information
            auto epaRes = EPA(sdA, sdB, simplex);
            if (epaRes.has_value()) {
                EPAResult res = epaRes.value();
                // Build contact info for solver
                auto contact = buildSingleContact(cp.eA, cp.eB,
                                               sdA, sdB,
                                               res.normal,
                                               res.penetration);
                manifold.collisions.push_back(contact);
            }
        }
    }

    return manifold;
}

} // namespace RigidBodyCollision