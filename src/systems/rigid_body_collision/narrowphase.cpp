#include <cmath>
#include <vector>

#include <entt/entt.hpp>

#include "nbody/systems/rigid_body_collision/narrowphase.hpp"
#include "nbody/systems/rigid_body_collision/collision_data.hpp"
#include "nbody/algo/gjk.hpp"
#include "nbody/algo/epa.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/math/vector_math.hpp"
#include "nbody/math/polygon.hpp"

namespace RigidBodyCollision
{

// ------------------------------------------------------------------
// Extract shape data from ECS entity (used by GJK/EPA)
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

// ------------------------------------------------------------------
// Single “findAccurateContact” fallback for circle or simple approach
static std::vector<Vector> getWorldVerts(const ShapeData &shape)
{
    std::vector<Vector> verts;
    if (shape.isCircle) {
        // approximate circle by sampling
        const int samples = 8;
        double step = (2.0 * M_PI) / samples;
        for (int i=0; i<samples; i++) {
            double angle = i * step + shape.angle;
            double cx = shape.pos.x + shape.radius * std::cos(angle);
            double cy = shape.pos.y + shape.radius * std::sin(angle);
            verts.push_back(Vector(cx, cy));
        }
    } else {
        // transform polygon vertices
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

// fallback for circle-or-limited approach
static void findAccurateContact(const ShapeData &A,
                                const ShapeData &B,
                                const Vector &normal,
                                double /*penetration*/,
                                Vector &contactA,
                                Vector &contactB)
{
    // We'll sample the “supporting face” in +normal for A, and in -normal for B
    auto getMaxDot = [&](const ShapeData &S, const Vector &dir){
        auto wv = getWorldVerts(S);
        if (wv.empty()) {
            // fallback
            return Vector(S.pos.x, S.pos.y);
        }
        double bestVal = -1e30;
        Vector bestPt;
        for (auto &p : wv) {
            double d = p.dotProduct(dir);
            if (d > bestVal) {
                bestVal = d;
                bestPt = p;
            }
        }
        return bestPt;
    };

    Vector supA = getMaxDot(A,  normal);
    Vector supB = getMaxDot(B, -normal);

    contactA = supA;
    contactB = supB;
}

// ------------------------------------------------------------------
// For polygon–polygon, we might do a “build manifold” approach, but
// for now, we do single contact approach from original code
static CollisionInfo buildSingleContact(entt::entity eA,
                                        entt::entity eB,
                                        const ShapeData &A,
                                        const ShapeData &B,
                                        const Vector &n,
                                        double penetration)
{
    Vector cA, cB;
    findAccurateContact(A, B, n, penetration, cA, cB);
    Vector mid = (cA + cB)*0.5;

    CollisionInfo info;
    info.a = eA;
    info.b = eB;
    info.normal = n;
    info.penetration = penetration;
    info.contactPoint = mid;
    return info;
}

// ------------------------------------------------------------------
// The main narrowPhase
CollisionManifold narrowPhase(entt::registry &registry,
                              const std::vector<CandidatePair> &pairs)
{
    CollisionManifold manifold;
    manifold.clear();

    for (auto &cp : pairs) {
        if (!registry.valid(cp.eA) || !registry.valid(cp.eB)) continue;

        auto sdA = extractShapeData(registry, cp.eA);
        auto sdB = extractShapeData(registry, cp.eB);

        // GJK
        Simplex simplex;
        if (GJKIntersect(sdA, sdB, simplex)) {
            // EPA
            auto epaRes = EPA(sdA, sdB, simplex);
            if (epaRes.has_value()) {
                EPAResult res = epaRes.value();
                // single contact
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