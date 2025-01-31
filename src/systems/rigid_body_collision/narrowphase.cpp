/**
 * @file narrowphase.cpp
 * @brief Implementation of GJK/EPA-based detailed collision detection
 *
 * Updated to produce multiple contact points for polygon–polygon collisions
 * after retrieving the single global normal from EPA. Circles remain single-contact.
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

#define ENABLE_NARROWPHASE_DEBUG 0

// Add a global debug filter flag
static bool g_debugFilter = true;

#define DEBUG(x) do { \
    if (ENABLE_NARROWPHASE_DEBUG && g_debugFilter) { \
        std::cout << "[DEBUG NARROWPHASE] " << x << std::endl; \
    } \
} while(0)

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
 * @brief Generates world-space vertices for a shape.
 * 
 * - For circles: Creates an 8-point approximation
 * - For polygons (CCW): Transforms local vertices to world space
 */
static std::vector<Vector> getWorldVerts(const ShapeData &shape) {
    std::vector<Vector> verts;
    if (shape.isCircle) {
        // Approximate circle with 8 vertices
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
 * @brief Basic contact data for circle collisions or a single contact.
 */
static CollisionInfo buildSingleContact(entt::entity eA,
                                        entt::entity eB,
                                        const ShapeData &A,
                                        const ShapeData &B,
                                        const Vector &n,
                                        double penetration,
                                        const Vector &contactPoint)
{
    CollisionInfo info;
    info.a = eA;
    info.b = eB;
    info.normal = n; // From A to B
    info.penetration = penetration;
    info.contactPoint = contactPoint;
    return info;
}

/**
 * @brief Reference face selection: pick which polygon is "reference" vs "incident."
 * 
 * We pick the polygon with the face that is most aligned to the overall collision normal
 * as the reference. The other polygon is the "incident."
 *
 * @param normal Global normal from EPA (points from A to B)
 * @param Averts CCW world verts of shape A
 * @param Bverts CCW world verts of shape B
 * @param eA entity A
 * @param eB entity B
 * @return A struct telling us which shape is reference vs. incident, along with the face index
 */
struct ReferenceChoice {
    std::vector<Vector> refVerts;
    std::vector<Vector> incVerts;
    entt::entity refEnt;
    entt::entity incEnt;
    int refFaceIndex = 0;  // the reference face index in refVerts
    Vector refNormal;      // the face normal (should closely match the global normal)
    bool flipped = false;  // if we swapped normal direction
};

/**
 * @brief Finds the polygon face that has the greatest alignment to 'normal'
 *        i.e. minimize angle or maximize dot. Return that face's index
 */
static int findBestFace(const std::vector<Vector> &verts, const Vector &normal) {
    int bestFace = 0;
    double bestDot = -1e30;
    int count = (int)verts.size();
    for (int i = 0; i < count; i++) {
        // The face from verts[i] to verts[i+1], normal is computed from CCW
        int j = (i+1) % count;
        Vector edge = verts[j] - verts[i];
        // CCW polygon => face normal can be found by rotating edge 90° left:
        // e.g. normal = Vector(-edge.y, edge.x)
        Vector faceNormal(-edge.y, edge.x);
        faceNormal = faceNormal.normalized();

        double d = faceNormal.dotProduct(normal);
        if (d > bestDot) {
            bestDot = d;
            bestFace = i;
        }
    }
    return bestFace;
}

/**
 * @brief Build a ReferenceChoice describing which shape is reference vs. incident,
 *        and which face on the reference polygon to use.
 */
static ReferenceChoice chooseReference(const Vector &globalNormal,
                                       const std::vector<Vector> &Averts,
                                       const std::vector<Vector> &Bverts,
                                       entt::entity eA,
                                       entt::entity eB)
{
    DEBUG("chooseReference: Global normal = " << globalNormal.x << ", " << globalNormal.y);
    
    int faceA = findBestFace(Averts, globalNormal);
    int faceB = findBestFace(Bverts, -globalNormal);
    
    DEBUG("Best face indices - A: " << faceA << ", B: " << faceB);

    Vector edgeA = Averts[(faceA+1)%Averts.size()] - Averts[faceA];
    Vector normA(-edgeA.y, edgeA.x);
    normA = normA.normalized();
    double dotA = normA.dotProduct(globalNormal);

    Vector edgeB = Bverts[(faceB+1)%Bverts.size()] - Bverts[faceB];
    Vector normB(-edgeB.y, edgeB.x);
    normB = normB.normalized();
    double dotB = normB.dotProduct(-globalNormal);

    DEBUG("Face A normal: " << normA.x << ", " << normA.y << " (dot: " << dotA << ")");
    DEBUG("Face B normal: " << normB.x << ", " << normB.y << " (dot: " << dotB << ")");

    // If dotA >= dotB, pick A as reference, else B as reference
    ReferenceChoice rc;
    // if (dotA >= dotB) {
    if (true) {
        rc.refEnt = eA;
        rc.incEnt = eB;
        rc.refVerts = Averts;
        rc.incVerts = Bverts;
        rc.refFaceIndex = faceA;
        rc.refNormal = normA; // should be close to globalNormal
        rc.flipped = false;
    } else {
        rc.refEnt = eB;
        rc.incEnt = eA;
        rc.refVerts = Bverts;
        rc.incVerts = Averts;
        rc.refFaceIndex = faceB;
        rc.refNormal = normB; // should be close to -globalNormal => watch sign
        rc.flipped = true;
    }
    return rc;
}

/**
 * @brief Clip the incident polygon by a plane (defined by planeNormal and planeOffset).
 * 
 * planeNormal points "inside" (toward the reference polygon). planeOffset is the
 * distance from origin to plane. We keep only the points that are inside or on the plane.
 *
 * Implementation detail: We treat plane equation as `planeNormal · X <= planeOffset`.
 *
 * @return The clipped polygon (subset).
 */
static std::vector<Vector> clipFace(const std::vector<Vector> &poly,
                                    const Vector &planeNormal,
                                    double planeOffset)
{
    std::vector<Vector> out;
    int n = (int)poly.size();
    for (int i = 0; i < n; i++) {
        int j = (i+1)%n;
        const Vector &p1 = poly[i];
        const Vector &p2 = poly[j];

        // Evaluate distances
        double d1 = planeNormal.dotProduct(p1) - planeOffset;
        double d2 = planeNormal.dotProduct(p2) - planeOffset;

        bool inside1 = (d1 <= 0.0);
        bool inside2 = (d2 <= 0.0);

        // If p1 is inside, add it
        if (inside1) out.push_back(p1);

        // If edges cross plane, add intersection
        if (inside1 != inside2) {
            double t = d1 / (d1 - d2);
            Vector intersection = p1 + (p2 - p1)*t;
            out.push_back(intersection);
        }
    }
    return out;
}

/**
 * @brief Clip the incident polygon to the reference face side-planes, returning up to 2 points.
 */
static std::vector<Vector> clipIncidentPolygon(const ReferenceChoice &rc)
{
    // 1) Build an "incident polygon" from the incVerts (likely the entire polygon).
    //    We'll successively clip it to the region of the reference face's side planes.

    // Reference face from refFaceIndex
    int i1 = rc.refFaceIndex;
    int i2 = (i1+1) % rc.refVerts.size();

    Vector v1 = rc.refVerts[i1];
    Vector v2 = rc.refVerts[i2];

    // The "center" of the face is helpful
    Vector faceCenter = (v1 + v2) * 0.5;

    // referenceFaceNormal (already in rc.refNormal)
    // We also get side planes. The reference face is CCW, so the local "up" side is inside
    // For the plane offset: planeOffset = planeNormal dot anyPointOnPlane

    // Plane 1: the reference face plane itself (we want points "behind" it)
    // But for manifold generation, we usually keep incident polygon points
    // behind the face plane by ~penetration. We'll keep it simple: no extra offset.
    double faceOffset = rc.refNormal.dotProduct(v1);

    // We also need the "top" and "bottom" boundaries of this face: each face has 2 side planes if we want to bound it in a segment.
    // Actually we can get them from the face's endpoints by building outward normals.

    // We'll define 2 side normals:
    //   - The normal from v1 -> v2 is edge direction. We'll define a left-plane and right-plane in local sense.

    // direction of the edge
    Vector edge = v2 - v1;
    edge = edge.normalized();
    // "left" plane normal: rotate edge 90 deg CCW => Vector(-edge.y, edge.x)
    // "right" plane normal: rotate edge 90 deg CW => Vector(edge.y, -edge.x)

    // But we just want the planes that bound the reference face segment in the local normal direction. 
    // Let's define them carefully:

    // Plane "top" => normal = +edge, offset = edge · v2  (since v2 is the 'end' of the face)
    // Plane "bottom" => normal = -edge, offset = -edge · v1 (or edge · v1 with sign flipped)
    Vector planeTopNormal = edge; 
    double planeTopOffset = planeTopNormal.dotProduct(v2);

    Vector planeBotNormal = -edge;
    double planeBotOffset = planeBotNormal.dotProduct(v1);

    // Now we clip rc.incVerts step by step
    std::vector<Vector> poly = rc.incVerts;

    // Step 1: clip by the reference face plane (rc.refNormal)
    poly = clipFace(poly,  rc.refNormal,  faceOffset);

    // Step 2: clip by planeTopNormal
    poly = clipFace(poly,  planeTopNormal, planeTopOffset);

    // Step 3: clip by planeBotNormal
    poly = clipFace(poly,  planeBotNormal, planeBotOffset);

    return poly;
}

// Helper function to check if entity is a boundary
static bool isBoundary(const entt::registry &registry, entt::entity e) {
    return registry.valid(e) && registry.any_of<Components::Boundary>(e);
}

/**
 * @brief Builds multiple collision points for polygon–polygon after EPA.
 *        We'll return up to 2 contact points from reference-face clipping.
 */
static std::vector<CollisionInfo> buildPolygonPolygonContacts(
    entt::entity eA,
    entt::entity eB,
    const ShapeData &A,
    const ShapeData &B,
    const EPAResult &res,
    entt::registry &registry)
{
    // Set debug filter based on boundary status
    g_debugFilter = !isBoundary(registry, eA) && !isBoundary(registry, eB);

    DEBUG("\n=== New Polygon-Polygon Contact ===");
    DEBUG("EPA Normal: " << res.normal.x << ", " << res.normal.y);
    DEBUG("EPA Penetration: " << res.penetration);

    std::vector<CollisionInfo> contacts;
    auto Averts = getWorldVerts(A);
    auto Bverts = getWorldVerts(B);

    DEBUG("Shape A vertices:");
    for (const auto& v : Averts) {
        DEBUG("  (" << v.x << ", " << v.y << ")");
    }
    DEBUG("Shape B vertices:");
    for (const auto& v : Bverts) {
        DEBUG("  (" << v.x << ", " << v.y << ")");
    }

    Vector globalNormal = res.normal;
    ReferenceChoice rc = chooseReference(globalNormal, Averts, Bverts, eA, eB);

    DEBUG("Reference choice:");
    DEBUG("  Reference entity: " << (rc.refEnt == eA ? "A" : "B"));
    DEBUG("  Reference normal: " << rc.refNormal.x << ", " << rc.refNormal.y);
    DEBUG("  Flipped: " << (rc.flipped ? "yes" : "no"));

    auto clipped = clipIncidentPolygon(rc);
    
    DEBUG("Clipped points:");
    for (const auto& pt : clipped) {
        DEBUG("  (" << pt.x << ", " << pt.y << ")");
    }

    // We'll unify how we store normal for the final contact:
    // If rc.flipped is true, we want to invert the normal. Because from the solver perspective,
    // normal always points from A to B. So if B is reference, that means the normal we found
    // might actually need flipping. Let's define finalNormal accordingly:
    Vector finalNormal = (rc.flipped) ? -globalNormal : globalNormal;

    // We find the reference face plane offset
    int i1 = rc.refFaceIndex;
    int i2 = (i1+1) % rc.refVerts.size();
    Vector refV1 = rc.refVerts[i1];
    Vector refV2 = rc.refVerts[i2];
    double planeOffset = rc.refNormal.dotProduct(refV1);

    // Create collision info for each contact point
    for (const auto &contactPoint : clipped) {
        // Calculate penetration depth for this contact point
        double penetration = -(rc.refNormal.dotProduct(contactPoint) - planeOffset);
        
        CollisionInfo ci;
        ci.normal = finalNormal;
        ci.penetration = penetration;
        ci.contactPoint = contactPoint;
        ci.a = eA;
        ci.b = eB;
        contacts.push_back(ci);
    }

    // We'll clamp to at most 2 contact points (common practice)
    // while (contacts.size() > 2) {
    //     contacts.pop_back();
    // }

    DEBUG("Final contacts generated: " << contacts.size());
    for (const auto& contact : contacts) {
        DEBUG("=== New Contact ===");
        DEBUG("\tContact point: (" << contact.contactPoint.x << ", " << contact.contactPoint.y << ")");
        DEBUG("\tContact normal: (" << contact.normal.x << ", " << contact.normal.y << ")");
        DEBUG("\tPenetration: " << contact.penetration);
    }
    DEBUG("=== End Contact ===\n");

    return contacts;
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

        // Set debug filter based on boundary status
        g_debugFilter = !isBoundary(registry, cp.eA) && !isBoundary(registry, cp.eB);

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
                // For circle–anything or anything–circle, we still do single-contact
                if (sdA.isCircle && sdB.isCircle) {
                    // circle–circle => single contact
                    // find the one contact point along the normal
                    Vector contactPt = Vector(sdB.pos) - res.normal * (sdB.radius);
                    auto contact = buildSingleContact(cp.eA, cp.eB, sdA, sdB,
                                                      res.normal,
                                                      res.penetration,
                                                      contactPt);
                    manifold.collisions.push_back(contact);
                }
                else if (sdA.isCircle && !sdB.isCircle) {
                    // circle–polygon => single contact:
                    // contact point is the circle center minus normal * radius
                    Vector contactPt = Vector(sdA.pos) + res.normal * (sdA.radius);
                    auto contact = buildSingleContact(cp.eA, cp.eB,
                                                      sdA, sdB,
                                                      res.normal,
                                                      res.penetration,
                                                      contactPt);
                    manifold.collisions.push_back(contact);
                }
                else if (!sdA.isCircle && sdB.isCircle) {
                    // polygon–circle => single contact:
                    // contact point is circle center minus normal * radius
                    // but normal points from A->B, so B is circle => B center is sdB.pos
                    // The circle boundary = center - normal * radius
                    Vector contactPt = Vector(sdB.pos) - res.normal * (sdB.radius);
                    auto contact = buildSingleContact(cp.eA, cp.eB,
                                                      sdA, sdB,
                                                      res.normal,
                                                      res.penetration,
                                                      contactPt);
                    manifold.collisions.push_back(contact);
                }
                else {
                    // polygon–polygon => produce multiple contact points
                    auto multContacts = buildPolygonPolygonContacts(
                        cp.eA, cp.eB, sdA, sdB, res, registry);
                    for (auto &cinfo : multContacts) {
                        manifold.collisions.push_back(cinfo);
                    }
                }
            }
        }
    }

    // Reset debug filter for final message
    g_debugFilter = true;
    DEBUG("=== Narrow Phase Complete ===\n");

    return manifold;
}

} // namespace RigidBodyCollision