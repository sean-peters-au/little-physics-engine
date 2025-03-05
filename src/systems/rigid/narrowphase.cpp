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

#include "nbody/systems/rigid/narrowphase.hpp"
#include "nbody/systems/rigid/collision_data.hpp"
#include "nbody/systems/rigid/gjk.hpp"
#include "nbody/systems/rigid/epa.hpp"
#include "nbody/entities/entity_components.hpp"
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
        double const step = (2.0 * M_PI) / samples;
        for (int i=0; i<samples; i++) {
            double const angle = i * step + shape.angle;
            double const cx = shape.pos.x + shape.radius * std::cos(angle);
            double const cy = shape.pos.y + shape.radius * std::sin(angle);
            verts.emplace_back(cx, cy);
        }
    } else {
        // Transform polygon vertices to world space
        for (const auto &v : shape.poly.vertices) {
            double const rx = v.x * std::cos(shape.angle) - v.y * std::sin(shape.angle);
            double const ry = v.x * std::sin(shape.angle) + v.y * std::cos(shape.angle);
            double const wx = shape.pos.x + rx;
            double const wy = shape.pos.y + ry;
            verts.emplace_back(wx, wy);
        }
    }
    return verts;
}

/**
 * @brief Basic contact data for circle collisions or a single contact.
 */
static CollisionInfo buildSingleContact(entt::entity eA,
                                        entt::entity eB,
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
    int const count = static_cast<int>(verts.size());
    for (int i = 0; i < count; i++) {
        // The face from verts[i] to verts[i+1], normal is computed from CCW
        int const j = (i+1) % count;
        Vector const edge = verts[j] - verts[i];
        // CCW polygon => face normal can be found by rotating edge 90° left:
        // e.g. normal = Vector(-edge.y, edge.x)
        Vector faceNormal(-edge.y, edge.x);
        faceNormal = faceNormal.normalized();

        double const d = faceNormal.dotProduct(normal);
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
                                       const std::vector<Vector> &averts,
                                       const std::vector<Vector> &bverts,
                                       entt::entity eA,
                                       entt::entity eB)
{
    int const faceA = findBestFace(averts, globalNormal);
    int const faceB = findBestFace(bverts, -globalNormal);

    Vector const edgeA = averts[(faceA+1)%averts.size()] - averts[faceA];
    Vector normA(-edgeA.y, edgeA.x);
    normA = normA.normalized();
    double const dotA = normA.dotProduct(globalNormal);

    Vector const edgeB = bverts[(faceB+1)%bverts.size()] - bverts[faceB];
    Vector normB(-edgeB.y, edgeB.x);
    normB = normB.normalized();
    double const dotB = normB.dotProduct(-globalNormal);

    // If dotA >= dotB, pick A as reference, else B as reference
    ReferenceChoice rc;
    // if (dotA >= dotB) {
    if (true) {
        rc.refEnt = eA;
        rc.incEnt = eB;
        rc.refVerts = averts;
        rc.incVerts = bverts;
        rc.refFaceIndex = faceA;
        rc.refNormal = normA; // should be close to globalNormal
        rc.flipped = false;
    } else {
        rc.refEnt = eB;
        rc.incEnt = eA;
        rc.refVerts = bverts;
        rc.incVerts = averts;
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
    int const n = static_cast<int>(poly.size());
    for (int i = 0; i < n; i++) {
        int const j = (i+1)%n;
        const Vector &p1 = poly[i];
        const Vector &p2 = poly[j];

        // Evaluate distances
        double const d1 = planeNormal.dotProduct(p1) - planeOffset;
        double const d2 = planeNormal.dotProduct(p2) - planeOffset;

        bool const inside1 = (d1 <= 0.0);
        bool const inside2 = (d2 <= 0.0);

        // If p1 is inside, add it
        if (inside1) { out.push_back(p1);
}

        // If edges cross plane, add intersection
        if (inside1 != inside2) {
            double const t = d1 / (d1 - d2);
            Vector const intersection = p1 + (p2 - p1)*t;
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
    int const i1 = rc.refFaceIndex;
    int const i2 = (i1+1) % rc.refVerts.size();

    Vector const v1 = rc.refVerts[i1];
    Vector const v2 = rc.refVerts[i2];

    // The "center" of the face is helpful
    Vector const faceCenter = (v1 + v2) * 0.5;

    // referenceFaceNormal (already in rc.refNormal)
    // We also get side planes. The reference face is CCW, so the local "up" side is inside
    // For the plane offset: planeOffset = planeNormal dot anyPointOnPlane

    // Plane 1: the reference face plane itself (we want points "behind" it)
    // But for manifold generation, we usually keep incident polygon points
    // behind the face plane by ~penetration. We'll keep it simple: no extra offset.
    double const faceOffset = rc.refNormal.dotProduct(v1);

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
    Vector const planeTopNormal = edge; 
    double const planeTopOffset = planeTopNormal.dotProduct(v2);

    Vector const planeBotNormal = -edge;
    double const planeBotOffset = planeBotNormal.dotProduct(v1);

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

/**
 * @brief Builds multiple collision points for polygon–polygon after EPA.
 *        We'll return up to 2 contact points from reference-face clipping.
 */
static std::vector<CollisionInfo> buildPolygonPolygonContacts(
    entt::entity eA,
    entt::entity eB,
    const ShapeData &a,
    const ShapeData &b,
    const EPAResult &res,
    entt::registry &registry)
{
    std::vector<CollisionInfo> contacts;
    auto averts = getWorldVerts(a);
    auto bverts = getWorldVerts(b);

    Vector const globalNormal = res.normal;
    ReferenceChoice rc = chooseReference(globalNormal, averts, bverts, eA, eB);

    auto clipped = clipIncidentPolygon(rc);

    // We'll unify how we store normal for the final contact:
    // If rc.flipped is true, we want to invert the normal. Because from the solver perspective,
    // normal always points from A to B. So if B is reference, that means the normal we found
    // might actually need flipping. Let's define finalNormal accordingly:
    Vector const finalNormal = (rc.flipped) ? -globalNormal : globalNormal;

    // We find the reference face plane offset
    int const i1 = rc.refFaceIndex;
    int const i2 = (i1+1) % rc.refVerts.size();
    Vector const refV1 = rc.refVerts[i1];
    Vector const refV2 = rc.refVerts[i2];
    double const planeOffset = rc.refNormal.dotProduct(refV1);

    // Create collision info for each contact point
    for (const auto &contactPoint : clipped) {
        // Calculate penetration depth for this contact point
        double const penetration = -(rc.refNormal.dotProduct(contactPoint) - planeOffset);
        
        CollisionInfo ci;
        ci.normal = finalNormal;
        ci.penetration = penetration;
        ci.contactPoint = contactPoint;
        ci.a = eA;
        ci.b = eB;
        contacts.push_back(ci);
    }

    return contacts;
}

CollisionManifold narrowPhase(entt::registry &registry,
                             const std::vector<CandidatePair> &pairs)
{
    PROFILE_SCOPE("NarrowPhase");

    CollisionManifold manifold;
    manifold.clear();

    // Process each candidate pair from broad-phase
    for (const auto &cp : pairs) {
        if (!registry.valid(cp.eA) || !registry.valid(cp.eB)) { continue; }

        // Extract shape data for GJK/EPA
        auto sdA = extractShapeData(registry, cp.eA);
        auto sdB = extractShapeData(registry, cp.eB);

        // Run GJK to detect intersection
        Simplex simplex;
        if (GJKIntersect(sdA, sdB, simplex)) {
            // Run EPA to get contact information
            auto epaRes = EPA(sdA, sdB, simplex);
            if (epaRes.has_value()) {
                EPAResult const res = epaRes.value();
                // For circle–anything or anything–circle, we still do single-contact
                if (sdA.isCircle && sdB.isCircle) {
                    // circle–circle => single contact
                    // find the one contact point along the normal
                    Vector const contactPt = Vector(sdB.pos) - res.normal * (sdB.radius);
                    auto contact = buildSingleContact(cp.eA, cp.eB, res.normal,
                                                      res.penetration,
                                                      contactPt);
                    manifold.collisions.push_back(contact);
                }
                else if (sdA.isCircle && !sdB.isCircle) {
                    // circle–polygon => single contact:
                    // contact point is the circle center minus normal * radius
                    Vector const contactPt = Vector(sdA.pos) + res.normal * (sdA.radius);
                    auto contact = buildSingleContact(cp.eA, cp.eB,
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
                    Vector const contactPt = Vector(sdB.pos) - res.normal * (sdB.radius);
                    auto contact = buildSingleContact(cp.eA, cp.eB,
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

    return manifold;
}

} // namespace RigidBodyCollision