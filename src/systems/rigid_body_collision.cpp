/**
 * @file rigid_body_collision.cpp
 * @brief Rigid-body collision system supporting arbitrary polygons (and circles)
 *        via GJK/EPA. Now uses a more accurate method to compute the collision
 *        contact point by finding the supporting faces of each shape along
 *        the EPA normal, rather than naively shifting shape centers.
 *
 *        This helps prevent exaggerated torque/rotation from large contact offsets.
 */

#include "nbody/systems/rigid_body_collision.hpp"
#include <memory>
#include <algorithm>
#include <cmath>
#include <vector>
#include <entt/entt.hpp>

#include "nbody/core/constants.hpp"
#include "nbody/algo/gjk.hpp"          // GJK algorithm
#include "nbody/algo/epa.hpp"          // EPA algorithm
#include "nbody/components/basic.hpp"  // Position, Velocity, Mass, etc.
#include "nbody/math/vector_math.hpp"  // Vector operations
#include "nbody/math/polygon.hpp"      // For polygon shape
#include "nbody/systems/collision/collision_data.hpp"

namespace {

/**
 * @brief For a polygon or circle, compute its world-space vertices.
 *        (Circles just produce a single vertex at center + radius in the normal direction.)
 */
static std::vector<Vector> getWorldVerts(const ShapeData &shape)
{
    std::vector<Vector> verts;
    if (shape.isCircle) {
        // We'll produce a small set of "sample" vertices for a circle,
        // or simply treat the circle center as a single "vertex" if we want
        // a specialized approach. For contact points, we do need the actual edge.
        // Minimal approach: produce a single vertex "center + normal * radius"
        // (But that only works if we know the normal. We'll do a general approach with multiple samples.)
        // For simplicity, let's produce 8 sample vertices around the circle.
        const int samples = 8;
        double step = (2.0 * M_PI) / samples;
        for (int i=0; i<samples; i++) {
            double angle = i * step + shape.angle; 
            double cx = shape.pos.x + shape.radius * std::cos(angle);
            double cy = shape.pos.y + shape.radius * std::sin(angle);
            verts.push_back(Vector(cx, cy));
        }
    }
    else {
        // Polygon => transform each vertex by shape.angle and shape.pos
        for (auto &v : shape.poly.vertices) {
            // rotate
            double rx = (v.x * std::cos(shape.angle)) - (v.y * std::sin(shape.angle));
            double ry = (v.x * std::sin(shape.angle)) + (v.y * std::cos(shape.angle));
            // translate
            double wx = shape.pos.x + rx;
            double wy = shape.pos.y + ry;
            verts.push_back(Vector(wx, wy));
        }
    }
    return verts;
}

/**
 * @brief Find the supporting face (or point) for a shape in a given direction.
 *
 * For a polygon, we gather all vertices that are within some tiny epsilon of
 * the maximum projection onto 'dir'. Those vertices typically define an edge or
 * a single point if it's a corner. For a circle, we produce up to 2 "support" points
 * if the shape is discretized (like above), or just one if we prefer.
 */
static std::vector<Vector> findSupportingFace(const std::vector<Vector> &worldVerts,
                                              const Vector &dir)
{
    // 1) Project all vertices onto dir, track the max dot
    double maxProj = -1e30;
    std::vector<double> projVals;
    projVals.reserve(worldVerts.size());
    for (auto &w : worldVerts) {
        double dot = w.dotProduct(dir);
        projVals.push_back(dot);
        if (dot > maxProj) {
            maxProj = dot;
        }
    }

    // 2) Gather all vertices whose projection is within EPS of maxProj
    static const double EPS_FACE = 1e-6; 
    std::vector<Vector> support;
    support.reserve(4);
    for (size_t i=0; i<worldVerts.size(); i++) {
        if (std::fabs(projVals[i] - maxProj) < EPS_FACE) {
            support.push_back(worldVerts[i]);
        }
    }

    // Typically 1 or 2 vertices define the "support edge" for a polygon.
    // If there's more, it might be a flat side or circle approximation => keep them all.
    return support;
}

/**
 * @brief Given the normal from EPA, find more accurate contact points on each shape
 *        by gathering the supporting face(s) from shape A in direction +normal,
 *        and from shape B in direction -normal, then computing the "closest approach"
 *        between these two edges or sets of points.
 *
 * This yields two points contactA, contactB on each shape's boundary, and you can
 * pick the midpoint as the final contact point. 
 */
static void findAccurateContact(const ShapeData &A,
                                const ShapeData &B,
                                const Vector &normal, 
                                double penetration,
                                Vector &contactA,
                                Vector &contactB)
{
    // 1) Gather all world verts for each shape
    auto vertsA = getWorldVerts(A);
    auto vertsB = getWorldVerts(B);

    // 2) Find supporting face on A in direction of +normal
    auto faceA = findSupportingFace(vertsA,  normal);
    // 3) Find supporting face on B in direction of -normal
    auto faceB = findSupportingFace(vertsB,  -normal);

    // If either is empty, fallback:
    if (faceA.empty()) {
        // fallback: put contact at A's center
        contactA = Vector(A.pos.x, A.pos.y);
    }
    if (faceB.empty()) {
        // fallback: put contact at B's center
        contactB = Vector(B.pos.x, B.pos.y);
    }

    // If we got 1 vertex for A and 1 vertex for B, easy:
    //   the contact points are basically those points or we offset them by half pen.
    // If we have edges, we want the closest pair of points between the two edges.

    // For simplicity, let's define a helper function to getClosestPointsBetweenTwoPolylines.
    // We'll do a minimal version that checks each segment in faceA against each segment in faceB,
    // and finds the closest pair. If faceA or faceB has 1 point, treat it as a degenerate segment.

    // We'll define them as polylines in ring order (though we might only have 1 or 2 points).
    // We'll gather the min distance pair. Then we'll clamp if we have circle approximations, etc.

    // Step A: build edges from faceA
    std::vector<std::pair<Vector,Vector>> edgesA;
    if (faceA.size() == 1) {
        edgesA.push_back({faceA[0], faceA[0]});
    }
    else {
        // form consecutive edges
        for (size_t i=0; i<faceA.size()-1; i++) {
            edgesA.push_back({ faceA[i], faceA[i+1] });
        }
        // if we want them in ring => faceA.back() to faceA.front() if needed, but usually 2 points
        if (faceA.size() > 2) {
            edgesA.push_back({ faceA.back(), faceA.front() });
        }
    }

    // Step B: build edges from faceB
    std::vector<std::pair<Vector,Vector>> edgesB;
    if (faceB.size() == 1) {
        edgesB.push_back({faceB[0], faceB[0]});
    }
    else {
        for (size_t i=0; i<faceB.size()-1; i++) {
            edgesB.push_back({ faceB[i], faceB[i+1] });
        }
        if (faceB.size() > 2) {
            edgesB.push_back({ faceB.back(), faceB.front() });
        }
    }

    double bestDist = 1e30;
    Vector bestA(0,0), bestB(0,0);

    // Helper function: closest point from segment [p1..p2] to a point p
    auto closestPointOnSegment = [](const Vector &p1,
                                    const Vector &p2,
                                    const Vector &p) {
        Vector seg = p2 - p1;
        double segLen2 = seg.dotProduct(seg);
        if (segLen2 < 1e-12) {
            return p1; // degenerate
        }
        double t = (p - p1).dotProduct(seg) / segLen2;
        t = std::max(0.0, std::min(1.0, t));
        return p1 + seg*(t);
    };

    // We'll brute force all edges of A vs. all edges of B, computing the closest pair
    for (auto &ea : edgesA) {
        Vector A1 = ea.first;
        Vector A2 = ea.second;

        for (auto &eb : edgesB) {
            Vector B1 = eb.first;
            Vector B2 = eb.second;

            // We'll do a standard "closest distance between two line segments" approach.
            // approach: 
            //   - parametric approach or do iterative approach
            //   - for now let's do an easy iteration: pick each end, project to other seg, etc.

            // We'll pick a few candidate pairs:
            //   1) closest from A1 to segment B1..B2
            Vector b_closestA1 = closestPointOnSegment(B1, B2, A1);
            double dA1 = (A1 - b_closestA1).dotProduct(A1 - b_closestA1);

            //   2) closest from A2 to segment B1..B2
            Vector b_closestA2 = closestPointOnSegment(B1, B2, A2);
            double dA2 = (A2 - b_closestA2).dotProduct(A2 - b_closestA2);

            //   3) closest from B1 to segment A1..A2
            Vector a_closestB1 = closestPointOnSegment(A1, A2, B1);
            double dB1 = (B1 - a_closestB1).dotProduct(B1 - a_closestB1);

            //   4) closest from B2 to segment A1..A2
            Vector a_closestB2 = closestPointOnSegment(A1, A2, B2);
            double dB2 = (B2 - a_closestB2).dotProduct(B2 - a_closestB2);

            // pick the minimal among these 4
            if (dA1 < bestDist) {
                bestDist = dA1;
                bestA = A1;
                bestB = b_closestA1;
            }
            if (dA2 < bestDist) {
                bestDist = dA2;
                bestA = A2;
                bestB = b_closestA2;
            }
            if (dB1 < bestDist) {
                bestDist = dB1;
                bestA = a_closestB1;
                bestB = B1;
            }
            if (dB2 < bestDist) {
                bestDist = dB2;
                bestA = a_closestB2;
                bestB = B2;
            }
        }
    }

    // bestA, bestB are now the closest pair between supporting faces
    contactA = bestA;  
    contactB = bestB;  
    // Some might do further logic to clamp or shift them based on the known penetration,
    // but this is already a big improvement over naive "pos +/- halfPen."
}

/**
 * @brief Extract shape + transformation from an entity to pass into GJK/EPA.
 */
ShapeData extractShapeData(entt::registry &reg, entt::entity e) {
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
 * -------------------------------------------------------------------------
 * Everything below is the same bounding-box-based broad-phase and collision
 * system as before, except we replaced how we compute the final contact point
 * in narrowPhase(...) from EPA results.
 * -------------------------------------------------------------------------
 */

/**
 * @struct AABBEntity
 * @brief Stores an entity and its bounding box in world coordinates.
 */
struct AABBEntity {
    entt::entity entity;
    double minx, miny, maxx, maxy;
};

/**
 * @brief Check if two bounding boxes overlap.
 */
bool boxesOverlap(const AABBEntity &a, const AABBEntity &b) {
    // No overlap if one is strictly left/right or above/below
    if (a.maxx < b.minx || a.minx > b.maxx) return false;
    if (a.maxy < b.miny || a.miny > b.maxy) return false;
    return true;
}

/**
 * @struct BoxNode
 * @brief Quadtree node storing bounding boxes for performance in broad-phase.
 */
struct BoxNode {
    double x, y, size;
    int capacity;
    bool is_leaf;
    std::vector<AABBEntity> objects;
    std::unique_ptr<BoxNode> nw, ne, sw, se;

    BoxNode(double _x, double _y, double _size, int _capacity)
        : x(_x), y(_y), size(_size), capacity(_capacity), is_leaf(true) {}

    bool nodeContains(const AABBEntity &bb) const {
        // fully inside [x..x+size, y..y+size] ?
        return (bb.minx >= x && bb.maxx < (x + size) &&
                bb.miny >= y && bb.maxy < (y + size));
    }

    bool nodeOverlaps(const AABBEntity &bb) const {
        // partial overlap check
        if (bb.maxx < x || bb.minx > (x + size)) return false;
        if (bb.maxy < y || bb.miny > (y + size)) return false;
        return true;
    }

    void subdivide() {
        double half = size / 2.0;
        nw = std::make_unique<BoxNode>(x,        y,        half, capacity);
        ne = std::make_unique<BoxNode>(x+half,   y,        half, capacity);
        sw = std::make_unique<BoxNode>(x,        y+half,   half, capacity);
        se = std::make_unique<BoxNode>(x+half,   y+half,   half, capacity);
        is_leaf = false;
    }

    void insert(const AABBEntity &bb) {
        // skip if no overlap
        if (!nodeOverlaps(bb)) {
            return;
        }
        if (is_leaf && (int)objects.size() < capacity) {
            objects.push_back(bb);
            return;
        }
        if (is_leaf) {
            subdivide();
            // push down existing
            std::vector<AABBEntity> old = std::move(objects);
            objects.clear();
            for (auto &o : old) {
                if (nodeContains(o)) {
                    if (nw->nodeContains(o)) nw->insert(o);
                    else if (ne->nodeContains(o)) ne->insert(o);
                    else if (sw->nodeContains(o)) sw->insert(o);
                    else if (se->nodeContains(o)) se->insert(o);
                    else {
                        objects.push_back(o);
                    }
                } else {
                    objects.push_back(o);
                }
            }
        }

        if (nodeContains(bb)) {
            if (nw->nodeContains(bb)) nw->insert(bb);
            else if (ne->nodeContains(bb)) ne->insert(bb);
            else if (sw->nodeContains(bb)) sw->insert(bb);
            else if (se->nodeContains(bb)) se->insert(bb);
            else {
                objects.push_back(bb);
            }
        } else {
            objects.push_back(bb);
        }
    }

    void query(double qminx, double qminy, double qmaxx, double qmaxy,
               std::vector<AABBEntity> &found) const
    {
        // skip if no overlap
        if (qmaxx < x || qminx > (x + size)) return;
        if (qmaxy < y || qminy > (y + size)) return;

        // local
        for (auto &o : objects) {
            if (o.maxx < qminx || o.minx > qmaxx) continue;
            if (o.maxy < qminy || o.miny > qmaxy) continue;
            found.push_back(o);
        }
        if (!is_leaf) {
            nw->query(qminx, qminy, qmaxx, qmaxy, found);
            ne->query(qminx, qminy, qmaxx, qmaxy, found);
            sw->query(qminx, qminy, qmaxx, qmaxy, found);
            se->query(qminx, qminy, qmaxx, qmaxy, found);
        }
    }
};

/**
 * @brief Compute bounding box for an entity.
 */
static void computeAABB(entt::registry &reg, entt::entity e,
                        double &minx, double &miny,
                        double &maxx, double &maxy)
{
    const auto &pos = reg.get<Components::Position>(e);
    double angle = 0.0;
    if (reg.any_of<Components::AngularPosition>(e)) {
        angle = reg.get<Components::AngularPosition>(e).angle;
    }

    if (reg.any_of<CircleShape>(e)) {
        double r = reg.get<CircleShape>(e).radius;
        minx = pos.x - r;
        maxx = pos.x + r;
        miny = pos.y - r;
        maxy = pos.y + r;
        return;
    }

    const auto &poly = reg.get<PolygonShape>(e);
    minx = pos.x; 
    maxx = pos.x;
    miny = pos.y;
    maxy = pos.y;

    for (auto &v : poly.vertices) {
        double rx = (v.x * std::cos(angle)) - (v.y * std::sin(angle));
        double ry = (v.x * std::sin(angle)) + (v.y * std::cos(angle));
        double wx = pos.x + rx;
        double wy = pos.y + ry;

        if (wx < minx) minx = wx;
        if (wx > maxx) maxx = wx;
        if (wy < miny) miny = wy;
        if (wy > maxy) maxy = wy;
    }
}

std::unique_ptr<BoxNode> buildQuadtree(entt::registry &registry) {
    double size = SimulatorConstants::UniverseSizeMeters;
    double extra = 500.0;
    auto root = std::make_unique<BoxNode>(-extra, -extra, size + 2*extra, 8);

    // Insert bounding boxes
    auto view = registry.view<Components::Position, Components::Mass, Components::ParticlePhase>();
    for (auto e : view) {
        double minx, miny, maxx, maxy;
        computeAABB(registry, e, minx, miny, maxx, maxy);
        AABBEntity box{ e, minx, miny, maxx, maxy };
        root->insert(box);
    }
    return root;
}

void broadPhase(entt::registry &registry, std::vector<CandidatePair> &pairs) {
    pairs.clear();
    auto root = buildQuadtree(registry);

    std::vector<AABBEntity> found;
    found.reserve(32);

    auto view = registry.view<Components::Position, Components::Mass, Components::ParticlePhase>();
    for (auto e : view) {
        double minx, miny, maxx, maxy;
        computeAABB(registry, e, minx, miny, maxx, maxy);
        AABBEntity queryBB { e, minx, miny, maxx, maxy };

        found.clear();
        root->query(minx, miny, maxx, maxy, found);

        for (auto &f : found) {
            if (f.entity != e && f.entity > e) {
                if (boxesOverlap(queryBB, f)) {
                    pairs.push_back({ e, f.entity });
                }
            }
        }
    }
}

/**
 * @brief GJK/EPA narrow-phase with improved contact point calculation.
 */
void narrowPhase(entt::registry &registry,
                 const std::vector<CandidatePair> &pairs,
                 CollisionManifold &manifold)
{
    manifold.clear();
    for (auto &cp : pairs) {
        if (!registry.valid(cp.eA) || !registry.valid(cp.eB)) continue;

        auto sdA = extractShapeData(registry, cp.eA);
        auto sdB = extractShapeData(registry, cp.eB);

        Simplex simplex;
        if (GJKIntersect(sdA, sdB, simplex)) {
            auto epaRes = EPA(sdA, sdB, simplex);
            if (epaRes.has_value()) {
                EPAResult res = epaRes.value();
                // Instead of naive "pos +/- halfPen", we find the supporting faces in +/- res.normal
                Vector cA, cB;
                findAccurateContact(sdA, sdB, res.normal, res.penetration, cA, cB);

                // Our final contact is midpoint of those support points
                Vector contact = (cA + cB) * 0.5;

                CollisionInfo info;
                info.a = cp.eA;
                info.b = cp.eB;
                info.normal = res.normal;
                info.penetration = res.penetration;
                info.contactPoint = contact;
                manifold.collisions.push_back(info);
            }
        }
    }
}

void solidCollisionResponse(entt::registry &registry, CollisionManifold &manifold) {
    const double baumgarte   = 0.5;
    const double slop        = 0.001;
    const double angularDamp = 0.98;

    for (auto &col : manifold.collisions) {
        if (!registry.valid(col.a) || !registry.valid(col.b)) continue;

        bool asleepA = false, asleepB = false;
        if (registry.any_of<Components::Sleep>(col.a)) {
            asleepA = registry.get<Components::Sleep>(col.a).asleep;
        }
        if (registry.any_of<Components::Sleep>(col.b)) {
            asleepB = registry.get<Components::Sleep>(col.b).asleep;
        }
        if (asleepA && asleepB) continue;

        auto &phaseA = registry.get<Components::ParticlePhase>(col.a);
        auto &phaseB = registry.get<Components::ParticlePhase>(col.b);
        if (phaseA.phase != Components::Phase::Solid && phaseB.phase != Components::Phase::Solid) {
            continue;
        }

        auto posA  = registry.get<Components::Position>(col.a);
        auto velA  = registry.get<Components::Velocity>(col.a);
        auto &massA = registry.get<Components::Mass>(col.a);

        auto posB  = registry.get<Components::Position>(col.b);
        auto velB  = registry.get<Components::Velocity>(col.b);
        auto &massB = registry.get<Components::Mass>(col.b);

        double invMassA = (massA.value > 1e29) ? 0.0 : 1.0/massA.value;
        double invMassB = (massB.value > 1e29) ? 0.0 : 1.0/massB.value;
        double invSum   = invMassA + invMassB;
        if (invSum < 1e-12) continue;

        Vector n = col.normal;
        double penetration = col.penetration;

        Vector relVel = velB - velA;
        double normalSpeed = relVel.dotProduct(n);

        double j = 0.0;
        if (normalSpeed < 0) {
            j = -(0.5 + SimulatorConstants::CollisionCoeffRestitution)* normalSpeed / invSum;
        }
        Vector impulse = n * j;

        // Apply velocity changes
        velA = velA - (impulse * invMassA);
        velB = velB + (impulse * invMassB);

        // Positional correction
        double corr = std::max(penetration - slop, 0.0) * baumgarte / invSum;
        posA.x -= n.x * (corr * invMassA);
        posA.y -= n.y * (corr * invMassA);
        posB.x += n.x * (corr * invMassB);
        posB.y += n.y * (corr * invMassB);

        // Friction
        double staticFrictionA  = 5.9, dynamicFrictionA  = 5.3;
        double staticFrictionB  = 5.9, dynamicFrictionB  = 5.3;
        if (registry.any_of<Components::Material>(col.a)) {
            auto &mA = registry.get<Components::Material>(col.a);
            staticFrictionA  = mA.staticFriction;
            dynamicFrictionA = mA.dynamicFriction;
        }
        if (registry.any_of<Components::Material>(col.b)) {
            auto &mB = registry.get<Components::Material>(col.b);
            staticFrictionB  = mB.staticFriction;
            dynamicFrictionB = mB.dynamicFriction;
        }
        double combinedStaticFriction  = 0.5*(staticFrictionA + staticFrictionB);
        double combinedDynamicFriction = 0.5*(dynamicFrictionA + dynamicFrictionB);

        Vector t = relVel - (n * relVel.dotProduct(n));
        double tLen = t.length();
        if (tLen > EPSILON) {
            t = t / tLen;
        } else {
            t = Vector(0,0);
        }

        double tangentialSpeed = relVel.dotProduct(t);
        double jt = -tangentialSpeed / invSum;
        double frictionImpulseMag = std::fabs(jt);

        // Coulomb friction
        if (frictionImpulseMag < j * combinedStaticFriction) {
            jt = -tangentialSpeed / invSum;
        } else {
            jt = -j * combinedDynamicFriction * ((jt>0)?1:-1);
        }
        Vector frictionImpulse = t * jt;
        velA = velA - (frictionImpulse * invMassA);
        velB = velB + (frictionImpulse * invMassB);

        // Angular impulses
        if (registry.all_of<Components::AngularVelocity, Components::Inertia>(col.a) &&
            registry.all_of<Components::AngularVelocity, Components::Inertia>(col.b)) {

            auto &angA = registry.get<Components::AngularVelocity>(col.a);
            auto &I_A  = registry.get<Components::Inertia>(col.a);
            auto &angB = registry.get<Components::AngularVelocity>(col.b);
            auto &I_B  = registry.get<Components::Inertia>(col.b);

            Vector pA(posA.x, posA.y);
            Vector pB(posB.x, posB.y);
            Vector c(col.contactPoint.x, col.contactPoint.y);

            Vector rA = c - pA;
            Vector rB = c - pB;

            double angImpA = rA.cross(impulse);
            double angImpB = -rB.cross(impulse);
            angA.omega += angImpA / I_A.I;
            angB.omega += angImpB / I_B.I;

            double fAngA = rA.cross(frictionImpulse);
            double fAngB = -rB.cross(frictionImpulse);
            angA.omega += fAngA / I_A.I;
            angB.omega += fAngB / I_B.I;

            // Dampen spin
            angA.omega *= angularDamp;
            angB.omega *= angularDamp;

            registry.replace<Components::AngularVelocity>(col.a, angA);
            registry.replace<Components::AngularVelocity>(col.b, angB);
        }

        // Write back
        registry.replace<Components::Position>(col.a, posA);
        registry.replace<Components::Position>(col.b, posB);
        registry.replace<Components::Velocity>(col.a, velA);
        registry.replace<Components::Velocity>(col.b, velB);
    }
}

/**
 * @brief Additional solver passes to reduce persistent penetration.
 */
void positionalSolver(entt::registry &registry,
                      CollisionManifold &manifold,
                      int iterations,
                      double baumgarte,
                      double slop)
{
    for (int i = 0; i < iterations; ++i) {
        for (auto &col : manifold.collisions) {
            if (!registry.valid(col.a) || !registry.valid(col.b)) continue;
            if (!registry.all_of<Components::Position, Components::Mass, Components::ParticlePhase>(col.a)) continue;
            if (!registry.all_of<Components::Position, Components::Mass, Components::ParticlePhase>(col.b)) continue;

            auto &massA = registry.get<Components::Mass>(col.a);
            auto &massB = registry.get<Components::Mass>(col.b);
            auto posA   = registry.get<Components::Position>(col.a);
            auto posB   = registry.get<Components::Position>(col.b);

            auto &phaseA = registry.get<Components::ParticlePhase>(col.a);
            auto &phaseB = registry.get<Components::ParticlePhase>(col.b);

            if (phaseA.phase != Components::Phase::Solid && phaseB.phase != Components::Phase::Solid) {
                continue;
            }

            double invMassA = (massA.value > 1e29) ? 0.0 : ((massA.value > 1e-12) ? 1.0/massA.value : 0.0);
            double invMassB = (massB.value > 1e29) ? 0.0 : ((massB.value > 1e-12) ? 1.0/massB.value : 0.0);
            double invSum = invMassA + invMassB;
            if (invSum < 1e-12) continue;

            double pen = col.penetration;
            if (pen <= slop) continue;

            double corr = (pen - slop)* baumgarte / invSum;
            Vector n = col.normal;

            double dxA = n.x * corr * invMassA;
            double dyA = n.y * corr * invMassA;
            double dxB = n.x * corr * invMassB;
            double dyB = n.y * corr * invMassB;

            posA.x -= dxA;
            posA.y -= dyA;
            posB.x += dxB;
            posB.y += dyB;

            registry.replace<Components::Position>(col.a, posA);
            registry.replace<Components::Position>(col.b, posB);
        }
    }
}

} // end anonymous namespace


namespace Systems {

/**
 * @class RigidBodyCollisionSystem
 * @brief Single system orchestrating:
 *        (1) bounding-box quadtree broad-phase,
 *        (2) GJK/EPA narrow-phase,
 *        (3) improved contact point for collision response,
 *        (4) optional positional solver.
 */
void RigidBodyCollisionSystem::update(entt::registry &registry,
                                      int solverIterations,
                                      int positionalSolverIterations,
                                      double baumgarte,
                                      double slop)
{
    std::vector<CandidatePair> candidatePairs;
    candidatePairs.reserve(128);

    CollisionManifold manifold;

    // We re-run broadPhase + narrowPhase each iteration so that large position shifts
    // can be re-detected. 
    for (int i = 0; i < solverIterations; ++i) {
        candidatePairs.clear();
        broadPhase(registry, candidatePairs);

        narrowPhase(registry, candidatePairs, manifold);

        if (manifold.collisions.empty()) {
            break;
        }
        solidCollisionResponse(registry, manifold);
    }

    if (positionalSolverIterations > 0) {
        positionalSolver(registry, manifold, positionalSolverIterations, baumgarte, slop);
    }
}

} // namespace Systems