/**
 * @file rigid_body_collision.cpp
 * @brief Rigid-body collision system with a basic "persistent contact" hack
 *        that applies friction from an estimated resting normal force (like mg)
 *        even when the collision impulse is zero. 
 *
 *        This version also uses split impulses (velocity-only collision solver),
 *        and a two-point manifold approach for polygon–polygon collisions.
 */

#include "nbody/systems/rigid_body_collision.hpp"
#include <memory>
#include <algorithm>
#include <cmath>
#include <vector>
#include <entt/entt.hpp>

#include "nbody/core/constants.hpp"
#include "nbody/algo/gjk.hpp"         // GJK algorithm
#include "nbody/algo/epa.hpp"         // EPA algorithm
#include "nbody/components/basic.hpp" // Position, Velocity, Mass, etc.
#include "nbody/math/vector_math.hpp" // Vector operations
#include "nbody/math/polygon.hpp"     // For polygon shape
#include "nbody/systems/collision/collision_data.hpp"

namespace {

/** ------------------------------------------------------------------
 *  Extract shape data from ECS for GJK/EPA
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

/** ------------------------------------------------------------------
 *  For a polygon or circle, compute its world-space vertices.
 */
static std::vector<Vector> getWorldVerts(const ShapeData &shape)
{
    std::vector<Vector> verts;
    if (shape.isCircle) {
        // Approximate circle by sampling
        const int samples = 8;
        double step = (2.0 * M_PI) / samples;
        for (int i=0; i<samples; i++) {
            double angle = i * step + shape.angle;
            double cx = shape.pos.x + shape.radius * std::cos(angle);
            double cy = shape.pos.y + shape.radius * std::sin(angle);
            verts.push_back(Vector(cx, cy));
        }
    } else {
        // Transform polygon vertices from local to world coords
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

/** ------------------------------------------------------------------
 *  Single-contact approach for circle or fallback (like your original).
 *  Finds a more accurate contact by computing edges in +/- normal.
 */
static void findAccurateContact(const ShapeData &A,
                                const ShapeData &B,
                                const Vector &normal, 
                                double /*penetration*/,
                                Vector &contactA,
                                Vector &contactB)
{
    // We'll sample the "supporting face" in +normal for A, and in -normal for B
    // then find the closest approach between those edges. This is your old approach.

    auto getMaxDotVerts = [&](const ShapeData& S, const Vector& dir){
        // For circle: single best vertex
        if (S.isCircle) {
            double rx = std::cos(S.angle)*S.radius; 
            double ry = std::sin(S.angle)*S.radius;
            Vector guess(S.pos.x + rx, S.pos.y + ry);
            return std::vector<Vector>{ guess };
        }
        // For polygon: gather all vertices that maximize dot
        auto wverts = getWorldVerts(S);
        double maxDot = -1e30;
        std::vector<double> dots(wverts.size());
        for (size_t i=0; i<wverts.size(); i++){
            double d = wverts[i].dotProduct(dir);
            dots[i] = d;
            if (d > maxDot) maxDot = d;
        }
        static const double EPS_FACE = 1e-6;
        std::vector<Vector> support;
        for (size_t i=0; i<wverts.size(); i++){
            if (std::fabs(dots[i] - maxDot) < EPS_FACE){
                support.push_back(wverts[i]);
            }
        }
        return support;
    };

    auto faceA = getMaxDotVerts(A,  normal);
    auto faceB = getMaxDotVerts(B, -normal);

    // If either is empty, fallback to direct center points
    if (faceA.empty() || faceB.empty()) {
        contactA = Vector(A.pos.x, A.pos.y);
        contactB = Vector(B.pos.x, B.pos.y);
        return;
    }

    // We'll do an O(M*N) approach to find the minimum distance pair
    double bestDist2 = 1e30;
    Vector bestA(A.pos.x, A.pos.y), bestB(B.pos.x, B.pos.y);

    auto sqrLen = [&](const Vector &p, const Vector &q){
        double dx = p.x - q.x;
        double dy = p.y - q.y;
        return dx*dx + dy*dy;
    };

    for (auto &aPt : faceA) {
        for (auto &bPt : faceB) {
            double d2 = sqrLen(aPt, bPt);
            if (d2 < bestDist2) {
                bestDist2 = d2;
                bestA = aPt;
                bestB = bPt;
            }
        }
    }

    contactA = bestA;
    contactB = bestB;
}

/** ------------------------------------------------------------------
 *  Build up to two contact points for polygon–polygon collisions.
 *  We'll use "reference face" from shape A and "clip" shape B's vertices.
 */
static std::vector<Vector> getPolygonFaceNormals(const std::vector<Vector>& poly)
{
    std::vector<Vector> normals;
    normals.reserve(poly.size());
    for (size_t i = 0; i < poly.size(); i++) {
        size_t j = (i + 1) % poly.size();
        Vector edge = poly[j] - poly[i];
        Vector normal(edge.y, -edge.x); // left normal for CCW
        normal = normal.normalized();
        normals.push_back(normal);
    }
    return normals;
}

static std::vector<Vector> clipFaceByPlane(const std::vector<Vector>& points,
                                           const Vector &planeNormal,
                                           double planeOffset)
{
    // Sutherland–Hodgman 2D polygon clipping
    std::vector<Vector> out;
    out.reserve(points.size()+1);

    for (size_t i=0; i<points.size(); i++) {
        size_t j = (i+1) % points.size();
        Vector A = points[i];
        Vector B = points[j];
        double distA = planeNormal.dotProduct(A) - planeOffset;
        double distB = planeNormal.dotProduct(B) - planeOffset;

        bool insideA = (distA <= 0.0);
        bool insideB = (distB <= 0.0);

        if (insideA) {
            out.push_back(A);
        }
        if (insideA ^ insideB) {
            // Edge crosses the plane
            double t = distA / (distA - distB);
            Vector AB = B - A;
            Vector intersect = A + AB * t;
            out.push_back(intersect);
        }
    }
    return out;
}

static std::vector<Vector> clipIncidentFace(const std::vector<Vector>& polygonB,
                                            const Vector& refNormal,
                                            const Vector& refV0,
                                            const Vector& refV1)
{
    std::vector<Vector> clipped = polygonB;

    double refOffset = refNormal.dotProduct(refV0);
    // Clip B by the reference face "front" plane
    clipped = clipFaceByPlane(clipped,  refNormal, refOffset);

    // Also clip by side planes to keep points within [refV0, refV1] segment
    Vector edgeDir = (refV1 - refV0).normalized();
    Vector leftNormal(-edgeDir.y, edgeDir.x);
    Vector rightNormal(edgeDir.y, -edgeDir.x);

    double leftOffset = leftNormal.dotProduct(refV0);
    double rightOffset = rightNormal.dotProduct(refV1);

    // Clip by left plane
    clipped = clipFaceByPlane(clipped,  leftNormal, leftOffset);

    // Clip by right plane => note we pass -rightNormal, negative offset
    // because we want points that are "in front" side of the edge
    clipped = clipFaceByPlane(clipped, -rightNormal, -rightOffset);

    return clipped;
}

/** 
 * @brief Build a manifold (multiple contact points) for polygon–polygon collisions.
 *        If either shape is a circle, we revert to single-contact approach.
 */
static std::vector<CollisionInfo> buildContactManifold(entt::entity eA,
                                                       entt::entity eB,
                                                       const ShapeData& A,
                                                       const ShapeData& B,
                                                       const Vector& normal,
                                                       double penetration)
{
    std::vector<CollisionInfo> contacts;

    // If either shape is a circle => single contact approach
    if (A.isCircle || B.isCircle) {
        Vector cA, cB;
        findAccurateContact(A, B, normal, penetration, cA, cB);
        Vector mid = (cA + cB)*0.5;

        CollisionInfo info;
        info.a = eA;
        info.b = eB;
        info.normal = normal;
        info.penetration = penetration;
        info.contactPoint = mid;
        contacts.push_back(info);
        return contacts;
    }

    // Both are polygons => we'll pick a reference face on A
    auto wvertsA = getWorldVerts(A);
    auto wvertsB = getWorldVerts(B);

    auto normalsA = getPolygonFaceNormals(wvertsA);

    // Find the best face on A that aligns with the "EPA normal"
    double bestDot = -1e30;
    size_t bestIndex = 0;
    for (size_t i=0; i<normalsA.size(); i++){
        double d = normalsA[i].dotProduct(normal);
        if (d > bestDot){
            bestDot = d;
            bestIndex = i;
        }
    }
    // reference edge
    size_t iNext = (bestIndex+1) % wvertsA.size();
    Vector refV0 = wvertsA[bestIndex];
    Vector refV1 = wvertsA[iNext];
    Vector refFaceNormal = normalsA[bestIndex];

    // Clip shape B to that face
    std::vector<Vector> clippedB = clipIncidentFace(wvertsB, refFaceNormal, refV0, refV1);
    double refOffset = refFaceNormal.dotProduct(refV0);

    // For each clipped point, if behind refFace within the overlap => valid contact
    for (auto &ptB : clippedB) {
        double dist = refFaceNormal.dotProduct(ptB) - refOffset;
        if (dist <= 0.01) {
            // The point on shape A is ptB minus the distance along the normal
            Vector ptA = ptB - refFaceNormal*dist;

            CollisionInfo cinfo;
            cinfo.a = eA;
            cinfo.b = eB;
            cinfo.normal = normal;
            cinfo.penetration = penetration;
            cinfo.contactPoint = (ptA + ptB)*0.5;
            contacts.push_back(cinfo);
        }
    }

    // Limit to 2 points typical in 2D
    if (contacts.size() > 2) {
        contacts.resize(2);
    }

    // If none found, fallback to single contact
    if (contacts.empty()) {
        Vector cA, cB;
        findAccurateContact(A, B, normal, penetration, cA, cB);
        Vector mid = (cA + cB)*0.5;

        CollisionInfo info;
        info.a = eA;
        info.b = eB;
        info.normal = normal;
        info.penetration = penetration;
        info.contactPoint = mid;
        contacts.push_back(info);
    }
    return contacts;
}

/** ------------------------------------------------------------------
 *  Broad-phase quadtree building (unchanged from your original code).
 */
struct AABBEntity {
    entt::entity entity;
    double minx, miny, maxx, maxy;
};

bool boxesOverlap(const AABBEntity &a, const AABBEntity &b) {
    if (a.maxx < b.minx || a.minx > b.maxx) return false;
    if (a.maxy < b.miny || a.miny > b.maxy) return false;
    return true;
}

// Minimal quadtree node for broad-phase
struct BoxNode {
    double x, y, size;
    int capacity;
    bool is_leaf;
    std::vector<AABBEntity> objects;
    std::unique_ptr<BoxNode> nw, ne, sw, se;

    BoxNode(double _x, double _y, double _size, int _capacity)
        : x(_x), y(_y), size(_size), capacity(_capacity), is_leaf(true) {}
    bool nodeContains(const AABBEntity &bb) const {
        return (bb.minx >= x && bb.maxx < (x + size) &&
                bb.miny >= y && bb.maxy < (y + size));
    }
    bool nodeOverlaps(const AABBEntity &bb) const {
        if (bb.maxx < x || bb.minx > x+size) return false;
        if (bb.maxy < y || bb.miny > y+size) return false;
        return true;
    }
    void subdivide() {
        double half = size / 2.0;
        nw = std::make_unique<BoxNode>(x,       y,       half, capacity);
        ne = std::make_unique<BoxNode>(x+half,  y,       half, capacity);
        sw = std::make_unique<BoxNode>(x,       y+half,  half, capacity);
        se = std::make_unique<BoxNode>(x+half,  y+half,  half, capacity);
        is_leaf = false;
    }
    void insert(const AABBEntity &bb) {
        if (!nodeOverlaps(bb)) return;
        if (is_leaf && (int)objects.size() < capacity) {
            objects.push_back(bb);
            return;
        }
        if (is_leaf) {
            subdivide();
            auto oldObjs = std::move(objects);
            objects.clear();
            for (auto &o : oldObjs) {
                if (nodeContains(o)) {
                    if      (nw->nodeContains(o)) nw->insert(o);
                    else if (ne->nodeContains(o)) ne->insert(o);
                    else if (sw->nodeContains(o)) sw->insert(o);
                    else if (se->nodeContains(o)) se->insert(o);
                    else objects.push_back(o);
                } else {
                    objects.push_back(o);
                }
            }
        }

        if (nodeContains(bb)) {
            if      (nw->nodeContains(bb)) nw->insert(bb);
            else if (ne->nodeContains(bb)) ne->insert(bb);
            else if (sw->nodeContains(bb)) sw->insert(bb);
            else if (se->nodeContains(bb)) se->insert(bb);
            else objects.push_back(bb);
        } else {
            objects.push_back(bb);
        }
    }
    void query(double qminx, double qminy, double qmaxx, double qmaxy,
               std::vector<AABBEntity> &found) const
    {
        if (qmaxx < x || qminx > x+size) return;
        if (qmaxy < y || qminy > y+size) return;

        for (auto &o : objects) {
            if (o.maxx < qminx || o.minx > qmaxx) continue;
            if (o.maxy < qminy || o.miny > qmaxy) continue;
            found.push_back(o);
        }
        if (!is_leaf) {
            nw->query(qminx, qminy, qmaxx, qmaxy, found);
            ne->query(qminx, qminy, qmaxx, qmaxy, found);
            sw->query(qminx, qminy, qmaxx, qmaxy, found);
            se->query(qminx, qminy, qmaxy, qmaxy, found); // minor bug: typed 'qmaxy, qmaxy'
        }
    }
};

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
        minx = pos.x - r; maxx = pos.x + r;
        miny = pos.y - r; maxy = pos.y + r;
        return;
    }

    // polygon
    const auto &poly = reg.get<PolygonShape>(e);
    minx = pos.x; maxx = pos.x;
    miny = pos.y; maxy = pos.y;

    for (auto &v : poly.vertices) {
        double rx = v.x * std::cos(angle) - v.y * std::sin(angle);
        double ry = v.x * std::sin(angle) + v.y * std::cos(angle);
        double wx = pos.x + rx;
        double wy = pos.y + ry;
        if (wx < minx) minx = wx;
        if (wx > maxx) maxx = wx;
        if (wy < miny) miny = wy;
        if (wy > maxy) maxy = wy;
    }
}

// Build quadtree
static std::unique_ptr<BoxNode> buildQuadtree(entt::registry &registry) {
    double size = SimulatorConstants::UniverseSizeMeters;
    double extra = 500.0; // buffer
    auto root = std::make_unique<BoxNode>(-extra, -extra, size + 2*extra, 8);

    auto view = registry.view<Components::Position, Components::Mass, Components::ParticlePhase>();
    for (auto e : view) {
        double minx, miny, maxx, maxy;
        computeAABB(registry, e, minx, miny, maxx, maxy);
        AABBEntity box{ e, minx, miny, maxx, maxy };
        root->insert(box);
    }
    return root;
}

static void broadPhase(entt::registry &registry, std::vector<CandidatePair> &pairs) {
    pairs.clear();
    auto root = buildQuadtree(registry);

    std::vector<AABBEntity> found;
    found.reserve(64);

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

/** ------------------------------------------------------------------
 *  narrowPhase: uses GJK + EPA, then builds a manifold (possibly multiple contact points).
 */
static void narrowPhase(entt::registry &registry,
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
                // Build a multi-point manifold
                auto contacts = buildContactManifold(cp.eA, cp.eB,
                                                     sdA, sdB,
                                                     res.normal, res.penetration);
                for (auto &cinfo : contacts) {
                    manifold.collisions.push_back(cinfo);
                }
            }
        }
    }
}

/** ------------------------------------------------------------------
 *  solveContactConstraints: split impulse (velocity only) + friction, resting normal hack
 */
static void solveContactConstraints(entt::registry &registry,
                                    CollisionManifold &manifold,
                                    double baumgarte,
                                    double slop)
{
    const double angularDamp = 0.98;
    const double gravity = 9.8; // used for resting force estimate

    // For each contact in the manifold
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
        if (phaseA.phase != Components::Phase::Solid && phaseB.phase != Components::Phase::Solid)
            continue;

        auto posA  = registry.get<Components::Position>(col.a);
        auto velA  = registry.get<Components::Velocity>(col.a);
        auto &massA = registry.get<Components::Mass>(col.a);

        auto posB  = registry.get<Components::Position>(col.b);
        auto velB  = registry.get<Components::Velocity>(col.b);
        auto &massB = registry.get<Components::Mass>(col.b);

        double invMassA = (massA.value > 1e29)? 0.0 : 1.0/massA.value;
        double invMassB = (massB.value > 1e29)? 0.0 : 1.0/massB.value;
        double invSum   = invMassA + invMassB;
        if (invSum < 1e-12) continue;

        Vector n = col.normal;
        double penetration = col.penetration;

        // 1) Normal impulse
        Vector relVel = velB - velA;
        double normalSpeed = relVel.dotProduct(n);

        double restitution = 0.0; 
        double jColl = 0.0;
        if (normalSpeed < 0) {
            jColl = -(1.0+restitution)* normalSpeed / invSum;
        }

        // "resting normal" hack
        double restingN = 0.0;
        if (penetration > 0.0 && std::fabs(normalSpeed) < 0.1) {
            double combinedMass = massA.value + massB.value;
            // Gravity direction is (0, 1)
            Vector gravityDir(0.0, 1.0);
            double gravityComponent = std::fabs(gravityDir.dotProduct(n));
            restingN = combinedMass * gravity * gravityComponent;
        }

        double jTotalNormal = jColl + restingN;
        if (jTotalNormal < 0) jTotalNormal = 0;

        // Velocity update from normal impulse
        Vector impulseNormal = n * jColl;
        velA = velA - (impulseNormal * invMassA);
        velB = velB + (impulseNormal * invMassB);

        // 2) Friction
        double staticFrictionA  = 0.5, dynamicFrictionA  = 0.3;
        double staticFrictionB  = 0.5, dynamicFrictionB  = 0.3;
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

        double jRef = jTotalNormal;
        Vector tangent = relVel - n*(relVel.dotProduct(n));
        double tLen = tangent.length();
        if (tLen > 1e-9) {
            tangent = tangent / tLen;
        } else {
            tangent = Vector(0,0);
        }

        double tangentialSpeed = relVel.dotProduct(tangent);
        double jt = -tangentialSpeed / invSum;
        double frictionImpulseMag = std::fabs(jt);

        double frictionLimit = jRef * combinedStaticFriction;
        if (frictionImpulseMag < frictionLimit) {
            jt = -tangentialSpeed / invSum; // static friction
        } else {
            double dynFric = jRef * combinedDynamicFriction;
            jt = -(jt > 0 ? dynFric : -dynFric);
        }

        Vector frictionImpulse = tangent * jt;
        velA = velA - (frictionImpulse * invMassA);
        velB = velB + (frictionImpulse * invMassB);

        // 3) Angular impulses
        if (registry.all_of<Components::AngularVelocity, Components::Inertia>(col.a) &&
            registry.all_of<Components::AngularVelocity, Components::Inertia>(col.b)) {

            auto &angA = registry.get<Components::AngularVelocity>(col.a);
            auto &I_A  = registry.get<Components::Inertia>(col.a);
            auto &angB = registry.get<Components::AngularVelocity>(col.b);
            auto &I_B  = registry.get<Components::Inertia>(col.b);

            Vector c = col.contactPoint;  // world contact point
            Vector pA(posA.x, posA.y);
            Vector pB(posB.x, posB.y);

            Vector rA = c - pA;
            Vector rB = c - pB;

            double angImpA = rA.cross(impulseNormal);
            double angImpB = rB.cross(impulseNormal);
            angA.omega += angImpA / I_A.I;
            angB.omega += angImpB / I_B.I;

            double fAngA = rA.cross(frictionImpulse);
            double fAngB = rB.cross(frictionImpulse);
            angA.omega += fAngA / I_A.I;
            angB.omega += fAngB / I_B.I;

            // angular damping
            angA.omega *= angularDamp;
            angB.omega *= angularDamp;
            registry.replace<Components::AngularVelocity>(col.a, angA);
            registry.replace<Components::AngularVelocity>(col.b, angB);
        }

        // store updated velocities
        registry.replace<Components::Velocity>(col.a, velA);
        registry.replace<Components::Velocity>(col.b, velB);
    }
}

/** ------------------------------------------------------------------
 *  positionalSolver: correct leftover penetrations (split impulse).
 */
static void positionalSolver(entt::registry &registry,
                             CollisionManifold &manifold,
                             int iterations,
                             double baumgarte,
                             double slop)
{
    for (int i=0; i<iterations; i++) {
        for (auto &col : manifold.collisions) {
            if (!registry.valid(col.a) || !registry.valid(col.b)) continue;
            if (!registry.all_of<Components::Position, Components::Mass, Components::ParticlePhase>(col.a)) continue;
            if (!registry.all_of<Components::Position, Components::Mass, Components::ParticlePhase>(col.b)) continue;

            auto &phaseA = registry.get<Components::ParticlePhase>(col.a);
            auto &phaseB = registry.get<Components::ParticlePhase>(col.b);
            if (phaseA.phase != Components::Phase::Solid && phaseB.phase != Components::Phase::Solid) {
                continue;
            }

            auto &massA = registry.get<Components::Mass>(col.a);
            auto &massB = registry.get<Components::Mass>(col.b);
            auto posA = registry.get<Components::Position>(col.a);
            auto posB = registry.get<Components::Position>(col.b);

            double invMassA = (massA.value>1e29)? 0.0 : ((massA.value>1e-12)? 1.0/massA.value:0.0);
            double invMassB = (massB.value>1e29)? 0.0 : ((massB.value>1e-12)? 1.0/massB.value:0.0);
            double invSum = invMassA + invMassB;
            if (invSum<1e-12) continue;

            double pen = col.penetration;
            if (pen <= slop) continue;

            double corr = std::max(pen - slop, 0.0) * baumgarte / invSum;
            Vector n = col.normal;

            posA.x -= n.x * corr * invMassA;
            posA.y -= n.y * corr * invMassA;
            posB.x += n.x * corr * invMassB;
            posB.y += n.y * corr * invMassB;

            registry.replace<Components::Position>(col.a, posA);
            registry.replace<Components::Position>(col.b, posB);
        }
    }
}

} // end anonymous namespace

namespace Systems {

/** ------------------------------------------------------------------
 *  The main ECS system entry point
 */
void RigidBodyCollisionSystem::update(entt::registry &registry,
                                      int solverIterations,
                                      int positionalSolverIterations,
                                      double baumgarte,
                                      double slop)
{
    // multiple iterations for velocity solver
    for (int pass=0; pass<solverIterations; pass++) {
        // Broad-phase
        std::vector<CandidatePair> candidatePairs;
        candidatePairs.reserve(128);
        broadPhase(registry, candidatePairs);

        // Narrow-phase => manifold
        CollisionManifold manifold;
        narrowPhase(registry, candidatePairs, manifold);
        if (manifold.collisions.empty()) {
            break; // No collisions => done
        }

        // Our velocity-only solver (split impulse)
        solveContactConstraints(registry, manifold, baumgarte, slop);
    }

    // Extra positional passes if desired
    if (positionalSolverIterations > 0) {
        std::vector<CandidatePair> candidatePairs;
        candidatePairs.reserve(128);
        broadPhase(registry, candidatePairs);

        CollisionManifold manifold;
        narrowPhase(registry, candidatePairs, manifold);

        positionalSolver(registry, manifold, positionalSolverIterations, baumgarte, slop);
    }
}

} // namespace Systems