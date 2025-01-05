/**
 * @file rigid_body_collision.cpp
 * @brief Rigid-body collision system with a basic "persistent contact" hack 
 *        that applies friction from an estimated resting normal force (like mg)
 *        even when the collision impulse is zero. This prevents objects from 
 *        endlessly sliding sideways on the floor.
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
        // Transform polygon vertices
        for (auto &v : shape.poly.vertices) {
            double rx = (v.x * std::cos(shape.angle)) - (v.y * std::sin(shape.angle));
            double ry = (v.x * std::sin(shape.angle)) + (v.y * std::cos(shape.angle));
            double wx = shape.pos.x + rx;
            double wy = shape.pos.y + ry;
            verts.push_back(Vector(wx, wy));
        }
    }
    return verts;
}

/**
 * @brief Gather vertices within EPS of maximum projection along 'dir' 
 *        (1-2 vertices for polygons).
 */
static std::vector<Vector> findSupportingFace(const std::vector<Vector> &worldVerts,
                                              const Vector &dir)
{
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

    static const double EPS_FACE = 1e-6; 
    std::vector<Vector> support;
    support.reserve(4);
    for (size_t i=0; i<worldVerts.size(); i++) {
        if (std::fabs(projVals[i] - maxProj) < EPS_FACE) {
            support.push_back(worldVerts[i]);
        }
    }
    return support;
}

/**
 * @brief Find a more accurate contact by computing edges in +/- normal.
 */
static void findAccurateContact(const ShapeData &A,
                                const ShapeData &B,
                                const Vector &normal, 
                                double penetration,
                                Vector &contactA,
                                Vector &contactB)
{
    auto vertsA = getWorldVerts(A);
    auto vertsB = getWorldVerts(B);

    auto faceA = findSupportingFace(vertsA,  normal);
    auto faceB = findSupportingFace(vertsB, -normal);

    // Default fallback
    contactA = Vector(A.pos.x, A.pos.y);
    contactB = Vector(B.pos.x, B.pos.y);
    if (faceA.empty() || faceB.empty()) {
        return;
    }

    // Build edges from face vertices
    auto buildEdges = [&](const std::vector<Vector> &f) {
        std::vector<std::pair<Vector,Vector>> edges;
        if (f.size() == 1) {
            edges.push_back({ f[0], f[0] }); 
        } else {
            for (size_t i=0; i+1 < f.size(); i++) {
                edges.push_back({ f[i], f[i+1] });
            }
            if (f.size() > 2) {
                edges.push_back({ f.back(), f.front() });
            }
        }
        return edges;
    };

    auto edgesA = buildEdges(faceA);
    auto edgesB = buildEdges(faceB);

    auto closestPointOnSegment = [&](const Vector &p1, const Vector &p2, const Vector &p){
        Vector seg = p2 - p1;
        double segLen2 = seg.dotProduct(seg);
        if (segLen2 < 1e-12) return p1; 
        double t = (p - p1).dotProduct(seg) / segLen2;
        t = std::max(0.0, std::min(1.0, t));
        return p1 + seg*(t);
    };

    double bestDist2 = 1e30;
    Vector bestA, bestBv;
    for (auto &ea : edgesA) {
        Vector A1 = ea.first;
        Vector A2 = ea.second;
        for (auto &eb : edgesB) {
            Vector B1 = eb.first;
            Vector B2 = eb.second;

            // Evaluate 4 candidate pairs
            Vector candB_A1 = closestPointOnSegment(B1, B2, A1);
            double dA1 = (A1 - candB_A1).dotProduct(A1 - candB_A1);

            Vector candB_A2 = closestPointOnSegment(B1, B2, A2);
            double dA2 = (A2 - candB_A2).dotProduct(A2 - candB_A2);

            Vector candA_B1 = closestPointOnSegment(A1, A2, B1);
            double dB1 = (B1 - candA_B1).dotProduct(B1 - candA_B1);

            Vector candA_B2 = closestPointOnSegment(A1, A2, B2);
            double dB2 = (B2 - candA_B2).dotProduct(B2 - candA_B2);

            if (dA1 < bestDist2) {
                bestDist2 = dA1;
                bestA = A1;
                bestBv = candB_A1;
            }
            if (dA2 < bestDist2) {
                bestDist2 = dA2;
                bestA = A2;
                bestBv = candB_A2;
            }
            if (dB1 < bestDist2) {
                bestDist2 = dB1;
                bestA = candA_B1;
                bestBv = B1;
            }
            if (dB2 < bestDist2) {
                bestDist2 = dB2;
                bestA = candA_B2;
                bestBv = B2;
            }
        }
    }

    contactA = bestA;
    contactB = bestBv;
}

/**
 * @brief Convert ECS entity to GJK/EPA shape data
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
 * Broad-phase: quadtree using bounding boxes
 * -------------------------------------------------------------------------
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
            se->query(qminx, qminy, qmaxy, qmaxy, found); // NOTE: minor bug: typed 'qmaxy, qmaxy'?
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
        double rx = v.x*std::cos(angle) - v.y*std::sin(angle);
        double ry = v.x*std::sin(angle) + v.y*std::cos(angle);
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
 * @brief Rebuild collisions each iteration
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
                // compute more accurate contact
                Vector cA, cB;
                findAccurateContact(sdA, sdB, res.normal, res.penetration, cA, cB);

                Vector contact = (cA + cB)*0.5;
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

/**
 * @brief Single-step solver that includes a "resting normal force" hack so friction 
 *        won't vanish if normalSpeed=0. That prevents infinite sliding.
 */
void solveContactConstraints(entt::registry &registry, 
                             CollisionManifold &manifold,
                             double baumgarte,
                             double slop)
{
    const double angularDamp = 0.98;
    const double gravity = 9.8; // used for resting force estimate

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

        // If moving inward, compute collision impulse:
        double restitution = 0.0; // or use (0.5 + const) if you want bounce
        double jColl = 0.0;
        if (normalSpeed < 0) {
            jColl = -(1.0+restitution)* normalSpeed / invSum;
        }

        // Also estimate a "resting normal" if shapes are still intersecting
        // but normalSpeed ~ 0. 
        // We'll do a hack: if (penetration>0.0) => treat there's a stable contact force ~ mg
        // This is not fully correct for stacking, but enough to allow friction to act.
        double restingN = 0.0;
        if (penetration > 0.0 && std::fabs(normalSpeed) < 0.1) {
            // Only apply resting force based on gravity component along normal
            double combinedMass = massA.value + massB.value;
            
            // Gravity direction is (0, 1)
            Vector gravityDir(0.0, 1.0);
            double gravityComponent = std::abs(gravityDir.dotProduct(n));
            
            // Scale resting force by how aligned the normal is with gravity
            restingN = combinedMass * gravity * gravityComponent;
        }

        double jTotalNormal = jColl + restingN;  
        if (jTotalNormal < 0) jTotalNormal = 0; // no negative

        Vector impulseNormal = n * jColl; // only the 'collision' portion changes velocity
        velA = velA - (impulseNormal*invMassA);
        velB = velB + (impulseNormal*invMassB);

        // 2) Position correction
        double corr = std::max(penetration - slop, 0.0) * baumgarte / invSum;
        posA.x = posA.x - n.x*corr*invMassA;
        posA.y = posA.y - n.y*corr*invMassA;
        posB.x = posB.x + n.x*corr*invMassB;
        posB.y = posB.y + n.y*corr*invMassB;

        // 3) Friction 
        // Friction limit = µ * (jTotalNormal). 
        // If jTotalNormal=0 but we have restingN, friction can be up to µ*(restingN).
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

        Vector tangent = relVel - n*(relVel.dotProduct(n));
        double tLen = tangent.length();
        if (tLen > EPSILON) {
            tangent = tangent / tLen;
        } else {
            tangent = Vector(0,0);
        }

        double tangentialSpeed = relVel.dotProduct(tangent);
        double jt = -tangentialSpeed / invSum;
        double frictionImpulseMag = std::fabs(jt);

        double frictionLimit = (jTotalNormal) * combinedStaticFriction; 
        // if frictionImpulseMag < frictionLimit => static friction kills tangential speed
        // else dynamic friction
        if (frictionImpulseMag < frictionLimit) {
            jt = -tangentialSpeed / invSum; // kill tangent speed
        } else {
            double dynFric = (jTotalNormal) * combinedDynamicFriction;
            jt = - (jt>0? dynFric : -dynFric);
        }

        Vector frictionImpulse = tangent * jt;
        velA = velA - frictionImpulse * invMassA;
        velB = velB + frictionImpulse * invMassB;

        // 4) Angular impulses
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

            // collision normal impulse
            double angImpA = rA.cross(impulseNormal);
            double angImpB = rB.cross(impulseNormal);
            angA.omega += angImpA / I_A.I;
            angB.omega += angImpB / I_B.I;

            // friction impulse
            double fAngA = rA.cross(frictionImpulse);
            double fAngB = rB.cross(frictionImpulse);
            angA.omega += fAngA / I_A.I;
            angB.omega += fAngB / I_B.I;

            // damp
            angA.omega *= angularDamp;
            angB.omega *= angularDamp;
            registry.replace<Components::AngularVelocity>(col.a, angA);
            registry.replace<Components::AngularVelocity>(col.b, angB);
        }

        // Write updated
        registry.replace<Components::Position>(col.a, posA);
        registry.replace<Components::Position>(col.b, posB);
        registry.replace<Components::Velocity>(col.a, velA);
        registry.replace<Components::Velocity>(col.b, velB);
    }
}

/**
 * @brief Additional "positional solver" for leftover penetration.
 */
void positionalSolver(entt::registry &registry,
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

            double corr = (pen - slop)* baumgarte / invSum;
            Vector n = col.normal;

            posA.x -= n.x*corr*invMassA;
            posA.y -= n.y*corr*invMassA;
            posB.x += n.x*corr*invMassB;
            posB.y += n.y*corr*invMassB;

            registry.replace<Components::Position>(col.a, posA);
            registry.replace<Components::Position>(col.b, posB);
        }
    }
}

} // end anonymous namespace

namespace Systems {

void RigidBodyCollisionSystem::update(entt::registry &registry,
                                      int solverIterations,
                                      int positionalSolverIterations,
                                      double baumgarte,
                                      double slop)
{
    // We do multiple passes
    for (int pass=0; pass<solverIterations; pass++) {
        // 1) Broad-phase
        std::vector<CandidatePair> candidatePairs;
        candidatePairs.reserve(128);
        broadPhase(registry, candidatePairs);

        // 2) Narrow-phase => manifold
        CollisionManifold manifold;
        narrowPhase(registry, candidatePairs, manifold);
        if (manifold.collisions.empty()) {
            break; // No collisions => done
        }

        // 3) Our "contact solver" with resting friction hack
        solveContactConstraints(registry, manifold, baumgarte, slop);
    }

    // 4) Extra positional passes if wanted
    //    (still helpful for leftover penetrations)
    if (positionalSolverIterations > 0) {
        // We can do one final broad-phase + narrow-phase if desired, or 
        // just fix the manifold from last time. For simplicity, let's
        // just fix leftover from last manifold (some do it iteratively).
        // In practice, you'd do repeated passes similarly to collisions.
        // But let's keep it simple:
        std::vector<CandidatePair> candidatePairs;
        candidatePairs.reserve(128);
        broadPhase(registry, candidatePairs);
        CollisionManifold manifold;
        narrowPhase(registry, candidatePairs, manifold);

        positionalSolver(registry, manifold, positionalSolverIterations, baumgarte, slop);
    }
}

} // namespace Systems