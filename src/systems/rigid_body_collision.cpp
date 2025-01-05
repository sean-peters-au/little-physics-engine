/**
 * @file rigid_body_collision.cpp
 * @brief Rigid-body collision system supporting arbitrary polygons. The broad-phase
 *        now stores bounding boxes (AABBs) instead of single points, allowing timely
 *        detection of collisions even for large or offset shapes.
 *
 *        Performance-wise, this solution uses a simple quadtree for bounding boxes,
 *        reducing the risk of “late collision detection” inherent in point-based
 *        approaches. Polygons, circles, or any convex shapes are simply inserted
 *        via an axis-aligned bounding box. During queries, we check overlapping
 *        bounding boxes to gather candidate pairs.
 *
 *        After collecting candidate pairs, we proceed with GJK/EPA (narrow phase)
 *        and finally a collision response system that applies impulses and
 *        positional corrections.
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
 * @struct AABB
 * @brief Stores an entity and its bounding box in world coordinates.
 */
struct AABBEntity {
    entt::entity entity;
    double minx, miny, maxx, maxy;
};

/**
 * @brief Helper to check if two bounding boxes overlap.
 */
bool boxesOverlap(const AABBEntity &a, const AABBEntity &b) {
    // No overlap if one box is to the left/right or above/below the other
    if (a.maxx < b.minx || a.minx > b.maxx) return false;
    if (a.maxy < b.miny || a.miny > b.maxy) return false;
    return true;
}

/**
 * @struct BoxNode
 * @brief Quadtree node storing bounding boxes for a subset of entities.
 *
 * Each node covers [x, y, x+size, y+size]. We subdivide if capacity is exceeded
 * and the box can fit entirely in a child region. If not, or if partial coverage,
 * we store it in this node.
 */
struct BoxNode {
    double x, y, size;   ///< Node bounding region: [x, y] .. [x+size, y+size]
    int capacity;
    bool is_leaf;
    std::vector<AABBEntity> objects; ///< Bboxes stored at this node
    std::unique_ptr<BoxNode> nw, ne, sw, se;

    BoxNode(double _x, double _y, double _size, int _capacity)
        : x(_x), y(_y), size(_size), capacity(_capacity), is_leaf(true) {}

    /**
     * @brief Node bounding box overlap check vs. an AABBEntity
     */
    bool nodeContains(const AABBEntity &bb) const {
        // Check if the entire bounding box (bb) fits fully inside this node region
        if (bb.minx >= x && bb.maxx < (x + size) &&
            bb.miny >= y && bb.maxy < (y + size)) {
            return true;
        }
        return false;
    }

    bool nodeOverlaps(const AABBEntity &bb) const {
        // Box for this node is [x, y, x+size, y+size]
        // We check partial overlap
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

    /**
     * @brief Insert an AABB. If it fully fits in a child, push it down; otherwise store here.
     */
    void insert(const AABBEntity &bb) {
        // If bounding box doesn't overlap node at all, skip
        if (!nodeOverlaps(bb)) {
            return;
        }
        if (is_leaf && (int)objects.size() < capacity) {
            objects.push_back(bb);
            return;
        }
        if (is_leaf) {
            // Subdivide and redistribute
            subdivide();
            // Move our existing objects into children if they fully fit
            std::vector<AABBEntity> old = std::move(objects);
            objects.clear();
            for (auto &o : old) {
                if (nodeContains(o)) {
                    // see if it fully fits in a child
                    if (nw->nodeContains(o)) nw->insert(o);
                    else if (ne->nodeContains(o)) ne->insert(o);
                    else if (sw->nodeContains(o)) sw->insert(o);
                    else if (se->nodeContains(o)) se->insert(o);
                    else {
                        // partial coverage => store here
                        objects.push_back(o);
                    }
                } else {
                    // partial coverage => store here
                    objects.push_back(o);
                }
            }
        }
        // Now try to insert the new one
        if (nodeContains(bb)) {
            // see if it fully fits in a child
            if (nw->nodeContains(bb)) nw->insert(bb);
            else if (ne->nodeContains(bb)) ne->insert(bb);
            else if (sw->nodeContains(bb)) sw->insert(bb);
            else if (se->nodeContains(bb)) se->insert(bb);
            else {
                // partial coverage => store here
                objects.push_back(bb);
            }
        } else {
            // partial coverage => store here
            objects.push_back(bb);
        }
    }

    /**
     * @brief Query all objects in this node that overlap the query bounding box (qminx etc.).
     */
    void query(double qminx, double qminy, double qmaxx, double qmaxy,
               std::vector<AABBEntity> &found) const
    {
        // If node bounding box doesn't overlap query region, skip
        if (qmaxx < x || qminx > (x + size)) return;
        if (qmaxy < y || qminy > (y + size)) return;

        // Check local objects
        for (auto &o : objects) {
            // Overlap test between query region & object bounding box
            if (o.maxx < qminx || o.minx > qmaxx) continue;
            if (o.maxy < qminy || o.miny > qmaxy) continue;
            found.push_back(o);
        }

        // If leaf, done
        if (is_leaf) return;

        // Else descend to children
        nw->query(qminx, qminy, qmaxx, qmaxy, found);
        ne->query(qminx, qminy, qmaxx, qmaxy, found);
        sw->query(qminx, qminy, qmaxx, qmaxy, found);
        se->query(qminx, qminy, qmaxx, qmaxy, found);
    }
};


/**
 * @brief Compute the axis-aligned bounding box (AABB) for a single entity (polygon or circle).
 */
void computeAABB(entt::registry &reg, entt::entity e,
                 double &minx, double &miny, double &maxx, double &maxy)
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

    // Arbitrary polygon => transform each vertex
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

/**
 * @brief Build a quadtree for the entire simulation area and insert bounding boxes for each entity.
 */
std::unique_ptr<BoxNode> buildQuadtree(entt::registry &registry) {
    double size = SimulatorConstants::UniverseSizeMeters;
    double extra = 500.0; // margin
    auto root = std::make_unique<BoxNode>(-extra, -extra, size + 2*extra, 8);

    // For all (Position, Mass, ParticlePhase) entities, compute bounding box and insert
    auto view = registry.view<Components::Position, Components::Mass, Components::ParticlePhase>();
    for (auto e : view) {
        double minx, miny, maxx, maxy;
        computeAABB(registry, e, minx, miny, maxx, maxy);

        AABBEntity box{ e, minx, miny, maxx, maxy };
        // Insert bounding box into quadtree
        root->insert(box);
    }
    return root;
}

/**
 * @brief Broad-phase that returns candidate pairs by overlapping bounding boxes.
 */
void broadPhase(entt::registry &registry, std::vector<CandidatePair> &pairs) {
    pairs.clear();

    // Build or rebuild quadtree
    auto root = buildQuadtree(registry);

    // For each inserted bounding box, we query the quadtree for potential overlaps
    // in its bounding region. Then we test for actual box-vs-box overlap.
    // We only push pairs (eA < eB).
    std::vector<AABBEntity> found;
    found.reserve(32);

    auto view = registry.view<Components::Position, Components::Mass, Components::ParticlePhase>();
    for (auto e : view) {
        double minx, miny, maxx, maxy;
        computeAABB(registry, e, minx, miny, maxx, maxy);

        AABBEntity queryBB { e, minx, miny, maxx, maxy };

        // Query the node for all AABB that overlap [minx..maxx, miny..maxy]
        found.clear();
        root->query(minx, miny, maxx, maxy, found);

        // For each found, do an actual box overlap test
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
 * @brief Extract the shape data for GJK/EPA.
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
 * @brief Narrow-phase: for each candidate pair, run GJK/EPA. If collision found,
 *        add to manifold with normal, penetration, contact point.
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

                // Approx. contact point
                Vector cA(sdA.pos.x, sdA.pos.y);
                Vector cB(sdB.pos.x, sdB.pos.y);
                Vector halfPen = res.normal * (res.penetration * 0.5);
                cA = cA - halfPen;
                cB = cB + halfPen;
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


/**
 * @brief Collision response for rigid bodies (solid).
 */
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

        // Simple restitution
        double restitution = 0.0;
        if (std::fabs(normalSpeed) < 0.5) {
            restitution *= 0.5;
        }

        double j = 0.0;
        if (normalSpeed < 0) {
            j = -(1.0 + restitution)* normalSpeed / invSum;
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

        // Write back position + velocity
        registry.replace<Components::Position>(col.a, posA);
        registry.replace<Components::Position>(col.b, posB);
        registry.replace<Components::Velocity>(col.a, velA);
        registry.replace<Components::Velocity>(col.b, velB);
    }
}

/**
 * @brief Additional positional solver to reduce lingering penetration.
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

            double invMassA = (massA.value > 1e29)? 0.0 : ((massA.value>1e-12)? 1.0/massA.value:0.0);
            double invMassB = (massB.value > 1e29)? 0.0 : ((massB.value>1e-12)? 1.0/massB.value:0.0);
            double invSum = invMassA + invMassB;
            if (invSum<1e-12) continue;

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
 *        (1) broad-phase bounding-box quadtree,
 *        (2) narrow-phase GJK/EPA,
 *        (3) solid collision response,
 *        (4) optional positional solver.
 */
void RigidBodyCollisionSystem::update(entt::registry &registry,
                                      int solverIterations,
                                      int positionalSolverIterations,
                                      double baumgarte,
                                      double slop)
{
    // We'll build a new quadtree & run broad-phase each iteration so that
    // large position shifts can still be caught in subsequent passes.
    std::vector<CandidatePair> candidatePairs;
    candidatePairs.reserve(128);

    CollisionManifold manifold;

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