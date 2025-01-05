#include "nbody/systems/rigid_body_collision.hpp"

/**
 * @file rigid_body_collision.cpp
 * @brief Implementation of a single-system approach to broad-phase, narrow-phase,
 *        and rigid body collision responses (including optional positional solver).
 */

#include <memory>
#include <algorithm>
#include <cmath>
#include <vector>

#include <entt/entt.hpp>

#include "nbody/core/constants.hpp"              // For UniverseSizeMeters, etc.
#include "nbody/algo/gjk.hpp"                    // GJK algorithm
#include "nbody/algo/epa.hpp"                    // EPA algorithm
#include "nbody/components/basic.hpp"            // Position, Velocity, Mass, etc.
#include "nbody/math/vector_math.hpp"            // Vector operations
#include "nbody/math/polygon.hpp"                // For polygon shape
#include "nbody/systems/collision/collision_data.hpp"

namespace {

/**
 * @struct ParticleInfo
 * @brief Internal helper struct for broad-phase quadtree insertion.
 */
struct ParticleInfo {
    entt::entity entity;
    double x, y;
};

/**
 * @struct QuadNode
 * @brief Node of a quadtree for broad-phase collision candidate generation.
 */
struct QuadNode {
    double x, y, size;
    int capacity;
    bool is_leaf;
    std::vector<ParticleInfo> particles;
    std::unique_ptr<QuadNode> nw, ne, sw, se;

    QuadNode(double _x, double _y, double _size, int _capacity)
        : x(_x), y(_y), size(_size), capacity(_capacity), is_leaf(true) {}

    bool contains(double px, double py) const {
        return (px >= x && px < (x + size) && py >= y && py < (y + size));
    }

    void subdivide() {
        double half = size / 2.0;
        nw = std::make_unique<QuadNode>(x,       y,       half, capacity);
        ne = std::make_unique<QuadNode>(x+half,  y,       half, capacity);
        sw = std::make_unique<QuadNode>(x,       y+half,  half, capacity);
        se = std::make_unique<QuadNode>(x+half,  y+half,  half, capacity);
        is_leaf = false;
    }

    void insert(const ParticleInfo &p) {
        if (!contains(p.x, p.y)) {
            return;
        }
        if (is_leaf && (int)particles.size() < capacity) {
            particles.push_back(p);
            return;
        }
        if (is_leaf) {
            // subdivide and redistribute
            subdivide();
            for (auto &part : particles) {
                insertIntoChild(part);
            }
            particles.clear();
        }
        insertIntoChild(p);
    }

    void insertIntoChild(const ParticleInfo &p) {
        if (nw->contains(p.x,p.y))      nw->insert(p);
        else if (ne->contains(p.x,p.y)) ne->insert(p);
        else if (sw->contains(p.x,p.y)) sw->insert(p);
        else if (se->contains(p.x,p.y)) se->insert(p);
    }

    bool overlaps(double qx, double qy, double qsize) const {
        bool outside = (qx > x+size) || (qy > y+size) || (qx+qsize < x) || (qy+qsize < y);
        return !outside;
    }

    void query(double qx, double qy, double qsize, std::vector<ParticleInfo> &found) const {
        if (!overlaps(qx, qy, qsize)) {
            return;
        }
        if (is_leaf) {
            for (auto &part : particles) {
                if (part.x >= qx && part.x <= qx+qsize &&
                    part.y >= qy && part.y <= qy+qsize) {
                    found.push_back(part);
                }
            }
            return;
        }
        nw->query(qx, qy, qsize, found);
        ne->query(qx, qy, qsize, found);
        sw->query(qx, qy, qsize, found);
        se->query(qx, qy, qsize, found);
    }
};

/**
 * @brief Builds a quadtree covering the entire simulation area (with a margin).
 */
std::unique_ptr<QuadNode> buildQuadtree(entt::registry &registry) {
    double size = SimulatorConstants::UniverseSizeMeters;
    double extra = 500.0; // margin to include walls or out-of-bounds objects

    auto root = std::make_unique<QuadNode>(-extra, -extra, size + 2*extra, 8);

    auto view = registry.view<Components::Position>();
    for (auto e : view) {
        const auto &pos = view.get<Components::Position>(e);
        ParticleInfo pinfo { e, pos.x, pos.y };
        root->insert(pinfo);
    }
    return root;
}

/**
 * @brief Helper to generate candidate pairs via the quadtree.
 */
void broadPhase(entt::registry &registry, std::vector<CandidatePair> &candidatePairs) {
    candidatePairs.clear();
    auto root = buildQuadtree(registry);

    auto view = registry.view<Components::Position, Components::Mass, Components::ParticlePhase>();
    for (auto eA : view) {
        const auto &posA = view.get<Components::Position>(eA);
        double query_size = 1.0;
        double qx = posA.x - query_size * 0.5;
        double qy = posA.y - query_size * 0.5;

        std::vector<ParticleInfo> candidates;
        candidates.reserve(16); // Arbitrary small reserve to reduce reallocs
        root->query(qx, qy, query_size, candidates);

        // Collect only unique pairs (order eA < eB).
        for (auto &c : candidates) {
            if (c.entity != eA && c.entity > eA) {
                candidatePairs.push_back({ eA, c.entity });
            }
        }
    }
}

/**
 * @brief Extracts the shape (circle or polygon) and transformation from an entity.
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
 * @brief Narrow-phase step using GJK/EPA for each candidate pair.
 */
void narrowPhase(entt::registry &registry,
                 const std::vector<CandidatePair> &candidatePairs,
                 CollisionManifold &manifold)
{
    manifold.clear();

    for (auto &pair : candidatePairs) {
        if (!registry.valid(pair.eA) || !registry.valid(pair.eB)) {
            continue;
        }

        // Extract shape data
        auto sdA = extractShapeData(registry, pair.eA);
        auto sdB = extractShapeData(registry, pair.eB);

        // Run GJK
        Simplex simplex;
        if (GJKIntersect(sdA, sdB, simplex)) {
            // If collision, refine with EPA
            auto epaRes = EPA(sdA, sdB, simplex);
            if (epaRes.has_value()) {
                EPAResult res = epaRes.value();

                // Approximate the contact point
                Vector cA(sdA.pos.x, sdA.pos.y);
                Vector cB(sdB.pos.x, sdB.pos.y);
                Vector halfPen = res.normal * (res.penetration * 0.5);
                cA = cA - halfPen;
                cB = cB + halfPen;
                Vector contact = (cA + cB) * 0.5;

                // Store in manifold
                CollisionInfo info;
                info.a = pair.eA;
                info.b = pair.eB;
                info.normal = res.normal;
                info.penetration = res.penetration;
                info.contactPoint = contact;
                manifold.collisions.push_back(info);
            }
        }
    }
}

/**
 * @brief Solves collisions for solids (rigid bodies), updating velocities and positions.
 */
void solidCollisionResponse(entt::registry &registry, CollisionManifold &manifold) {
    const double baumgarte = 0.5;
    const double slop = 0.001;
    const double angularDamp = 0.98;

    for (auto &col : manifold.collisions) {
        if (!registry.valid(col.a) || !registry.valid(col.b)) {
            continue;
        }
        // Check if both are asleep
        bool asleepA = false;
        bool asleepB = false;
        if (registry.any_of<Components::Sleep>(col.a)) {
            auto &sleepA = registry.get<Components::Sleep>(col.a);
            asleepA = sleepA.asleep;
        }
        if (registry.any_of<Components::Sleep>(col.b)) {
            auto &sleepB = registry.get<Components::Sleep>(col.b);
            asleepB = sleepB.asleep;
        }
        if (asleepA && asleepB) {
            continue;
        }

        // Only proceed if at least one is a solid
        auto &phaseA = registry.get<Components::ParticlePhase>(col.a);
        auto &phaseB = registry.get<Components::ParticlePhase>(col.b);
        if (phaseA.phase != Components::Phase::Solid && phaseB.phase != Components::Phase::Solid) {
            continue;
        }

        // Retrieve components
        auto posA  = registry.get<Components::Position>(col.a);
        auto velA  = registry.get<Components::Velocity>(col.a);
        auto &massA = registry.get<Components::Mass>(col.a);

        auto posB  = registry.get<Components::Position>(col.b);
        auto velB  = registry.get<Components::Velocity>(col.b);
        auto &massB = registry.get<Components::Mass>(col.b);

        // Handle infinite mass
        double invMassA = (massA.value > 1e29) ? 0.0 : 1.0 / massA.value;
        double invMassB = (massB.value > 1e29) ? 0.0 : 1.0 / massB.value;
        double invSum   = invMassA + invMassB;
        if (invSum < 1e-12) {
            continue;
        }

        Vector n = col.normal;
        double penetration = col.penetration;

        Vector relativeVel = velB - velA;
        double normalSpeed = relativeVel.dotProduct(n);

        // Simple restitution model
        double restitution = 0.0;
        if (std::fabs(normalSpeed) < 0.5) {
            restitution *= 0.5; // smaller bounce if speed is small
        }

        // Impulse along normal
        double j = 0.0;
        if (normalSpeed < 0) {
            j = -(1.0 + restitution) * normalSpeed / invSum;
        }
        Vector impulse = n * j;

        // Apply impulses
        velA = velA - (impulse * invMassA);
        velB = velB + (impulse * invMassB);

        // Positional correction (Baumgarte)
        double corr = std::max(penetration - slop, 0.0) * baumgarte / invSum;
        Position corrA(n.x * (corr * invMassA), n.y * (corr * invMassA));
        Position corrB(n.x * (corr * invMassB), n.y * (corr * invMassB));
        posA.x -= corrA.x;
        posA.y -= corrA.y;
        posB.x += corrB.x;
        posB.y += corrB.y;

        // Friction
        double staticFrictionA = 0.5, dynamicFrictionA = 0.3;
        double staticFrictionB = 0.5, dynamicFrictionB = 0.3;

        if (registry.any_of<Components::Material>(col.a)) {
            auto &matA = registry.get<Components::Material>(col.a);
            staticFrictionA = matA.staticFriction;
            dynamicFrictionA = matA.dynamicFriction;
        }
        if (registry.any_of<Components::Material>(col.b)) {
            auto &matB = registry.get<Components::Material>(col.b);
            staticFrictionB = matB.staticFriction;
            dynamicFrictionB = matB.dynamicFriction;
        }
        double combinedStaticFriction  = (staticFrictionA + staticFrictionB) * 0.5;
        double combinedDynamicFriction = (dynamicFrictionA + dynamicFrictionB) * 0.5;

        Vector t = relativeVel - (n * (relativeVel.dotProduct(n)));
        double tLen = t.length();
        if (tLen > EPSILON) {
            t = t / tLen;
        } else {
            t = Vector(0, 0);
        }

        double tangentSpeed = relativeVel.dotProduct(t);
        double jt = -tangentSpeed / invSum;
        double frictionImpulseMagnitude = std::fabs(jt);

        // Coulomb's friction model
        if (frictionImpulseMagnitude < j * combinedStaticFriction) {
            // Within static friction limit
            jt = -tangentSpeed / invSum;
        } else {
            // Use dynamic friction
            jt = -j * combinedDynamicFriction * ((jt > 0) ? 1 : -1);
        }
        Vector frictionImpulse = t * jt;
        velA = velA - (frictionImpulse * invMassA);
        velB = velB + (frictionImpulse * invMassB);

        // Angular impulses (if both have angular velocity and inertia)
        if (registry.all_of<Components::AngularVelocity, Components::Inertia>(col.a) &&
            registry.all_of<Components::AngularVelocity, Components::Inertia>(col.b)) {

            auto &angVelA = registry.get<Components::AngularVelocity>(col.a);
            auto &I_A     = registry.get<Components::Inertia>(col.a);
            auto &angVelB = registry.get<Components::AngularVelocity>(col.b);
            auto &I_B     = registry.get<Components::Inertia>(col.b);

            Vector pA(posA.x, posA.y);
            Vector pB(posB.x, posB.y);
            Vector c(col.contactPoint.x, col.contactPoint.y);

            // Lever arms
            Vector rA = c - pA;
            Vector rB = c - pB;

            // Angular impulse from normal impulse
            double angularImpulseA = rA.cross(impulse);
            double angularImpulseB = -rB.cross(impulse);

            angVelA.omega += angularImpulseA / I_A.I;
            angVelB.omega += angularImpulseB / I_B.I;

            // Angular impulse from friction
            double fAngA = rA.cross(frictionImpulse);
            double fAngB = -rB.cross(frictionImpulse);

            angVelA.omega += fAngA / I_A.I;
            angVelB.omega += fAngB / I_B.I;

            // Dampen angular velocities
            angVelA.omega *= angularDamp;
            angVelB.omega *= angularDamp;

            registry.replace<Components::AngularVelocity>(col.a, angVelA);
            registry.replace<Components::AngularVelocity>(col.b, angVelB);
        }

        // Write back updated components
        registry.replace<Components::Position>(col.a, posA);
        registry.replace<Components::Position>(col.b, posB);
        registry.replace<Components::Velocity>(col.a, velA);
        registry.replace<Components::Velocity>(col.b, velB);
    }
}

/**
 * @brief Additional positional correction passes to reduce jitter and penetration.
 */
void positionalSolver(entt::registry &registry,
                      CollisionManifold &manifold,
                      int iterations,
                      double baumgarte,
                      double slop)
{
    for (int iter = 0; iter < iterations; ++iter) {
        for (auto &col : manifold.collisions) {
            if (!registry.valid(col.a) || !registry.valid(col.b)) {
                continue;
            }
            if (!registry.all_of<Components::Position, Components::Mass, Components::ParticlePhase>(col.a)) {
                continue;
            }
            if (!registry.all_of<Components::Position, Components::Mass, Components::ParticlePhase>(col.b)) {
                continue;
            }
            auto posA  = registry.get<Components::Position>(col.a);
            auto &massA = registry.get<Components::Mass>(col.a);

            auto posB  = registry.get<Components::Position>(col.b);
            auto &massB = registry.get<Components::Mass>(col.b);

            auto &phaseA = registry.get<Components::ParticlePhase>(col.a);
            auto &phaseB = registry.get<Components::ParticlePhase>(col.b);

            // Only correct solids
            if (phaseA.phase != Components::Phase::Solid && phaseB.phase != Components::Phase::Solid) {
                continue;
            }

            double invMassA = (massA.value > 1e29) ? 0.0 : ((massA.value > 1e-12) ? 1.0/massA.value : 0.0);
            double invMassB = (massB.value > 1e29) ? 0.0 : ((massB.value > 1e-12) ? 1.0/massB.value : 0.0);

            double invSum = invMassA + invMassB;
            if (invSum < 1e-12) {
                continue;
            }

            double penetration = col.penetration;
            if (penetration <= slop) {
                continue;
            }

            double corr = (penetration - slop) * baumgarte / invSum;
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

void RigidBodyCollisionSystem::update(entt::registry &registry,
                                      int solverIterations,
                                      int positionalSolverIterations,
                                      double baumgarte,
                                      double slop)
{
    // 1) Broad-phase: gather candidate pairs
    std::vector<CandidatePair> candidatePairs;
    candidatePairs.reserve(128); // arbitrary reserve to reduce vector resizing
    broadPhase(registry, candidatePairs);

    // 2) Narrow-phase and manifold
    CollisionManifold manifold;

    // 3) Contact solver loop
    for (int i = 0; i < solverIterations; ++i) {
        // Re-run narrow-phase each iteration because positions can shift
        narrowPhase(registry, candidatePairs, manifold);

        // If no collisions, break early
        if (manifold.collisions.empty()) {
            break;
        }

        // Solve collisions for solid bodies
        solidCollisionResponse(registry, manifold);
    }

    // 4) Additional positional solver passes (optional)
    if (positionalSolverIterations > 0) {
        positionalSolver(registry, manifold, positionalSolverIterations, baumgarte, slop);
    }
}

} // namespace System