#include "nbody/systems/collision.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <memory>

// A Quadtree for broad-phase collision detection.
// Each node covers a square region of space and stores particles if under capacity.
// If it exceeds capacity, it subdivides into four children and distributes particles among them.

namespace Systems {

struct ParticleInfo {
    entt::entity entity;
    double x, y;
    double radius;
};

// Quadtree node
struct QuadNode {
    double x, y;       // top-left corner of the node
    double size;       // length of the node's side
    int capacity;      // max particles before subdivision
    std::vector<ParticleInfo> particles;
    bool is_leaf;

    std::unique_ptr<QuadNode> nw;
    std::unique_ptr<QuadNode> ne;
    std::unique_ptr<QuadNode> sw;
    std::unique_ptr<QuadNode> se;

    QuadNode(double x_, double y_, double size_, int capacity_)
        : x(x_), y(y_), size(size_), capacity(capacity_), is_leaf(true) {}

    bool contains(double px, double py) const {
        return px >= x && px < (x + size) && py >= y && py < (y + size);
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
        // If this node doesn't contain the particle, skip
        if (!contains(p.x, p.y)) return;

        // If leaf and not full, store here
        if (is_leaf && (int)particles.size() < capacity) {
            particles.push_back(p);
            return;
        }

        // If leaf but full, subdivide and redistribute
        if (is_leaf) {
            subdivide();
            // Move existing particles down
            for (auto &part : particles) {
                insertIntoChild(part);
            }
            particles.clear();
        }

        // Insert new particle into a child
        insertIntoChild(p);
    }

    void insertIntoChild(const ParticleInfo &p) {
        if (nw->contains(p.x, p.y)) nw->insert(p);
        else if (ne->contains(p.x, p.y)) ne->insert(p);
        else if (sw->contains(p.x, p.y)) sw->insert(p);
        else if (se->contains(p.x, p.y)) se->insert(p);
        else {
            // Should not happen if boundaries match the universe and insert calls are correct
            // If it does, consider fallback to store in parent node or handle boundaries carefully.
        }
    }

    // Query all particles in this node and descendants that lie within the query region
    void query(double qx, double qy, double qsize, std::vector<ParticleInfo>& found) const {
        // Check for overlap
        if (!overlaps(qx, qy, qsize)) return;

        // If leaf, check these particles
        if (is_leaf) {
            for (auto &part : particles) {
                if (part.x >= qx && part.x <= qx + qsize && 
                    part.y >= qy && part.y <= qy + qsize) {
                    found.push_back(part);
                }
            }
            return;
        }

        // Check children
        nw->query(qx, qy, qsize, found);
        ne->query(qx, qy, qsize, found);
        sw->query(qx, qy, qsize, found);
        se->query(qx, qy, qsize, found);
    }

    bool overlaps(double qx, double qy, double qsize) const {
        bool outside = (qx > x+size) || (qy > y+size) || (qx+qsize < x) || (qy+qsize < y);
        return !outside;
    }
};

// Collision handling functions (unchanged logic from before, but could be improved)
static void handleSolidSolidCollision(
    Components::Position& posA, Components::Velocity& velA, const Components::Mass& massA,
    Components::Position& posB, Components::Velocity& velB, const Components::Mass& massB,
    double dx, double dy, double dist, double min_dist)
{
    double restitution = 0.9;
    double nx = dx / dist;
    double ny = dy / dist;

    double relvx = velB.x - velA.x;
    double relvy = velB.y - velA.y;
    double relative_speed = relvx * nx + relvy * ny;

    if (relative_speed < 0) {
        double invMassA = 1.0 / massA.value;
        double invMassB = 1.0 / massB.value;
        double invMassSum = invMassA + invMassB;

        double impulse = -(1.0 + restitution)*relative_speed / invMassSum;

        velA.x -= (impulse * invMassA) * nx;
        velA.y -= (impulse * invMassA) * ny;
        velB.x += (impulse * invMassB) * nx;
        velB.y += (impulse * invMassB) * ny;

        double penetration = min_dist - dist;
        double correctionFactor = 0.5 * penetration / invMassSum;
        posA.x -= correctionFactor * invMassA * nx;
        posA.y -= correctionFactor * invMassA * ny;
        posB.x += correctionFactor * invMassB * nx;
        posB.y += correctionFactor * invMassB * ny;
    }
}

// Other handlers can remain the same as previously. For brevity, we just show the same structure:
static void handleSolidLiquidCollision(
    Components::Position& posA, Components::Velocity& /*velA*/, const Components::Mass& massA, Components::Phase /*phaseA*/,
    Components::Position& posB, Components::Velocity& /*velB*/, const Components::Mass& /*massB*/, Components::Phase /*phaseB*/,
    double dx, double dy, double dist, double min_dist)
{
    double nx = dx / dist;
    double ny = dy / dist;
    double penetration = min_dist - dist;
    posA.x -= 0.5 * penetration * nx;
    posA.y -= 0.5 * penetration * ny;
    posB.x += 0.5 * penetration * nx;
    posB.y += 0.5 * penetration * ny;
}

static void handleLiquidLiquidCollision(
    Components::Position& posA, Components::Velocity& /*velA*/, const Components::Mass& /*massA*/,
    Components::Position& posB, Components::Velocity& /*velB*/, const Components::Mass& /*massB*/,
    double dx, double dy, double dist, double min_dist)
{
    double nx = dx / dist;
    double ny = dy / dist;
    double penetration = min_dist - dist;
    posA.x -= 0.5 * penetration * nx;
    posA.y -= 0.5 * penetration * ny;
    posB.x += 0.5 * penetration * nx;
    posB.y += 0.5 * penetration * ny;
}

static void handleGasGasCollision(
    Components::Position& posA, Components::Velocity& velA, const Components::Mass& massA,
    Components::Position& posB, Components::Velocity& velB, const Components::Mass& massB,
    double dx, double dy, double dist, double min_dist)
{
    double restitution = 0.1;
    double nx = dx / dist;
    double ny = dy / dist;

    double relvx = velB.x - velA.x;
    double relvy = velB.y - velA.y;
    double relative_speed = relvx * nx + relvy * ny;

    if (relative_speed < 0) {
        double invMassA = 1.0 / massA.value;
        double invMassB = 1.0 / massB.value;
        double invMassSum = invMassA + invMassB;

        double impulse = -(1.0 + restitution)*relative_speed / invMassSum;

        velA.x -= (impulse * invMassA) * nx;
        velA.y -= (impulse * invMassA) * ny;
        velB.x += (impulse * invMassB) * nx;
        velB.y += (impulse * invMassB) * ny;
    }
}

static void handleSolidGasCollision(
    Components::Position& posA, Components::Velocity& velA, const Components::Mass& massA, Components::Phase /*phaseA*/,
    Components::Position& posB, Components::Velocity& velB, const Components::Mass& massB, Components::Phase /*phaseB*/,
    double dx, double dy, double dist, double min_dist)
{
    // For now, treat solid-gas like gas-gas
    handleGasGasCollision(posA, velA, massA, posB, velB, massB, dx, dy, dist, min_dist);
}

static void handleLiquidGasCollision(
    Components::Position& posA, Components::Velocity& /*velA*/, const Components::Mass& /*massA*/, Components::Phase /*phaseA*/,
    Components::Position& posB, Components::Velocity& /*velB*/, const Components::Mass& /*massB*/, Components::Phase /*phaseB*/,
    double dx, double dy, double dist, double min_dist)
{
    double nx = dx / dist;
    double ny = dy / dist;
    double penetration = min_dist - dist;
    posA.x -= 0.5 * penetration * nx;
    posA.y -= 0.5 * penetration * ny;
    posB.x += 0.5 * penetration * nx;
    posB.y += 0.5 * penetration * ny;
}

static void handleCollisionByPhase(
    Components::Position& posA, Components::Velocity& velA, const Components::Mass& massA, Components::Phase phaseA,
    Components::Position& posB, Components::Velocity& velB, const Components::Mass& massB, Components::Phase phaseB,
    double dx, double dy, double dist, double min_dist)
{
    if (phaseA == Components::Phase::Solid && phaseB == Components::Phase::Solid) {
        handleSolidSolidCollision(posA, velA, massA, posB, velB, massB, dx, dy, dist, min_dist);
    } else if ((phaseA == Components::Phase::Solid && phaseB == Components::Phase::Liquid) ||
               (phaseA == Components::Phase::Liquid && phaseB == Components::Phase::Solid)) {
        handleSolidLiquidCollision(posA, velA, massA, phaseA, posB, velB, massB, phaseB, dx, dy, dist, min_dist);
    } else if (phaseA == Components::Phase::Liquid && phaseB == Components::Phase::Liquid) {
        handleLiquidLiquidCollision(posA, velA, massA, posB, velB, massB, dx, dy, dist, min_dist);
    } else if (phaseA == Components::Phase::Gas && phaseB == Components::Phase::Gas) {
        handleGasGasCollision(posA, velA, massA, posB, velB, massB, dx, dy, dist, min_dist);
    } else if ((phaseA == Components::Phase::Solid && phaseB == Components::Phase::Gas) ||
               (phaseA == Components::Phase::Gas && phaseB == Components::Phase::Solid)) {
        handleSolidGasCollision(posA, velA, massA, phaseA, posB, velB, massB, phaseB, dx, dy, dist, min_dist);
    } else if ((phaseA == Components::Phase::Liquid && phaseB == Components::Phase::Gas) ||
               (phaseA == Components::Phase::Gas && phaseB == Components::Phase::Liquid)) {
        handleLiquidGasCollision(posA, velA, massA, phaseA, posB, velB, massB, phaseB, dx, dy, dist, min_dist);
    }
}

// Build the quadtree from all particles
static std::unique_ptr<QuadNode> buildQuadtree(entt::registry& registry) {
    // Universe boundaries: 0 to UniverseSizeMeters
    double size = SimulatorConstants::UniverseSizeMeters;
    int capacity = 8; // max particles per node before subdividing
    auto root = std::make_unique<QuadNode>(0.0, 0.0, size, capacity);

    auto view = registry.view<Components::Position, Components::Mass, Components::ParticlePhase>();
    for (auto entity : view) {
        auto &pos = view.get<Components::Position>(entity);

        double r = 0.0;
        if (auto radiusComp = registry.try_get<Components::Radius>(entity)) {
            r = radiusComp->value;
        }

        ParticleInfo pinfo;
        pinfo.entity = entity;
        pinfo.x = pos.x;
        pinfo.y = pos.y;
        pinfo.radius = r;

        // Insert into quadtree if within bounds
        if (pinfo.x >= 0 && pinfo.x < size && pinfo.y >= 0 && pinfo.y < size) {
            root->insert(pinfo);
        }
    }

    return root;
}

static void resolveCollisions(entt::registry& registry, QuadNode* root) {
    auto view = registry.view<Components::Position, Components::Velocity, Components::Mass, Components::ParticlePhase>();

    // For each particle, query neighbors from the quadtree
    for (auto entity : view) {
        auto &pos = view.get<Components::Position>(entity);
        auto &vel = view.get<Components::Velocity>(entity);
        auto &mass = view.get<Components::Mass>(entity);
        auto &phase = view.get<Components::ParticlePhase>(entity);

        double rA = 0.0;
        if (auto radiusA = registry.try_get<Components::Radius>(entity)) {
            rA = radiusA->value;
        }

        // Query region based on particle radius
        double query_size = (rA * 2.0) * 2.0; // a bit larger region
        // To ensure we find all candidates, we might need a safe margin
        // We'll just pick a small region around the particle: 
        double qx = pos.x - query_size*0.5;
        double qy = pos.y - query_size*0.5;

        std::vector<ParticleInfo> candidates;
        root->query(qx, qy, query_size, candidates);

        // Check collisions with candidates
        for (auto &candidate : candidates) {
            if (candidate.entity == entity) continue; // skip self

            // Retrieve other particle data
            auto &posB = registry.get<Components::Position>(candidate.entity);
            auto &velB = registry.get<Components::Velocity>(candidate.entity);
            auto &massB = registry.get<Components::Mass>(candidate.entity);
            auto &phaseB = registry.get<Components::ParticlePhase>(candidate.entity);

            double rB = candidate.radius;
            double combined_radius = rA + rB;
            if (combined_radius <= 0.0) continue;

            double dx = posB.x - pos.x;
            double dy = posB.y - pos.y;
            double dist_sq = dx*dx + dy*dy;

            if (dist_sq < (combined_radius * combined_radius)) {
                double dist = std::sqrt(dist_sq);
                if (dist < 1e-9) dist = combined_radius; // avoid div by zero
                handleCollisionByPhase(pos, vel, mass, phase.phase,
                                       posB, velB, massB, phaseB.phase,
                                       dx, dy, dist, combined_radius);
            }
        }
    }
}

void CollisionSystem::update(entt::registry& registry) {
    // Build quadtree from current particle distribution
    auto root = buildQuadtree(registry);

    // Broad-phase query and collision resolution
    resolveCollisions(registry, root.get());
}

} // namespace Systems