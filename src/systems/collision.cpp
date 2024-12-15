#include "nbody/systems/collision.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <memory>

namespace Systems {

struct ParticleInfo {
    entt::entity entity;
    double x, y;
    Components::ShapeType type;
    double size; // radius if circle, half-size if square
};

// Quadtree node (unchanged)
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
        if (!contains(p.x, p.y)) return;

        if (is_leaf && (int)particles.size() < capacity) {
            particles.push_back(p);
            return;
        }

        if (is_leaf) {
            subdivide();
            for (auto &part : particles) {
                insertIntoChild(part);
            }
            particles.clear();
        }

        insertIntoChild(p);
    }

    void insertIntoChild(const ParticleInfo &p) {
        if (nw->contains(p.x, p.y)) nw->insert(p);
        else if (ne->contains(p.x, p.y)) ne->insert(p);
        else if (sw->contains(p.x, p.y)) sw->insert(p);
        else if (se->contains(p.x, p.y)) se->insert(p);
    }

    void query(double qx, double qy, double qsize, std::vector<ParticleInfo>& found) const {
        if (!overlaps(qx, qy, qsize)) return;

        if (is_leaf) {
            for (auto &part : particles) {
                if (part.x >= qx && part.x <= qx + qsize && 
                    part.y >= qy && part.y <= qy + qsize) {
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

    bool overlaps(double qx, double qy, double qsize) const {
        bool outside = (qx > x+size) || (qy > y+size) || (qx+qsize < x) || (qy+qsize < y);
        return !outside;
    }
};

// Helper functions for shape collisions
static bool circleCircleCollision(double xA, double yA, double rA,
                                  double xB, double yB, double rB,
                                  double &dx, double &dy, double &dist, double &min_dist) {
    dx = xB - xA;
    dy = yB - yA;
    double dist_sq = dx*dx + dy*dy;
    min_dist = rA + rB;
    if (dist_sq < (min_dist * min_dist)) {
        dist = std::sqrt(dist_sq);
        if (dist < 1e-9) dist = min_dist;
        return true;
    }
    return false;
}

static bool squareSquareCollision(double xA, double yA, double halfA,
                                  double xB, double yB, double halfB,
                                  double &dx, double &dy, double &dist, double &min_dist) {
    // Squares are axis-aligned. Check overlap in x and y
    double leftA = xA - halfA;
    double rightA = xA + halfA;
    double topA = yA - halfA;
    double bottomA = yA + halfA;

    double leftB = xB - halfB;
    double rightB = xB + halfB;
    double topB = yB - halfB;
    double bottomB = yB + halfB;

    // Overlap in x?
    if (rightA < leftB || leftA > rightB) return false;
    // Overlap in y?
    if (bottomA < topB || topA > bottomB) return false;

    // If overlapping, find smallest overlap direction
    double overlapX = std::min(rightA, rightB) - std::max(leftA, leftB);
    double overlapY = std::min(bottomA, bottomB) - std::max(topA, topB);

    // Use the smaller overlap to define penetration
    if (overlapX < overlapY) {
        // Push apart in x
        dx = (xB > xA) ? overlapX : -overlapX;
        dy = 0.0;
        dist = std::fabs(dx);
        min_dist = dist; // no radius concept here
    } else {
        dx = 0.0;
        dy = (yB > yA) ? overlapY : -overlapY;
        dist = std::fabs(dy);
        min_dist = dist;
    }

    return true;
}

static bool circleSquareCollision(double xC, double yC, double r,
                                  double xS, double yS, double halfS,
                                  double &dx, double &dy, double &dist, double &min_dist) {
    // Closest point on the square to the circle center
    double left = xS - halfS;
    double right = xS + halfS;
    double top = yS - halfS;
    double bottom = yS + halfS;

    double closestX = (xC < left) ? left : (xC > right ? right : xC);
    double closestY = (yC < top) ? top : (yC > bottom ? bottom : yC);

    dx = closestX - xC;
    dy = closestY - yC;
    double dist_sq = dx*dx + dy*dy;
    if (dist_sq < r*r) {
        dist = std::sqrt(dist_sq);
        if (dist < 1e-9) dist = r;
        min_dist = r; // the circle radius defines the min_dist
        return true;
    }
    return false;
}

// Collision resolution functions
// We'll modify them to accept dx, dy, dist, min_dist directly, since we computed them above.

static void resolveCollision(Components::Position& posA, Components::Velocity& velA, const Components::Mass& massA,
                             Components::Position& posB, Components::Velocity& velB, const Components::Mass& massB,
                             Components::Phase phaseA, Components::Phase phaseB,
                             double dx, double dy, double dist, double min_dist) 
{
    // Determine restitution based on phase
    double restitution;
    if (phaseA == Components::Phase::Solid && phaseB == Components::Phase::Solid) {
        restitution = 0.9;
    } else if (phaseA == Components::Phase::Gas && phaseB == Components::Phase::Gas) {
        restitution = 0.1;
    } else {
        // For mixed phases or liquid-liquid, just use a middle ground
        restitution = 0.5;
    }

    double nx, ny;
    if (dist > 1e-9) {
        nx = dx / dist;
        ny = dy / dist;
    } else {
        nx = 1.0; ny = 0.0; 
    }

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

// Build the quadtree from all particles
static std::unique_ptr<QuadNode> buildQuadtree(entt::registry& registry) {
    double size = SimulatorConstants::UniverseSizeMeters;
    int capacity = 8; 
    auto root = std::make_unique<QuadNode>(0.0, 0.0, size, capacity);

    auto view = registry.view<Components::Position, Components::Mass, Components::ParticlePhase, Components::Shape>();
    for (auto entity : view) {
        auto &pos = view.get<Components::Position>(entity);
        auto &shape = view.get<Components::Shape>(entity);

        ParticleInfo pinfo;
        pinfo.entity = entity;
        pinfo.x = pos.x;
        pinfo.y = pos.y;
        pinfo.type = shape.type;
        pinfo.size = shape.size;

        if (pinfo.x >= 0 && pinfo.x < size && pinfo.y >= 0 && pinfo.y < size) {
            root->insert(pinfo);
        }
    }

    return root;
}

static void resolveCollisions(entt::registry& registry, QuadNode* root) {
    auto view = registry.view<Components::Position, Components::Velocity, Components::Mass, Components::ParticlePhase, Components::Shape>();

    for (auto entity : view) {
        auto &posA = view.get<Components::Position>(entity);
        auto &velA = view.get<Components::Velocity>(entity);
        auto &massA = view.get<Components::Mass>(entity);
        auto &phaseA = view.get<Components::ParticlePhase>(entity);
        auto &shapeA = view.get<Components::Shape>(entity);

        double query_size = (shapeA.type == Components::ShapeType::Circle) ? (shapeA.size*4.0) : (shapeA.size*4.0);
        double qx = posA.x - query_size*0.5;
        double qy = posA.y - query_size*0.5;

        std::vector<ParticleInfo> candidates;
        root->query(qx, qy, query_size, candidates);

        for (auto &candidate : candidates) {
            if (candidate.entity == entity) continue; // skip self

            auto &posB = registry.get<Components::Position>(candidate.entity);
            auto &velB = registry.get<Components::Velocity>(candidate.entity);
            auto &massB = registry.get<Components::Mass>(candidate.entity);
            auto &phaseB = registry.get<Components::ParticlePhase>(candidate.entity);
            auto &shapeB = registry.get<Components::Shape>(candidate.entity);

            double dx, dy, dist, min_dist;
            bool collided = false;

            if (shapeA.type == Components::ShapeType::Circle && shapeB.type == Components::ShapeType::Circle) {
                // circle-circle
                collided = circleCircleCollision(posA.x, posA.y, shapeA.size, posB.x, posB.y, shapeB.size,
                                                 dx, dy, dist, min_dist);
            } else if (shapeA.type == Components::ShapeType::Square && shapeB.type == Components::ShapeType::Square) {
                // square-square
                // We don't have a radius-dist concept. We'll just get dx,dy from overlap
                collided = squareSquareCollision(posA.x, posA.y, shapeA.size,
                                                 posB.x, posB.y, shapeB.size,
                                                 dx, dy, dist, min_dist);
            } else {
                // circle-square or square-circle
                // order doesn't matter, just check both ways
                if (shapeA.type == Components::ShapeType::Circle && shapeB.type == Components::ShapeType::Square) {
                    collided = circleSquareCollision(posA.x, posA.y, shapeA.size,
                                                      posB.x, posB.y, shapeB.size,
                                                      dx, dy, dist, min_dist);
                    // dx, dy points from circle center to closest point on square, 
                    // we want it to serve as our normal direction
                    // It's fine as is for resolving collision.
                } else {
                    // square-circle: just swap roles
                    collided = circleSquareCollision(posB.x, posB.y, shapeB.size,
                                                      posA.x, posA.y, shapeA.size,
                                                      dx, dy, dist, min_dist);
                    // This returns dx,dy from B->A perspective since we swapped roles
                    // Invert dx, dy to keep consistent perspective (from A to B)
                    dx = -dx;
                    dy = -dy;
                }
            }

            if (collided) {
                // Resolve collision
                resolveCollision(posA, velA, massA,
                                 posB, velB, massB,
                                 phaseA.phase, phaseB.phase,
                                 dx, dy, dist, min_dist);
            }
        }
    }
}

void CollisionSystem::update(entt::registry& registry) {
    auto root = buildQuadtree(registry);
    resolveCollisions(registry, root.get());
}

} // namespace Systems