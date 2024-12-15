#include "nbody/systems/collision.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"

#include <cmath>
#include <algorithm>
#include <vector>
#include <memory>
#include <iostream>

namespace Systems {

struct ParticleInfo {
    entt::entity entity;
    double x, y;
    Components::ShapeType type;
    double size;
};

struct QuadNode {
    double x, y;
    double size;
    int capacity;
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

static void applyAngularImpulse(entt::registry& registry,
                                entt::entity eA, entt::entity eB,
                                double nx, double ny,
                                double impulse,
                                double cx, double cy) {
    if (!registry.all_of<Components::AngularVelocity, Components::AngularPosition, Components::Inertia>(eA)) return;
    if (!registry.all_of<Components::AngularVelocity, Components::AngularPosition, Components::Inertia>(eB)) return;

    auto &posA = registry.get<Components::Position>(eA);
    auto &posB = registry.get<Components::Position>(eB);

    auto &angVelA = registry.get<Components::AngularVelocity>(eA);
    auto &inertiaA = registry.get<Components::Inertia>(eA);

    auto &angVelB = registry.get<Components::AngularVelocity>(eB);
    auto &inertiaB = registry.get<Components::Inertia>(eB);

    double rxA = cx - posA.x;
    double ryA = cy - posA.y;
    double rxB = cx - posB.x;
    double ryB = cy - posB.y;

    double angularImpulseA = (rxA * ny - ryA * nx)*impulse;
    double angularImpulseB = (rxB * ny - ryB * nx)*(-impulse);

    angVelA.omega += angularImpulseA / inertiaA.I;
    angVelB.omega += angularImpulseB / inertiaB.I;
}

// New helper: apply friction (tangential impulse) to reduce jitter
static void applyFriction(entt::registry& registry, entt::entity eA, entt::entity eB,
                          double nx, double ny, double impulseN) {
    auto &velA = registry.get<Components::Velocity>(eA);
    auto &velB = registry.get<Components::Velocity>(eB);
    auto &massA = registry.get<Components::Mass>(eA);
    auto &massB = registry.get<Components::Mass>(eB);

    double invMassA = 1.0 / massA.value;
    double invMassB = 1.0 / massB.value;

    // Relative velocity after normal impulse
    double relvx = velB.x - velA.x;
    double relvy = velB.y - velA.y;

    // Tangent direction
    double tx = -ny;
    double ty = nx;

    // Project rel velocity on tangent
    double relTangentSpeed = relvx * tx + relvy * ty;
    if (std::fabs(relTangentSpeed) < 1e-9) return;

    // Friction coefficient (simple constant)
    double frictionCoeff = 0.3; // Adjust as needed
    double maxFrictionImpulse = frictionCoeff * impulseN;

    double tangentImpulse = -relTangentSpeed / (invMassA + invMassB);
    // Limit friction impulse
    if (tangentImpulse > maxFrictionImpulse) tangentImpulse = maxFrictionImpulse;
    if (tangentImpulse < -maxFrictionImpulse) tangentImpulse = -maxFrictionImpulse;

    velA.x -= (tangentImpulse * invMassA) * tx;
    velA.y -= (tangentImpulse * invMassA) * ty;
    velB.x += (tangentImpulse * invMassB) * tx;
    velB.y += (tangentImpulse * invMassB) * ty;
}

static void resolveCollision(entt::registry& registry,
                             entt::entity eA, entt::entity eB,
                             double dx, double dy, double dist, double min_dist) 
{
    auto &posA = registry.get<Components::Position>(eA);
    auto &velA = registry.get<Components::Velocity>(eA);
    auto &massA = registry.get<Components::Mass>(eA);
    auto &phaseA = registry.get<Components::ParticlePhase>(eA);

    auto &posB = registry.get<Components::Position>(eB);
    auto &velB = registry.get<Components::Velocity>(eB);
    auto &massB = registry.get<Components::Mass>(eB);
    auto &phaseB = registry.get<Components::ParticlePhase>(eB);

    // Compute restitution based on relative speed
    double nx = dx / dist;
    double ny = dy / dist;
    double relvx = velB.x - velA.x;
    double relvy = velB.y - velA.y;
    double relative_speed = relvx * nx + relvy * ny;

    double baseRestitution;
    if (phaseA.phase == Components::Phase::Solid && phaseB.phase == Components::Phase::Solid) {
        baseRestitution = 0.9;
    } else if (phaseA.phase == Components::Phase::Gas && phaseB.phase == Components::Phase::Gas) {
        baseRestitution = 0.1;
    } else {
        baseRestitution = 0.5;
    }

    // Reduce restitution for low-speed collisions
    if (std::fabs(relative_speed) < 0.5) {
        baseRestitution *= 0.5;
    }

    double invMassA = 1.0 / massA.value;
    double invMassB = 1.0 / massB.value;
    double invMassSum = invMassA + invMassB;

    // Positional correction factors
    double baumgarte = 0.2;           // fraction of penetration to correct per step
    double penetration_slop = 0.001;  // don't correct if penetration is very small

    double penetration = min_dist - dist;
    if (penetration > penetration_slop && relative_speed < 0) {
        // Apply normal impulse
        double impulse = -(1.0 + baseRestitution)*relative_speed / invMassSum;
        velA.x -= (impulse * invMassA) * nx;
        velA.y -= (impulse * invMassA) * ny;
        velB.x += (impulse * invMassB) * nx;
        velB.y += (impulse * invMassB) * ny;

        // Positional correction using Baumgarte stabilization
        double correction = (std::max(penetration - penetration_slop, 0.0)*baumgarte) / invMassSum;
        posA.x -= correction * invMassA * nx;
        posA.y -= correction * invMassA * ny;
        posB.x += correction * invMassB * nx;
        posB.y += correction * invMassB * ny;

        // Angular impulse
        double cx = (posA.x + posB.x)*0.5;
        double cy = (posA.y + posB.y)*0.5;
        applyAngularImpulse(registry, eA, eB, nx, ny, impulse, cx, cy);

        // Apply friction impulse to reduce jittery sliding/spinning
        applyFriction(registry, eA, eB, nx, ny, impulse);

    } else if (penetration > penetration_slop && relative_speed >= 0) {
        // If they're overlapping but moving apart, we can still do a small positional correction
        double correction = (std::max(penetration - penetration_slop, 0.0)*baumgarte) / invMassSum;
        posA.x -= correction * invMassA * nx;
        posA.y -= correction * invMassA * ny;
        posB.x += correction * invMassB * nx;
        posB.y += correction * invMassB * ny;
    }
}

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
    double leftA = xA - halfA;
    double rightA = xA + halfA;
    double topA = yA - halfA;
    double bottomA = yA + halfA;

    double leftB = xB - halfB;
    double rightB = xB + halfB;
    double topB = yB - halfB;
    double bottomB = yB + halfB;

    if (rightA < leftB || leftA > rightB) return false;
    if (bottomA < topB || topA > bottomB) return false;

    double overlapX = std::min(rightA, rightB) - std::max(leftA, leftB);
    double overlapY = std::min(bottomA, bottomB) - std::max(topA, topB);

    if (overlapX < overlapY) {
        dx = (xB > xA) ? overlapX : -overlapX;
        dy = 0.0;
        dist = std::fabs(dx);
        min_dist = dist;
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
        min_dist = r;
        return true;
    }
    return false;
}

static void resolveCollisions(entt::registry& registry, QuadNode* root) {
    auto view = registry.view<Components::Position, Components::Velocity, Components::Mass, Components::ParticlePhase, Components::Shape>();

    for (auto eA : view) {
        auto &posA = view.get<Components::Position>(eA);
        auto &shapeA = view.get<Components::Shape>(eA);

        double query_size = shapeA.size * 4.0;
        double qx = posA.x - query_size*0.5;
        double qy = posA.y - query_size*0.5;

        std::vector<ParticleInfo> candidates;
        root->query(qx, qy, query_size, candidates);

        for (auto &candidate : candidates) {
            entt::entity eB = candidate.entity;
            if (eA == eB) continue;

            auto &posB = registry.get<Components::Position>(eB);
            auto &shapeB = registry.get<Components::Shape>(eB);

            double dx, dy, dist, min_dist;
            bool collided = false;

            if (shapeA.type == Components::ShapeType::Circle && shapeB.type == Components::ShapeType::Circle) {
                collided = circleCircleCollision(posA.x, posA.y, shapeA.size,
                                                 posB.x, posB.y, shapeB.size,
                                                 dx, dy, dist, min_dist);
            } else if (shapeA.type == Components::ShapeType::Square && shapeB.type == Components::ShapeType::Square) {
                collided = squareSquareCollision(posA.x, posA.y, shapeA.size,
                                                 posB.x, posB.y, shapeB.size,
                                                 dx, dy, dist, min_dist);
            } else {
                // circle-square
                if (shapeA.type == Components::ShapeType::Circle && shapeB.type == Components::ShapeType::Square) {
                    collided = circleSquareCollision(posA.x, posA.y, shapeA.size,
                                                      posB.x, posB.y, shapeB.size,
                                                      dx, dy, dist, min_dist);
                } else {
                    // square-circle
                    collided = circleSquareCollision(posB.x, posB.y, shapeB.size,
                                                      posA.x, posA.y, shapeA.size,
                                                      dx, dy, dist, min_dist);
                    dx = -dx;
                    dy = -dy;
                }
            }

            if (collided) {
                resolveCollision(registry, eA, eB, dx, dy, dist, min_dist);
            }
        }
    }
}

void CollisionSystem::update(entt::registry& registry) {
    auto root = buildQuadtree(registry);
    resolveCollisions(registry, root.get());
}

} // namespace Systems