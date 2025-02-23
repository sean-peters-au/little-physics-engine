/**
 * @file broadphase.cpp
 * @brief Implementation of quadtree-based broad-phase collision detection
 */

#include <cmath>
#include <memory>
#include <vector>

#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/core/profile.hpp"
#include "nbody/math/polygon.hpp"
#include "nbody/systems/rigid/broadphase.hpp"
#include "nbody/systems/rigid/collision_data.hpp"

namespace RigidBodyCollision
{

/**
 * @brief Entity data with its axis-aligned bounding box
 */
struct AABBEntity {
    entt::entity entity;
    double minx, miny, maxx, maxy;
};

/**
 * @brief Tests if two AABBs overlap
 */
static bool boxesOverlap(const AABBEntity &a, const AABBEntity &b) {
    if (a.maxx < b.minx || a.minx > b.maxx) { return false;
}
    if (a.maxy < b.miny || a.miny > b.maxy) { return false;
}
    return true;
}

/**
 * @brief A node in the quadtree spatial partitioning structure
 * 
 * Each node represents a square region of space that can either:
 * - Be a leaf containing up to 'capacity' objects
 * - Be subdivided into four child nodes (NW, NE, SW, SE)
 */
struct BoxNode {
    double x, y, size;
    int capacity;
    bool is_leaf{true};
    std::vector<AABBEntity> objects;
    std::unique_ptr<BoxNode> nw, ne, sw, se;

    BoxNode(double x, double y, double size, int capacity)
        : x(x), y(y), size(size), capacity(capacity) {}

    /**
     * @brief Tests if an AABB is fully contained within this node
     */
    [[nodiscard]] bool nodeContains(const AABBEntity &bb) const {
        return (bb.minx >= x && bb.maxx < (x + size) &&
                bb.miny >= y && bb.maxy < (y + size));
    }

    /**
     * @brief Tests if an AABB overlaps this node's region
     */
    [[nodiscard]] bool nodeOverlaps(const AABBEntity &bb) const {
        if (bb.maxx < x || bb.minx > x+size) { return false;
}
        if (bb.maxy < y || bb.miny > y+size) { return false;
}
        return true;
    }

    /**
     * @brief Subdivides a leaf node into four children
     */
    void subdivide() {
        double const half = size / 2.0;
        nw = std::make_unique<BoxNode>(x,       y,       half, capacity);
        ne = std::make_unique<BoxNode>(x+half,  y,       half, capacity);
        sw = std::make_unique<BoxNode>(x,       y+half,  half, capacity);
        se = std::make_unique<BoxNode>(x+half,  y+half,  half, capacity);
        is_leaf = false;
    }

    /**
     * @brief Inserts an AABB into the quadtree
     * 
     * If the node is full, it will be subdivided and objects redistributed
     * to the appropriate child nodes. Objects that span multiple children
     * remain in the parent node.
     */
    void insert(const AABBEntity &bb) {
        if (!nodeOverlaps(bb)) { return;
}

        if (is_leaf && static_cast<int>(objects.size()) < capacity) {
            objects.push_back(bb);
            return;
        }

        if (is_leaf) {
            subdivide();
            auto oldObjs = std::move(objects);
            objects.clear();
            for (auto &o : oldObjs) {
                if (nodeContains(o)) {
                    if      (nw->nodeContains(o)) { nw->insert(o);
                    } else if (ne->nodeContains(o)) { ne->insert(o);
                    } else if (sw->nodeContains(o)) { sw->insert(o);
                    } else if (se->nodeContains(o)) { se->insert(o);
                    } else { objects.push_back(o);
}
                } else {
                    objects.push_back(o);
                }
            }
        }

        if (nodeContains(bb)) {
            if      (nw->nodeContains(bb)) { nw->insert(bb);
            } else if (ne->nodeContains(bb)) { ne->insert(bb);
            } else if (sw->nodeContains(bb)) { sw->insert(bb);
            } else if (se->nodeContains(bb)) { se->insert(bb);
            } else { objects.push_back(bb);
}
        } else {
            objects.push_back(bb);
        }
    }

    /**
     * @brief Finds all AABBs that overlap with the query region
     */
    void query(double qminx, double qminy, double qmaxx, double qmaxy,
               std::vector<AABBEntity> &found) const
    {
        if (qmaxx < x || qminx > x+size) { return;
}
        if (qmaxy < y || qminy > y+size) { return;
}

        for (const auto &o : objects) {
            if (o.maxx < qminx || o.minx > qmaxx) { continue;
}
            if (o.maxy < qminy || o.miny > qmaxy) { continue;
}
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
 * @brief Computes the AABB for an entity's shape (circle or polygon)
 * 
 * Takes into account the entity's position and rotation to compute
 * a world-space axis-aligned bounding box.
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

    // Circle?
    if (reg.any_of<CircleShape>(e)) {
        double const r = reg.get<CircleShape>(e).radius;
        minx = pos.x - r; maxx = pos.x + r;
        miny = pos.y - r; maxy = pos.y + r;
        return;
    }

    // Otherwise polygon
    const auto &poly = reg.get<PolygonShape>(e);
    minx = pos.x; maxx = pos.x;
    miny = pos.y; maxy = pos.y;
    for (const auto &v : poly.vertices) {
        double const rx = v.x*std::cos(angle) - v.y*std::sin(angle);
        double const ry = v.x*std::sin(angle) + v.y*std::cos(angle);
        double const wx = pos.x + rx;
        double const wy = pos.y + ry;

        if (wx < minx) { minx = wx;
}
        if (wx > maxx) { maxx = wx;
}
        if (wy < miny) { miny = wy;
}
        if (wy > maxy) { maxy = wy;
}
    }
}

/**
 * @brief Constructs a quadtree containing all collidable entities
 * 
 * Creates a root node slightly larger than the universe size and
 * inserts AABBs for all entities with Position, Mass, and ParticlePhase
 * components.
 */
static std::unique_ptr<BoxNode> buildQuadtree(entt::registry &registry) {
    double const size = SimulatorConstants::UniverseSizeMeters;
    double const extra = 500.0; // some buffer
    auto root = std::make_unique<BoxNode>(-extra, -extra, size + 2*extra, 8);

    auto view = registry.view<Components::Position, Components::Mass, Components::ParticlePhase>();
    for (auto e : view) {
        // Only use rigid/solid bodies
        if (view.get<Components::ParticlePhase>(e).phase != Components::Phase::Solid) {
            continue;
        }
        double minx;
        double miny;
        double maxx;
        double maxy;
        computeAABB(registry, e, minx, miny, maxx, maxy);
        AABBEntity const box{ e, minx, miny, maxx, maxy };
        root->insert(box);
    }
    return root;
}

std::vector<CandidatePair> broadPhase(entt::registry &registry)
{
    PROFILE_SCOPE("Broadphase");
    // Build the quadtree spatial partitioning structure using only rigid/solid bodies
    auto root = buildQuadtree(registry);
    
    std::vector<CandidatePair> pairs;
    std::vector<AABBEntity> found;
    pairs.reserve(128);
    found.reserve(64);

    // For each entity, find potential collision partners by querying the quadtree with its AABB
    auto view = registry.view<Components::Position, Components::Mass, Components::ParticlePhase>();
    for (auto e : view) {
        // Only process rigid/solid bodies
        if (view.get<Components::ParticlePhase>(e).phase != Components::Phase::Solid) {
            continue;
        }
        double minx;
        double miny;
        double maxx;
        double maxy;
        computeAABB(registry, e, minx, miny, maxx, maxy);
        AABBEntity const queryBB { e, minx, miny, maxx, maxy };

        found.clear();
        root->query(minx, miny, maxx, maxy, found);

        for (auto &f : found) {
            // Process only if:
            //  1. Not the same entity
            //  2. The second entity has a higher id (to avoid duplicate pairs)
            if (f.entity == e || f.entity < e)
                continue;
            
            // Make sure the candidate is also a rigid/solid body
            if (registry.get<Components::ParticlePhase>(f.entity).phase != Components::Phase::Solid) {
                continue;
            }

            bool const aBoundary = registry.any_of<Components::Boundary>(e);
            bool const bBoundary = registry.any_of<Components::Boundary>(f.entity);
            
            // Skip if both are boundaries
            if (!(aBoundary && bBoundary)) {
                if (boxesOverlap(queryBB, f)) {
                    pairs.push_back({ e, f.entity });
                }
            }
        }
    }

    return pairs;
}

} // namespace RigidBodyCollision