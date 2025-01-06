#include <cmath>
#include <memory>
#include <vector>

#include "nbody/systems/rigid_body_collision/broadphase.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/math/polygon.hpp"
#include "nbody/systems/rigid_body_collision/collision_data.hpp"

namespace RigidBodyCollision
{

// ------------------------------------------------------------------
// Minimal structures used internally by the quadtree

struct AABBEntity {
    entt::entity entity;
    double minx, miny, maxx, maxy;
};

static bool boxesOverlap(const AABBEntity &a, const AABBEntity &b) {
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
            se->query(qminx, qminy, qmaxy, qmaxy, found); // minor bug in original
        }
    }
};

// ------------------------------------------------------------------
// Helper: compute AABB for an entity (circle or polygon)
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
        double r = reg.get<CircleShape>(e).radius;
        minx = pos.x - r; maxx = pos.x + r;
        miny = pos.y - r; maxy = pos.y + r;
        return;
    }

    // Otherwise polygon
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

// ------------------------------------------------------------------
// Build the quadtree root
static std::unique_ptr<BoxNode> buildQuadtree(entt::registry &registry) {
    double size = SimulatorConstants::UniverseSizeMeters;
    double extra = 500.0; // some buffer
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

// ------------------------------------------------------------------
// The public broadPhase function
std::vector<CandidatePair> broadPhase(entt::registry &registry)
{
    std::vector<CandidatePair> pairs;
    pairs.reserve(128);

    auto root = buildQuadtree(registry);

    std::vector<AABBEntity> found;
    found.reserve(64);

    // For each entity, query
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

    return pairs;
}

} // namespace RigidBodyCollision