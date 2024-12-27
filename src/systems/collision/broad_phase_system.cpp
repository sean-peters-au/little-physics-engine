#include "nbody/core/constants.hpp"
#include "nbody/systems/collision/broad_phase_system.hpp"

#include <memory>
#include <algorithm>

struct ParticleInfo {
    entt::entity entity;
    double x, y;
};

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
        if (!contains(p.x, p.y)) return;

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
        if (nw->contains(p.x,p.y)) nw->insert(p);
        else if (ne->contains(p.x,p.y)) ne->insert(p);
        else if (sw->contains(p.x,p.y)) sw->insert(p);
        else if (se->contains(p.x,p.y)) se->insert(p);
    }

    void query(double qx, double qy, double qsize, std::vector<ParticleInfo>& found) const {
        if (!overlaps(qx, qy, qsize)) return;

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

    bool overlaps(double qx, double qy, double qsize) const {
        bool outside = (qx > x+size) || (qy > y+size) || (qx+qsize < x) || (qy+qsize < y);
        return !outside;
    }
};

static std::unique_ptr<QuadNode> buildQuadtree(entt::registry &registry) {
    // Instead of restricting ourselves to [0,0,size,size],
    // let's cover a larger area that includes the walls outside the boundary.
    // For example, if we placed walls at -lineThickness and size+lineThickness,
    // we can cover an area slightly larger than the universe.

    double size = SimulatorConstants::UniverseSizeMeters;
    double extra = 500.0; // Some margin to include walls outside normal range
                          // Adjust this if needed depending on your wall placement.

    // Now the quadtree covers from (-extra,-extra) to (size+extra, size+extra)
    auto root = std::make_unique<QuadNode>(-extra, -extra, size + 2*extra, 8);

    // Insert ALL entities, including walls, regardless of their position
    // Remove any conditions on position range
    auto view = registry.view<Components::Position>();
    for (auto e : view) {
        const auto &pos = view.get<Components::Position>(e);
        // Insert all entities (walls included) into the quadtree
        ParticleInfo pinfo { e, pos.x, pos.y };
        root->insert(pinfo);
    }
    return std::move(root);
}

void Systems::BroadPhaseSystem::update(entt::registry &registry, std::vector<CandidatePair> &candidatePairs) {
    candidatePairs.clear();
    auto root = buildQuadtree(registry);

    auto view = registry.view<Components::Position, Components::Mass, Components::ParticlePhase>();
    for (auto eA : view) {
        const auto &posA = view.get<Components::Position>(eA);
        double query_size = 1.0; 
        double qx = posA.x - query_size*0.5;
        double qy = posA.y - query_size*0.5;

        std::vector<ParticleInfo> candidates;
        root->query(qx, qy, query_size, candidates);

        for (auto &c : candidates) {
            if (c.entity != eA && c.entity > eA) { 
                candidatePairs.push_back({eA, c.entity});
            }
        }
    }
}