/**
 * @fileoverview
 * @brief Implements an n-body gravitational solver using the Barnes-Hut algorithm.
 *
 * The Barnes-Hut algorithm reduces the computational complexity from O(n²) to O(n log n)
 * by approximating distant groups of bodies as single points. This system builds a quadtree,
 * aggregates mass and center of mass in each node, and then calculates gravitational forces
 * more efficiently for distant nodes.
 *
 * Optional feature: If every particle in a quadtree node is considered "small" (mass below
 * a threshold), we skip that entire subtree when computing forces. This trades off accuracy
 * for performance.
 */

#pragma once

#include <entt/entt.hpp>
#include <memory>
#include <vector>

#include "systems/i_system.hpp"
#include "entities/entity_components.hpp"
#include "entities/sim_components.hpp"

namespace Systems {

/**
 * @struct BarnesHutConfig
 * @brief Configuration parameters specific to the Barnes-Hut algorithm
 */
struct BarnesHutConfig {
    /**
     * @brief Accuracy parameter for Barnes-Hut.
     * Smaller theta = more accuracy but slower performance.
     */
    double theta = 0.5;

    /**
     * @brief Mass threshold for skipping small particles entirely.
     *
     * If > 0, quadtree nodes that contain only particles each below this mass
     * will be skipped from force calculation. This can improve performance
     * but reduces physical accuracy.
     */
    double smallMassThreshold = 1e3;
};

/**
 * @class BarnesHutSystem
 * @brief N-body solver using the Barnes-Hut algorithm
 *
 * This system:
 *  - Builds a quadtree of all non-boundary entities (with Mass, Position).
 *  - Aggregates mass/center of mass in each node.
 *  - Recursively calculates gravitational forces.
 *  - Skips entire nodes that contain only sub-threshold masses (optional).
 */
class BarnesHutSystem : public ConfigurableSystem<BarnesHutConfig> {
public:
    /**
     * @brief Default constructor
     */
    BarnesHutSystem();

    /**
     * @brief Virtual destructor
     */
    ~BarnesHutSystem() override = default;

    /**
     * @brief Updates gravitational forces on all (non-boundary) entities
     * @param registry The ECS registry
     */
    void update(entt::registry& registry) override;

private:
    /**
     * @struct QuadTreeNode
     * @brief Node in the Barnes-Hut quadtree
     */
    struct QuadTreeNode {
        // Default constructor to reset state
        QuadTreeNode() : registry(nullptr), totalMass(0.0), centerOfMassX(0.0), centerOfMassY(0.0),
                         boundaryX(0.0), boundaryY(0.0), boundarySize(0.0),
                         isLeaf(true), allSmall(true), singleParticle(entt::null),
                         nw(nullptr), ne(nullptr), sw(nullptr), se(nullptr) {}

        const entt::registry* registry = nullptr;
        double totalMass = 0.0;
        double centerOfMassX = 0.0;
        double centerOfMassY = 0.0;

        double boundaryX = 0.0;
        double boundaryY = 0.0;
        double boundarySize = 0.0;

        bool isLeaf = true;
        bool allSmall = true;  ///< Whether all masses in this node are below the small-mass threshold

        entt::entity singleParticle = entt::null;  ///< If leaf with exactly one particle

        // Change from unique_ptr to raw pointers
        QuadTreeNode* nw = nullptr;
        QuadTreeNode* ne = nullptr;
        QuadTreeNode* sw = nullptr;
        QuadTreeNode* se = nullptr;

        /**
         * @brief Checks whether a point (x, y) is within this node's bounding square
         */
        bool contains(double x, double y) const {
            return (x >= boundaryX && x < boundaryX + boundarySize &&
                    y >= boundaryY && y < boundaryY + boundarySize);
        }

        /**
         * @brief Determines which quadrant of this node the point (x, y) belongs in
         * @return 0 = NW, 1 = NE, 2 = SW, 3 = SE
         */
        int getQuadrant(double x, double y) const {
            double midX = boundaryX + boundarySize * 0.5;
            double midY = boundaryY + boundarySize * 0.5;

            if (x < midX) {
                // NW or SW
                return (y < midY) ? 0 : 2;
            } else {
                // NE or SE
                return (y < midY) ? 1 : 3;
            }
        }
    };

    /**
     * @brief Builds the quadtree from all applicable particles
     * @return Raw pointer to the root node (owned by the pool)
     */
    QuadTreeNode* buildTree(const entt::registry& registry);

    /**
     * @brief Inserts a single particle into the quadtree
     */
    void insertParticle(QuadTreeNode* node,
                        entt::entity entity,
                        const Components::Position& pos,
                        const Components::Mass& mass);

    /**
     * @brief Subdivides a leaf node into four child nodes
     */
    void subdivide(QuadTreeNode* node);

    /**
     * @brief Recursively computes the gravitational force from the quadtree node
     */
    void calculateForce(const QuadTreeNode* node,
                        entt::entity entity,
                        const Components::Position& pos,
                        Components::Velocity& vel,
                        const Components::Mass& mass,
                        const Components::SimulatorState& state);

    /**
     * @brief Helper to get a node from the pool, resizing if necessary.
     * @return Pointer to an initialized node.
     */
    QuadTreeNode* allocateNode();

    // Node Pool members
    std::vector<QuadTreeNode> nodePool_;
    size_t nextNodeIndex_ = 0;
    static constexpr size_t INITIAL_POOL_SIZE = 1024; // Or some reasonable starting size
};

}  // namespace Systems