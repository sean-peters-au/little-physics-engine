#pragma once

#include <entt/entt.hpp>
#include <memory>
#include "nbody/components/basic.hpp"
#include "nbody/components/sim.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/systems/i_system.hpp"

namespace Systems {
    
    /**
     * @struct BarnesHutConfig
     * @brief Configuration parameters specific to the Barnes-Hut algorithm
     */
    struct BarnesHutConfig {
        // Theta is the accuracy parameter. Smaller = more accurate but slower
        double theta = 0.5;
    };
    
    /**
     * @class BarnesHutSystem
     * @brief Implements an n-body gravitational solver using the Barnes-Hut algorithm
     * 
     * The Barnes-Hut algorithm reduces the computational complexity from O(n²) to 
     * O(n log n) by approximating distant groups of bodies as single points.
     */
    class BarnesHutSystem : public ConfigurableSystem<BarnesHutConfig> {
    public:
        /**
         * @brief Constructor with default configuration
         */
        BarnesHutSystem();
        
        /**
         * @brief Virtual destructor
         */
        ~BarnesHutSystem() override = default;
        
        /**
         * @brief Updates gravitational forces on all entities
         * @param registry EnTT registry containing entities and components
         */
        void update(entt::registry& registry) override;

        struct QuadTreeNode {
            const entt::registry* registry = nullptr;
            double total_mass = 0.0;
            double center_of_mass_x = 0.0;
            double center_of_mass_y = 0.0;
            double boundary_x = 0.0;
            double boundary_y = 0.0;
            double boundary_size = 0.0;
            bool is_leaf = true;
            entt::entity particle;  // Only used if is_leaf and contains exactly one particle
            
            std::unique_ptr<QuadTreeNode> nw;
            std::unique_ptr<QuadTreeNode> ne;
            std::unique_ptr<QuadTreeNode> sw;
            std::unique_ptr<QuadTreeNode> se;

            bool contains(double x, double y) const {
                return x >= boundary_x && x < boundary_x + boundary_size &&
                       y >= boundary_y && y < boundary_y + boundary_size;
            }

            int getQuadrant(double x, double y) const {
                double mid_x = boundary_x + boundary_size / 2;
                double mid_y = boundary_y + boundary_size / 2;
                if (x < mid_x) {
                    return (y < mid_y) ? 0 : 2;  // NW : SW
                } else {
                    return (y < mid_y) ? 1 : 3;  // NE : SE
                }
            }
        };

    private:
        std::unique_ptr<QuadTreeNode> buildTree(const entt::registry& registry);
        void insertParticle(QuadTreeNode* node, entt::entity entity, 
                            const Components::Position& pos,
                            const Components::Mass& mass);
        void subdivide(QuadTreeNode* node);
        void calculateForce(const QuadTreeNode* node,
                            entt::entity entity,
                            const Components::Position& pos,
                            Components::Velocity& vel,
                            const Components::Mass& mass,
                            const Components::SimulatorState& state);
    };
} 