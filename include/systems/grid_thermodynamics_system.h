#ifndef GRID_THERMODYNAMICS_SYSTEM_H
#define GRID_THERMODYNAMICS_SYSTEM_H

#include <entt/entt.hpp>
#include <vector>
#include <array>
#include "components.h"
#include "simulator_constants.h"

namespace Systems {
    class GridThermodynamicsSystem {
    public:
        static constexpr size_t MAX_PARTICLES_PER_CELL = 128;
        
        struct Cell {
            double total_mass = 0.0;
            double density = 0.0;  // kg/m³
            double temperature = 0.0;  // K
            size_t particle_count = 0;
            std::array<entt::entity, MAX_PARTICLES_PER_CELL> particles;

            void add_particle(entt::entity entity) {
                if (particle_count < MAX_PARTICLES_PER_CELL) {
                    particles[particle_count++] = entity;
                }
            }

            void clear() {
                total_mass = 0.0;
                density = 0.0;
                temperature = 0.0;
                particle_count = 0;
            }
        };

        using Grid = std::vector<std::vector<Cell>>;
        static void update(entt::registry& registry);

    private:
        static Grid create_grid();
        static void populate_grid(Grid& grid, entt::registry& registry);
        static void calculate_cell_properties(Grid& grid);
        static void apply_thermodynamics(const Grid& grid, entt::registry& registry);
        static void apply_cell_effects(const Cell& cell, 
                                     Components::Position& pos,
                                     Components::Velocity& vel,
                                     const Components::Mass& mass);
    };
}

#endif 