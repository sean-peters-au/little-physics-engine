#include "systems/grid_thermodynamics_system.h"
#include "simulator_constants.h"
#include <chrono>
#include <iostream>
#include <cmath>

namespace Systems {

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;

// Static grid to avoid reallocation
static GridThermodynamicsSystem::Grid static_grid;

// Cached conversion factors
static double cell_size_meters;
static double cell_volume;
static bool initialized = false;

void GridThermodynamicsSystem::update(entt::registry& registry) {
    // Initialize static values if needed
    if (!initialized) {
        cell_size_meters = SimulatorConstants::UniverseSizeMeters / SimulatorConstants::GridSize;
        cell_volume = cell_size_meters * cell_size_meters * cell_size_meters;
        initialized = true;
    }

    TimePoint start_time = Clock::now();
    
    Grid grid = create_grid();
    auto after_create = Clock::now();
    
    populate_grid(grid, registry);
    auto after_populate = Clock::now();
    
    // Count non-empty cells and their contents
    int non_empty_cells = 0;
    int max_particles_in_cell = 0;
    int total_particles_in_cells = 0;
    for (const auto& row : grid) {
        for (const auto& cell : row) {
            if (cell.particle_count > 0) {
                non_empty_cells++;
                total_particles_in_cells += cell.particle_count;
                max_particles_in_cell = std::max(max_particles_in_cell, static_cast<int>(cell.particle_count));
            }
        }
    }

    calculate_cell_properties(grid);
    auto after_properties = Clock::now();
    
    // Detailed timing for thermodynamics
    {
        auto view = registry.view<Components::Position, Components::Velocity, Components::Mass>();
        TimePoint loop_start = Clock::now();
        int particles_processed = 0;
        int neighbor_checks = 0;
        int moving_particles = 0;
        
        for (auto [entity, pos, vel, mass] : view.each()) {
            particles_processed++;
            
            double vel_sq = vel.x * vel.x + vel.y * vel.y;
            if (vel_sq > 0) {
                moving_particles++;
                int x = static_cast<int>(pos.x / cell_size_meters);
                int y = static_cast<int>(pos.y / cell_size_meters);
                x = std::clamp(x, 0, SimulatorConstants::GridSize-1);
                y = std::clamp(y, 0, SimulatorConstants::GridSize-1);
                
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < SimulatorConstants::GridSize && 
                            ny >= 0 && ny < SimulatorConstants::GridSize) {
                            const auto& cell = grid[nx][ny];
                            if (cell.particle_count > 0) {
                                neighbor_checks++;
                            }
                        }
                    }
                }
            }
        }
    }
    
    apply_thermodynamics(grid, registry);
    auto after_thermodynamics = Clock::now();
}

GridThermodynamicsSystem::Grid GridThermodynamicsSystem::create_grid() {
    // Initialize static grid if needed
    if (static_grid.empty()) {
        static_grid.resize(SimulatorConstants::GridSize);
        for (auto& row : static_grid) {
            row.resize(SimulatorConstants::GridSize);
        }
    }

    // Just clear the cells instead of recreating them
    for (auto& row : static_grid) {
        for (auto& cell : row) {
            cell.clear();
        }
    }
    
    return static_grid;
}

void GridThermodynamicsSystem::populate_grid(Grid& grid, entt::registry& registry) {
    auto view = registry.view<Components::Position, Components::Mass>();
    
    for (auto [entity, pos, mass] : view.each()) {
        // Map position to grid cell using meters
        int x = static_cast<int>(pos.x / cell_size_meters);
        int y = static_cast<int>(pos.y / cell_size_meters);
        
        x = std::clamp(x, 0, SimulatorConstants::GridSize-1);
        y = std::clamp(y, 0, SimulatorConstants::GridSize-1);
        
        auto& cell = grid[x][y];
        cell.total_mass += mass.value;
        cell.add_particle(entity);
    }
}

void GridThermodynamicsSystem::calculate_cell_properties(Grid& grid) {
    for (auto& row : grid) {
        for (auto& cell : row) {
            if (cell.particle_count > 0) {
                // Calculate density using cached cell volume
                cell.density = cell.total_mass / cell_volume;
                
                // Calculate temperature based on density and particle count
                // More particles = more collisions = higher temperature
                cell.temperature = 273.15 + (cell.density * cell.particle_count * 0.1);
            }
        }
    }
}

void GridThermodynamicsSystem::apply_thermodynamics(const Grid& grid, entt::registry& registry) {
    auto view = registry.view<Components::Position, Components::Velocity, Components::Mass>();
    
    double time_factor = SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration;
    
    for (auto [entity, pos, vel, mass] : view.each()) {
        // Map position to grid cell using meters
        int x = static_cast<int>(pos.x / cell_size_meters);
        int y = static_cast<int>(pos.y / cell_size_meters);
        
        x = std::clamp(x, 0, SimulatorConstants::GridSize-1);
        y = std::clamp(y, 0, SimulatorConstants::GridSize-1);
        
        // Pre-calculate velocity magnitude once
        double vel_sq = vel.x * vel.x + vel.y * vel.y;
        double speed = std::sqrt(vel_sq);  // Only one sqrt per particle
        
        if (speed > 0) {  // Only process if moving
            // Apply effects from current cell and neighbors
            double total_drag_factor = 1.0;
            
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx >= 0 && nx < SimulatorConstants::GridSize && 
                        ny >= 0 && ny < SimulatorConstants::GridSize) {
                        const auto& cell = grid[nx][ny];
                        if (cell.particle_count > 0) {
                            // Drag increases with density and temperature
                            double effective_drag = SimulatorConstants::DragCoeff * 
                                                cell.density * 
                                                std::sqrt(cell.temperature / 273.15);
                            
                            double drag = effective_drag * vel_sq;
                            total_drag_factor *= std::max(0.0, 1.0 - (drag * time_factor / speed));
                        }
                    }
                }
            }
            
            // Apply accumulated drag factor once
            vel.x *= total_drag_factor;
            vel.y *= total_drag_factor;
        }
    }
}

} 