#include "nbody/systems/thermodynamics.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/components/sim.hpp"
#include <chrono>
#include <iostream>
#include <cmath>

namespace Systems {

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;

// Static grid to avoid reallocation
static GridThermodynamicsSystem::Grid staticGrid;

// Cached conversion factors
static double cellSizeMeters;
static double cellVolume;
static bool initialized = false;

void GridThermodynamicsSystem::update(entt::registry& registry) {
    // Get simulator state
    const auto& state = registry.get<Components::SimulatorState>(
        registry.view<Components::SimulatorState>().front()
    );

    // Initialize static values if needed
    if (!initialized) {
        cellSizeMeters = SimulatorConstants::UniverseSizeMeters / SimulatorConstants::GridSize;
        cellVolume = cellSizeMeters * cellSizeMeters * cellSizeMeters;
        initialized = true;
    }

    TimePoint const startTime = Clock::now();
    
    Grid grid = create_grid();
    auto afterCreate = Clock::now();
    
    populate_grid(grid, registry);
    auto afterPopulate = Clock::now();
    
    // Count non-empty cells and their contents
    int nonEmptyCells = 0;
    int maxParticlesInCell = 0;
    int totalParticlesInCells = 0;
    for (const auto& row : grid) {
        for (const auto& cell : row) {
            if (cell.particle_count > 0) {
                nonEmptyCells++;
                totalParticlesInCells += cell.particle_count;
                maxParticlesInCell = std::max(maxParticlesInCell, static_cast<int>(cell.particle_count));
            }
        }
    }

    calculate_cell_properties(grid);
    auto afterProperties = Clock::now();
    
    // Detailed timing for thermodynamics
    {
        auto view = registry.view<Components::Position, Components::Velocity, Components::Mass>();
        TimePoint const loopStart = Clock::now();
        int particlesProcessed = 0;
        int neighborChecks = 0;
        int movingParticles = 0;
        
        // Time step in real seconds using simulator state
        double const dt = SimulatorConstants::SecondsPerTick * 
                   state.baseTimeAcceleration * state.timeScale;
        
        for (auto [entity, pos, vel, mass] : view.each()) {
            particlesProcessed++;
            
            double const velSq = vel.x * vel.x + vel.y * vel.y;
            if (velSq > 0) {
                movingParticles++;
                int x = static_cast<int>(pos.x / cellSizeMeters);
                int y = static_cast<int>(pos.y / cellSizeMeters);
                x = std::clamp(x, 0, SimulatorConstants::GridSize-1);
                y = std::clamp(y, 0, SimulatorConstants::GridSize-1);
                
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        int const nx = x + dx;
                        int const ny = y + dy;
                        if (nx >= 0 && nx < SimulatorConstants::GridSize && 
                            ny >= 0 && ny < SimulatorConstants::GridSize) {
                            const auto& cell = grid[nx][ny];
                            if (cell.particle_count > 0) {
                                neighborChecks++;
                            }
                        }
                    }
                }
            }
        }
    }
    
    apply_thermodynamics(grid, registry);
    auto afterThermodynamics = Clock::now();
}

GridThermodynamicsSystem::Grid GridThermodynamicsSystem::create_grid() {
    // Initialize static grid if needed
    if (staticGrid.empty()) {
        staticGrid.resize(SimulatorConstants::GridSize);
        for (auto& row : staticGrid) {
            row.resize(SimulatorConstants::GridSize);
        }
    }

    // Just clear the cells instead of recreating them
    for (auto& row : staticGrid) {
        for (auto& cell : row) {
            cell.clear();
        }
    }
    
    return staticGrid;
}

void GridThermodynamicsSystem::populate_grid(Grid& grid, entt::registry& registry) {
    auto view = registry.view<Components::Position, Components::Mass>();
    
    for (auto [entity, pos, mass] : view.each()) {
        // Map position to grid cell using meters
        int x = static_cast<int>(pos.x / cellSizeMeters);
        int y = static_cast<int>(pos.y / cellSizeMeters);
        
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
                cell.density = cell.total_mass / cellVolume;
                
                // Calculate temperature based on density and particle count
                // More particles = more collisions = higher temperature
                cell.temperature = 273.15 + (cell.density * cell.particle_count * 0.1);
            }
        }
    }
}

void GridThermodynamicsSystem::apply_thermodynamics(const Grid& grid, entt::registry& registry) {
    // TODO(speters): 
}

} 