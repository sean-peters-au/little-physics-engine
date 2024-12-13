#include "nbody/systems/sph.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sph.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace Systems {

static bool sph_initialized = false;
static double cell_size_meters;
static int grid_size;

// Commonly used constants for SPH
static double default_h = 1e9;         // Example smoothing length in meters, adjust as needed
static double default_c_s = 1000.0;    // Speed of sound in m/s, adjust as needed
static double rest_density = 1e-7;     // Example rest density, tune to your scenario

void SPHSystem::update(entt::registry &registry) {
    // Initialize on first run
    if (!sph_initialized) {
        grid_size = SimulatorConstants::GridSize; 
        cell_size_meters = SimulatorConstants::UniverseSizeMeters / grid_size;
        sph_initialized = true;
    }

    // Ensure each particle has needed components
    {
        auto view = registry.view<Components::ParticlePhase, Components::Mass, Components::SPHTemp>();
        for (auto [entity, phase, mass, sphTemp] : view.each()) {
            // Already has SPHTemp; ensure smoothing length and c_s
            if (!registry.any_of<Components::SmoothingLength>(entity)) {
                registry.emplace<Components::SmoothingLength>(entity, default_h);
            }
            if (!registry.any_of<Components::SpeedOfSound>(entity)) {
                registry.emplace<Components::SpeedOfSound>(entity, default_c_s);
            }
        }

        // For particles that might not yet have SPHTemp
        auto particles_view = registry.view<Components::ParticlePhase, Components::Mass>();
        for (auto entity : particles_view) {
            if (!registry.any_of<Components::SPHTemp>(entity)) {
                registry.emplace<Components::SPHTemp>(entity);
            }
            if (!registry.any_of<Components::SmoothingLength>(entity)) {
                registry.emplace<Components::SmoothingLength>(entity, default_h);
            }
            if (!registry.any_of<Components::SpeedOfSound>(entity)) {
                registry.emplace<Components::SpeedOfSound>(entity, default_c_s);
            }
        }
    }

    // Compute densities, pressures, and forces
    computeDensity(registry);
    computePressure(registry);
    computeForces(registry);
    applyForces(registry);
}

void SPHSystem::computeDensity(entt::registry &registry) {
    Grid grid = createGrid();
    populateGrid(grid, registry);

    // Compute density for each particle
    auto view = registry.view<Components::ParticlePhase, Components::Position, Components::SPHTemp, Components::SmoothingLength>();
    for (auto [entity, phase, pos, sphTemp, hComp] : view.each()) {
        if (phase.phase == Components::Phase::Gas) {
            computeDensityForParticle(entity, registry, grid);
        } else {
            // Non-gas particles: just leave density as is or set minimal
            sphTemp.density = 0.0;
        }
    }
}

void SPHSystem::computePressure(entt::registry &registry) {
    // Simple isothermal EOS: P = c_s^2 (ρ - ρ0)
    auto view = registry.view<Components::SPHTemp, Components::SpeedOfSound>();
    for (auto [entity, sphTemp, c_s] : view.each()) {
        double rho = sphTemp.density;
        double c = c_s.value;
        // offset density by rest_density if you want a stable reference
        sphTemp.pressure = c*c*(rho - rest_density);
        if (sphTemp.pressure < 0.0) {
            sphTemp.pressure = 0.0; // no negative pressure
        }
    }
}

void SPHSystem::computeForces(entt::registry &registry) {
    Grid grid = createGrid();
    populateGrid(grid, registry);

    // Reset accelerations
    auto reset_view = registry.view<Components::SPHTemp>();
    for (auto [entity, sphTemp] : reset_view.each()) {
        sphTemp.acc_x = 0.0;
        sphTemp.acc_y = 0.0;
    }

    auto view = registry.view<Components::ParticlePhase, Components::Position, Components::Velocity, Components::Mass, Components::SPHTemp, Components::SmoothingLength, Components::SpeedOfSound>();
    for (auto [entity, phase, pos, vel, mass, sphTemp, hComp, c_s] : view.each()) {
        if (phase.phase == Components::Phase::Gas) {
            computeForcesForParticle(entity, registry, grid);
        }
    }
}

void SPHSystem::applyForces(entt::registry &registry) {
    // Apply the computed accelerations to velocities
    auto view = registry.view<Components::Velocity, Components::SPHTemp, Components::Mass>();
    double dt = SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration;
    for (auto [entity, vel, sphTemp, mass] : view.each()) {
        if (mass.value > 0.0) {
            vel.x += (sphTemp.acc_x * dt);
            vel.y += (sphTemp.acc_y * dt);
        }
    }
}

SPHSystem::Grid SPHSystem::createGrid() {
    Grid grid(grid_size);
    for (int i = 0; i < grid_size; i++) {
        grid[i].resize(grid_size);
    }
    for (auto &row : grid) {
        for (auto &cell : row) {
            cell.clear();
        }
    }
    return grid;
}

void SPHSystem::populateGrid(Grid &grid, entt::registry &registry) {
    auto view = registry.view<Components::Position, Components::Mass, Components::ParticlePhase, Components::SPHTemp>();
    for (auto [entity, pos, mass, phase, sphTemp] : view.each()) {
        int x = static_cast<int>(pos.x / cell_size_meters);
        int y = static_cast<int>(pos.y / cell_size_meters);
        x = std::clamp(x, 0, grid_size-1);
        y = std::clamp(y, 0, grid_size-1);
        grid[x][y].particles.push_back(entity);
}
}

void SPHSystem::computeDensityForParticle(entt::entity e, entt::registry &registry, const Grid &grid) {
    auto &pos = registry.get<Components::Position>(e);
    auto &mass = registry.get<Components::Mass>(e);
    auto &sphTemp = registry.get<Components::SPHTemp>(e);
    auto &hComp = registry.get<Components::SmoothingLength>(e);

    double h = hComp.value;
    double h_sq = h*h;

    // Find neighbors
    std::vector<entt::entity> neighbors;
    findNeighbors(e, registry, grid, neighbors);

    double density = 0.0;
    for (auto n : neighbors) {
        const auto &pos_n = registry.get<Components::Position>(n);
        const auto &mass_n = registry.get<Components::Mass>(n);

        double dx = pos.x - pos_n.x;
        double dy = pos.y - pos_n.y;
        double r_sq = dx*dx + dy*dy;
        if (r_sq < h_sq) {
            double r = std::sqrt(r_sq);
            density += mass_n.value * W_poly6(r, h);
        }
    }
    sphTemp.density = density;
}

void SPHSystem::computeForcesForParticle(entt::entity e, entt::registry &registry, const Grid &grid) {
    const auto &pos = registry.get<Components::Position>(e);
    auto &vel = registry.get<Components::Velocity>(e);
    const auto &mass = registry.get<Components::Mass>(e);
    const auto &sphTemp = registry.get<Components::SPHTemp>(e);
    const auto &hComp = registry.get<Components::SmoothingLength>(e);
    const auto &c_s = registry.get<Components::SpeedOfSound>(e);

    double h = hComp.value;
    double h_sq = h*h;

    std::vector<entt::entity> neighbors;
    findNeighbors(e, registry, grid, neighbors);

    double acc_x = 0.0;
    double acc_y = 0.0;

    double rho_i = sphTemp.density;
    double P_i = sphTemp.pressure;

    for (auto n : neighbors) {
        if (n == e) continue; // skip self

        const auto &pos_n = registry.get<Components::Position>(n);
        const auto &vel_n = registry.get<Components::Velocity>(n);
        const auto &mass_n = registry.get<Components::Mass>(n);
        const auto &sphTemp_n = registry.get<Components::SPHTemp>(n);

        double dx = pos.x - pos_n.x;
        double dy = pos.y - pos_n.y;
        double r_sq = dx*dx + dy*dy;
        if (r_sq < h_sq && r_sq > 1e-12) {
            double r = std::sqrt(r_sq);
            double rho_j = sphTemp_n.density;
            double P_j = sphTemp_n.pressure;

            // Pressure force
            double gradW = gradW_spiky(r, h);
            double common = mass_n.value * (P_i/(rho_i*rho_i) + P_j/(rho_j*rho_j));

            double nx = dx / r;
            double ny = dy / r;

            acc_x -= common * gradW * nx;
            acc_y -= common * gradW * ny;

            // Artificial viscosity
            double vel_x = vel.x - vel_n.x;
            double vel_y = vel.y - vel_n.y;
            double mu = (h * (vel_x * nx + vel_y * ny)) / (r_sq + 0.01*h_sq);
            if (mu < 0.0) {
                double c = std::max(c_s.value, c_s.value); // Just c_s for both
                double rho_avg = 0.5*(rho_i + rho_j);
                double visc = (-alpha * c * mu + beta * mu*mu) / rho_avg;
                acc_x -= mass_n.value * visc * gradW * nx;
                acc_y -= mass_n.value * visc * gradW * ny;
            }
        }
    }

    // Assign accelerations back to SPHTemp
    auto &sphTemp_mut = registry.get<Components::SPHTemp>(e);
    sphTemp_mut.acc_x += acc_x;
    sphTemp_mut.acc_y += acc_y;
}

// Kernel functions
double SPHSystem::W_poly6(double r, double h) {
    double alpha = 315.0/(64.0*M_PI*std::pow(h,9));
    double diff = (h*h - r*r);
    return diff > 0.0 ? alpha * diff*diff*diff : 0.0;
}

double SPHSystem::gradW_spiky(double r, double h) {
    double beta = -45.0/(M_PI*std::pow(h,6));
    return (r > 0 && r < h) ? beta * ( (h-r)*(h-r) ) : 0.0;
}

double SPHSystem::lapW_visc(double r, double h) {
    double gamma = 45.0/(M_PI*std::pow(h,6));
    return (r < h) ? gamma*(h - r) : 0.0;
}

void SPHSystem::findNeighbors(entt::entity e, entt::registry &registry, const Grid &grid,
                              std::vector<entt::entity> &neighbors) {
    const auto &pos = registry.get<Components::Position>(e);
    const auto &hComp = registry.get<Components::SmoothingLength>(e);
    double h = hComp.value;

    int cell_range = static_cast<int>(std::ceil(h / cell_size_meters));

    int x = static_cast<int>(pos.x / cell_size_meters);
    int y = static_cast<int>(pos.y / cell_size_meters);

    for (int dx = -cell_range; dx <= cell_range; dx++) {
        for (int dy = -cell_range; dy <= cell_range; dy++) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx < 0 || nx >= grid_size || ny < 0 || ny >= grid_size) continue;
            for (auto &p : grid[nx][ny].particles) {
                neighbors.push_back(p);
            }
        }
    }
}

}