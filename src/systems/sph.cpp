#include "nbody/systems/sph.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sph.hpp"
#include "nbody/components/sim.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace Systems {

static bool sphInitialized = false;
static double cellSizeMeters;
static int gridSize;

// Commonly used constants for SPH
static double defaultH = 1e9;         // Example smoothing length in meters, adjust as needed
static double defaultCS = 1000.0;    // Speed of sound in m/s, adjust as needed
static double restDensity = 1e-7;     // Example rest density, tune to your scenario

void SPHSystem::update(entt::registry &registry) {

    // Initialize on first run
    if (!sphInitialized) {
        gridSize = SimulatorConstants::GridSize; 
        cellSizeMeters = SimulatorConstants::UniverseSizeMeters / gridSize;
        sphInitialized = true;
    }

    // Ensure each particle has needed components
    {
        auto view = registry.view<Components::ParticlePhase, Components::Mass, Components::SPHTemp>();
        for (auto [entity, phase, mass, sphTemp] : view.each()) {
            // Already has SPHTemp; ensure smoothing length and c_s
            if (!registry.any_of<Components::SmoothingLength>(entity)) {
                registry.emplace<Components::SmoothingLength>(entity, defaultH);
            }
            if (!registry.any_of<Components::SpeedOfSound>(entity)) {
                registry.emplace<Components::SpeedOfSound>(entity, defaultCS);
            }
        }

        // For particles that might not yet have SPHTemp
        auto particlesView = registry.view<Components::ParticlePhase, Components::Mass>();
        for (auto entity : particlesView) {
            if (!registry.any_of<Components::SPHTemp>(entity)) {
                registry.emplace<Components::SPHTemp>(entity);
            }
            if (!registry.any_of<Components::SmoothingLength>(entity)) {
                registry.emplace<Components::SmoothingLength>(entity, defaultH);
            }
            if (!registry.any_of<Components::SpeedOfSound>(entity)) {
                registry.emplace<Components::SpeedOfSound>(entity, defaultCS);
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
        double const rho = sphTemp.density;
        double const c = c_s.value;
        // offset density by rest_density if you want a stable reference
        sphTemp.pressure = c*c*(rho - restDensity);
        if (sphTemp.pressure < 0.0) {
            sphTemp.pressure = 0.0; // no negative pressure
        }
    }
}

void SPHSystem::computeForces(entt::registry &registry) {
    Grid grid = createGrid();
    populateGrid(grid, registry);

    // Reset accelerations
    auto resetView = registry.view<Components::SPHTemp>();
    for (auto [entity, sphTemp] : resetView.each()) {
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
    // Get simulator state
    const auto& state = registry.get<Components::SimulatorState>(
        registry.view<Components::SimulatorState>().front()
    );

    // Time step in real seconds using simulator state
    double const dt = SimulatorConstants::SecondsPerTick * 
               state.baseTimeAcceleration * state.timeScale;

    // Apply the computed accelerations to velocities
    auto view = registry.view<Components::Velocity, Components::SPHTemp, Components::Mass>();

    for (auto [entity, vel, sphTemp, mass] : view.each()) {
        if (mass.value > 0.0) {
            vel.x += (sphTemp.acc_x * dt);
            vel.y += (sphTemp.acc_y * dt);
        }
    }
}

SPHSystem::Grid SPHSystem::createGrid() {
    Grid grid(gridSize);
    for (int i = 0; i < gridSize; i++) {
        grid[i].resize(gridSize);
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
        int x = static_cast<int>(pos.x / cellSizeMeters);
        int y = static_cast<int>(pos.y / cellSizeMeters);
        x = std::clamp(x, 0, gridSize-1);
        y = std::clamp(y, 0, gridSize-1);
        grid[x][y].particles.push_back(entity);
}
}

void SPHSystem::computeDensityForParticle(entt::entity e, entt::registry &registry, const Grid &grid) {
    auto &pos = registry.get<Components::Position>(e);
    auto &mass = registry.get<Components::Mass>(e);
    auto &sphTemp = registry.get<Components::SPHTemp>(e);
    auto &hComp = registry.get<Components::SmoothingLength>(e);

    double const h = hComp.value;
    double const hSq = h*h;

    // Find neighbors
    std::vector<entt::entity> neighbors;
    findNeighbors(e, registry, grid, neighbors);

    double density = 0.0;
    for (auto n : neighbors) {
        const auto &posN = registry.get<Components::Position>(n);
        const auto &massN = registry.get<Components::Mass>(n);

        double const dx = pos.x - posN.x;
        double const dy = pos.y - posN.y;
        double const rSq = dx*dx + dy*dy;
        if (rSq < hSq) {
            double const r = std::sqrt(rSq);
            density += massN.value * W_poly6(r, h);
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
    const auto &cS = registry.get<Components::SpeedOfSound>(e);

    double const h = hComp.value;
    double const hSq = h*h;

    std::vector<entt::entity> neighbors;
    findNeighbors(e, registry, grid, neighbors);

    double accX = 0.0;
    double accY = 0.0;

    double const rhoI = sphTemp.density;
    double const pI = sphTemp.pressure;

    for (auto n : neighbors) {
        if (n == e) { continue; // skip self
}

        const auto &posN = registry.get<Components::Position>(n);
        const auto &velN = registry.get<Components::Velocity>(n);
        const auto &massN = registry.get<Components::Mass>(n);
        const auto &sphTempN = registry.get<Components::SPHTemp>(n);

        double const dx = pos.x - posN.x;
        double const dy = pos.y - posN.y;
        double const rSq = dx*dx + dy*dy;
        if (rSq < hSq && rSq > 1e-12) {
            double const r = std::sqrt(rSq);
            double const rhoJ = sphTempN.density;
            double const pJ = sphTempN.pressure;

            // Pressure force
            double const gradW = gradW_spiky(r, h);
            double const common = massN.value * (pI/(rhoI*rhoI) + pJ/(rhoJ*rhoJ));

            double const nx = dx / r;
            double const ny = dy / r;

            accX -= common * gradW * nx;
            accY -= common * gradW * ny;

            // Artificial viscosity
            double const velX = vel.x - velN.x;
            double const velY = vel.y - velN.y;
            double const mu = (h * (velX * nx + velY * ny)) / (rSq + 0.01*hSq);
            if (mu < 0.0) {
                double const c = std::max(cS.value, cS.value); // Just c_s for both
                double const rhoAvg = 0.5*(rhoI + rhoJ);
                double const visc = (-alpha * c * mu + beta * mu*mu) / rhoAvg;
                accX -= massN.value * visc * gradW * nx;
                accY -= massN.value * visc * gradW * ny;
            }
        }
    }

    // Assign accelerations back to SPHTemp
    auto &sphTempMut = registry.get<Components::SPHTemp>(e);
    sphTempMut.acc_x += accX;
    sphTempMut.acc_y += accY;
}

// Kernel functions
double SPHSystem::W_poly6(double r, double h) {
    double const alpha = 315.0/(64.0*M_PI*std::pow(h,9));
    double const diff = (h*h - r*r);
    return diff > 0.0 ? alpha * diff*diff*diff : 0.0;
}

double SPHSystem::gradW_spiky(double r, double h) {
    double const beta = -45.0/(M_PI*std::pow(h,6));
    return (r > 0 && r < h) ? beta * ( (h-r)*(h-r) ) : 0.0;
}

double SPHSystem::lapW_visc(double r, double h) {
    double const gamma = 45.0/(M_PI*std::pow(h,6));
    return (r < h) ? gamma*(h - r) : 0.0;
}

void SPHSystem::findNeighbors(entt::entity e, entt::registry &registry, const Grid &grid,
                              std::vector<entt::entity> &neighbors) {
    const auto &pos = registry.get<Components::Position>(e);
    const auto &hComp = registry.get<Components::SmoothingLength>(e);
    double const h = hComp.value;

    int const cellRange = static_cast<int>(std::ceil(h / cellSizeMeters));

    int const x = static_cast<int>(pos.x / cellSizeMeters);
    int const y = static_cast<int>(pos.y / cellSizeMeters);

    for (int dx = -cellRange; dx <= cellRange; dx++) {
        for (int dy = -cellRange; dy <= cellRange; dy++) {
            int const nx = x + dx;
            int const ny = y + dy;
            if (nx < 0 || nx >= gridSize || ny < 0 || ny >= gridSize) { continue;
}
            for (const auto &p : grid[nx][ny].particles) {
                neighbors.push_back(p);
            }
        }
    }
}

}