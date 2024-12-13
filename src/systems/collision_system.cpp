#include "systems/collision_system.h"
#include <cmath>
#include <algorithm>
#include "components.h"
#include "simulator_constants.h"

namespace Systems {

static double particle_radius = 1.0; // in meters, adjust as needed
static std::vector<std::vector<std::vector<entt::entity>>> spatialGrid;
static bool initialized = false;

static void initializeGrid() {
    if (!initialized) {
        spatialGrid.resize(SimulatorConstants::GridSize);
        for (auto& row : spatialGrid) {
            row.resize(SimulatorConstants::GridSize);
        }
        initialized = true;
    }
}

static void clearGrid() {
    for (auto& row : spatialGrid) {
        for (auto& cell : row) {
            cell.clear();
        }
    }
}

static void insertIntoGrid(entt::registry& registry) {
    auto view = registry.view<Components::Position>();
    double cell_size_meters = SimulatorConstants::UniverseSizeMeters / SimulatorConstants::GridSize;

    for (auto [entity, pos] : view.each()) {
        int x = static_cast<int>(pos.x / cell_size_meters);
        int y = static_cast<int>(pos.y / cell_size_meters);
        x = std::clamp(x, 0, SimulatorConstants::GridSize - 1);
        y = std::clamp(y, 0, SimulatorConstants::GridSize - 1);
        spatialGrid[x][y].push_back(entity);
    }
}

// -------------------------------------
// Collision Handling Functions
// -------------------------------------

// Solid-Solid Elastic Collision
// Uses a simple elastic collision response, like before.
static void handleSolidSolidCollision(
    Components::Position& posA, Components::Velocity& velA, const Components::Mass& massA,
    Components::Position& posB, Components::Velocity& velB, const Components::Mass& massB,
    double dx, double dy, double dist, double min_dist)
{
    double restitution = 0.9; // for solids
    double nx = dx / dist;
    double ny = dy / dist;

    double relvx = velB.x - velA.x;
    double relvy = velB.y - velA.y;
    double relative_speed = relvx * nx + relvy * ny;

    if (relative_speed < 0) {
        double invMassA = 1.0 / massA.value;
        double invMassB = 1.0 / massB.value;
        double invMassSum = invMassA + invMassB;

        double impulse = -(1.0 + restitution)*relative_speed / invMassSum;

        // Apply impulse
        velA.x -= (impulse * invMassA) * nx;
        velA.y -= (impulse * invMassA) * ny;
        velB.x += (impulse * invMassB) * nx;
        velB.y += (impulse * invMassB) * ny;

        // Positional correction to prevent sinking
        double penetration = min_dist - dist;
        double correctionFactor = 0.5 * penetration / invMassSum;
        posA.x -= correctionFactor * invMassA * nx;
        posA.y -= correctionFactor * invMassA * ny;
        posB.x += correctionFactor * invMassB * nx;
        posB.y += correctionFactor * invMassB * ny;
    }
}

// Solid-Liquid Interaction (Stub)
static void handleSolidLiquidCollision(
    Components::Position& posA, Components::Velocity& velA, const Components::Mass& massA, Components::Phase phaseA,
    Components::Position& posB, Components::Velocity& velB, const Components::Mass& massB, Components::Phase phaseB,
    double dx, double dy, double dist, double min_dist)
{
    // For now, just separate particles with no velocity change (placeholder).
    // Later, implement viscosity, partial penetration, etc.
    double nx = dx / dist;
    double ny = dy / dist;
    double penetration = min_dist - dist;

    posA.x -= 0.5 * penetration * nx;
    posA.y -= 0.5 * penetration * ny;
    posB.x += 0.5 * penetration * nx;
    posB.y += 0.5 * penetration * ny;
}

// Liquid-Liquid Interaction (Stub)
static void handleLiquidLiquidCollision(
    Components::Position& posA, Components::Velocity& velA, const Components::Mass& massA,
    Components::Position& posB, Components::Velocity& velB, const Components::Mass& massB,
    double dx, double dy, double dist, double min_dist)
{
    // Placeholder: simple repulsion or no action.
    // A full fluid simulation would compute pressures and apply forces here.
    double nx = dx / dist;
    double ny = dy / dist;
    double penetration = min_dist - dist;

    // Just separate to prevent overlap
    posA.x -= 0.5 * penetration * nx;
    posA.y -= 0.5 * penetration * ny;
    posB.x += 0.5 * penetration * nx;
    posB.y += 0.5 * penetration * ny;
}

// Gas-Gas Interaction (Stub)
static void handleGasGasCollision(
    Components::Position& posA, Components::Velocity& velA, const Components::Mass& massA,
    Components::Position& posB, Components::Velocity& velB, const Components::Mass& massB,
    double dx, double dy, double dist, double min_dist)
{
    // For gas, maybe do nothing or minimal interaction (particles can pass through?).
    // If you want a hard-sphere gas, similar to solid but with perhaps a lower restitution.
    double restitution = 0.1; // more dissipative?
    double nx = dx / dist;
    double ny = dy / dist;

    double relvx = velB.x - velA.x;
    double relvy = velB.y - velA.y;
    double relative_speed = relvx * nx + relvy * ny;

    if (relative_speed < 0) {
        double invMassA = 1.0 / massA.value;
        double invMassB = 1.0 / massB.value;
        double invMassSum = invMassA + invMassB;

        double impulse = -(1.0 + restitution)*relative_speed / invMassSum;

        velA.x -= (impulse * invMassA) * nx;
        velA.y -= (impulse * invMassA) * ny;
        velB.x += (impulse * invMassB) * nx;
        velB.y += (impulse * invMassB) * ny;
    }
}

// Solid-Gas Interaction (Stub)
static void handleSolidGasCollision(/* same params */ 
    Components::Position& posA, Components::Velocity& velA, const Components::Mass& massA, Components::Phase phaseA,
    Components::Position& posB, Components::Velocity& velB, const Components::Mass& massB, Components::Phase phaseB,
    double dx, double dy, double dist, double min_dist)
{
    // Possibly treat gas as very light particles, with minimal collision response.
    // For now, just do a minimal elastic response similar to gas-gas.
    handleGasGasCollision(posA, velA, massA, posB, velB, massB, dx, dy, dist, min_dist);
}

// Liquid-Gas Interaction (Stub)
static void handleLiquidGasCollision(
    Components::Position& posA, Components::Velocity& velA, const Components::Mass& massA, Components::Phase phaseA,
    Components::Position& posB, Components::Velocity& velB, const Components::Mass& massB, Components::Phase phaseB,
    double dx, double dy, double dist, double min_dist)
{
    // Could implement weaker interactions or partial slip conditions.
    // Placeholder: minimal separation.
    double nx = dx / dist;
    double ny = dy / dist;
    double penetration = min_dist - dist;

    posA.x -= 0.5 * penetration * nx;
    posA.y -= 0.5 * penetration * ny;
    posB.x += 0.5 * penetration * nx;
    posB.y += 0.5 * penetration * ny;
}

// Decide which collision handler to call based on phases
static void handleCollisionByPhase(
    Components::Position& posA, Components::Velocity& velA, const Components::Mass& massA, Components::Phase phaseA,
    Components::Position& posB, Components::Velocity& velB, const Components::Mass& massB, Components::Phase phaseB,
    double dx, double dy, double dist, double min_dist)
{
    // Sort phases so that order of checking does not matter (Solid/Liquid = Liquid/Solid)
    // For simplicity, we'll just do direct checks:
    if (phaseA == Components::Phase::Solid && phaseB == Components::Phase::Solid) {
        handleSolidSolidCollision(posA, velA, massA, posB, velB, massB, dx, dy, dist, min_dist);
    } else if ((phaseA == Components::Phase::Solid && phaseB == Components::Phase::Liquid) ||
               (phaseA == Components::Phase::Liquid && phaseB == Components::Phase::Solid)) {
        handleSolidLiquidCollision(posA, velA, massA, phaseA, posB, velB, massB, phaseB, dx, dy, dist, min_dist);
    } else if (phaseA == Components::Phase::Liquid && phaseB == Components::Phase::Liquid) {
        handleLiquidLiquidCollision(posA, velA, massA, posB, velB, massB, dx, dy, dist, min_dist);
    } else if (phaseA == Components::Phase::Gas && phaseB == Components::Phase::Gas) {
        handleGasGasCollision(posA, velA, massA, posB, velB, massB, dx, dy, dist, min_dist);
    } else if ((phaseA == Components::Phase::Solid && phaseB == Components::Phase::Gas) ||
               (phaseA == Components::Phase::Gas && phaseB == Components::Phase::Solid)) {
        handleSolidGasCollision(posA, velA, massA, phaseA, posB, velB, massB, phaseB, dx, dy, dist, min_dist);
    } else if ((phaseA == Components::Phase::Liquid && phaseB == Components::Phase::Gas) ||
               (phaseA == Components::Phase::Gas && phaseB == Components::Phase::Liquid)) {
        handleLiquidGasCollision(posA, velA, massA, phaseA, posB, velB, massB, phaseB, dx, dy, dist, min_dist);
    }
}

static void resolveCollisions(entt::registry& registry) {
    double min_dist = 2 * particle_radius;

    for (int gx = 0; gx < SimulatorConstants::GridSize; gx++) {
        for (int gy = 0; gy < SimulatorConstants::GridSize; gy++) {
            // Check this cell and neighbors
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    int nx = gx + dx;
                    int ny = gy + dy;
                    if (nx < 0 || nx >= SimulatorConstants::GridSize || 
                        ny < 0 || ny >= SimulatorConstants::GridSize) continue;

                    const auto& cellA = spatialGrid[gx][gy];
                    const auto& cellB = spatialGrid[nx][ny];

                    for (size_t i = 0; i < cellA.size(); i++) {
                        for (size_t j = (gx==nx && gy==ny ? i+1 : 0); j < cellB.size(); j++) {
                            auto eA = cellA[i];
                            auto eB = cellB[j];
                            if (eA == eB) continue;

                            auto& posA = registry.get<Components::Position>(eA);
                            auto& velA = registry.get<Components::Velocity>(eA);
                            auto& massA = registry.get<Components::Mass>(eA);
                            auto& phaseA = registry.get<Components::ParticlePhase>(eA);

                            auto& posB = registry.get<Components::Position>(eB);
                            auto& velB = registry.get<Components::Velocity>(eB);
                            auto& massB = registry.get<Components::Mass>(eB);
                            auto& phaseB = registry.get<Components::ParticlePhase>(eB);

                            double dx = posB.x - posA.x;
                            double dy = posB.y - posA.y;
                            double dist_sq = dx*dx + dy*dy;

                            if (dist_sq < min_dist*min_dist) {
                                double dist = std::sqrt(dist_sq);
                                if (dist < 1e-9) dist = min_dist; // avoid div by zero

                                handleCollisionByPhase(posA, velA, massA, phaseA.phase,
                                                       posB, velB, massB, phaseB.phase,
                                                       dx, dy, dist, min_dist);
                            }
                        }
                    }
                }
            }
        }
    }
}

void CollisionSystem::update(entt::registry& registry) {
    initializeGrid();
    clearGrid();
    insertIntoGrid(registry);
    resolveCollisions(registry);

    // Integration of fluids (SPH/PBF) or other advanced phase handling would go here.
}

} // namespace Systems