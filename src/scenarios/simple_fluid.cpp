/**
 * @file simple_fluid_scenario.cpp
 * @brief A scenario placing a "tank" of fluid particles with four boundary walls.
 */

#include <random>
#include <cmath>
#include <ctime>
#include <iostream>

#include "nbody/scenarios/simple_fluid.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sph.hpp"
#include "nbody/math/polygon.hpp"

/**
 * @brief Helper: create a static boundary wall with infinite mass
 */
static void makeBoundaryWall(entt::registry &registry,
                             double cx, double cy,
                             double halfW, double halfH,
                             double staticFriction, double dynamicFriction,
                             double wallMass)
{
    auto wallEnt = registry.create();
    registry.emplace<Components::Position>(wallEnt, cx, cy);
    registry.emplace<Components::Velocity>(wallEnt, 0.0, 0.0);
    registry.emplace<Components::Mass>(wallEnt, wallMass);
    registry.emplace<Components::Boundary>(wallEnt);

    // Mark it as a "solid" boundary so it doesn't move
    registry.emplace<Components::ParticlePhase>(wallEnt, Components::Phase::Solid);

    // Sleep so we skip certain updates
    auto &sleepC = registry.emplace<Components::Sleep>(wallEnt);
    sleepC.asleep = true;
    sleepC.sleepCounter = 9999999;

    // Material friction (though for fluid, friction may be minimal)
    registry.emplace<Components::Material>(wallEnt, staticFriction, dynamicFriction);

    // Build a rectangle polygon shape for collision walls
    PolygonShape poly;
    poly.type = Components::ShapeType::Polygon;
    poly.vertices.emplace_back(-halfW, -halfH);
    poly.vertices.emplace_back(-halfW,  halfH);
    poly.vertices.emplace_back( halfW,  halfH);
    poly.vertices.emplace_back( halfW, -halfH);

    registry.emplace<Components::Shape>(wallEnt, Components::ShapeType::Polygon, halfH);
    registry.emplace<PolygonShape>(wallEnt, poly);
    registry.emplace<Components::AngularPosition>(wallEnt, 0.0);
}

/**
 * @class SimpleFluidScenario
 * @brief Scenario with a rectangular tank of fluid.
 */
SystemConfig SimpleFluidScenario::getConfig() const
{
    SystemConfig cfg;
    cfg.MetersPerPixel = 1e-2;
    cfg.UniverseSizeMeters = SimulatorConstants::ScreenLength * cfg.MetersPerPixel;
    cfg.SecondsPerTick = 1.0 / SimulatorConstants::StepsPerSecond;
    cfg.TimeAcceleration = 1.0;
    cfg.GridSize = 50;
    cfg.CellSizePixels = static_cast<double>(SimulatorConstants::ScreenLength) / cfg.GridSize;

    cfg.GravitationalSoftener = 0.0;
    cfg.CollisionCoeffRestitution = 0.0; // for fluid, often near inelastic
    cfg.DragCoeff = 0.0;
    cfg.ParticleDensity = 0.5;

    // We enable the "FLUID" system (defined below), plus movement, boundary, etc.
    cfg.activeSystems = {
        Systems::SystemType::FLUID,
        Systems::SystemType::BASIC_GRAVITY,
        Systems::SystemType::DAMPENING,
        Systems::SystemType::SLEEP,
        Systems::SystemType::MOVEMENT,
        Systems::SystemType::BOUNDARY,
    };

    return cfg;
}

void SimpleFluidScenario::createEntities(entt::registry &registry) const
{
    SystemConfig cfg = getConfig();

    double const sizeM = cfg.UniverseSizeMeters;

    // 1) Create bounding walls
    double halfWall = scenarioConfig.wallThickness * 0.5;
    // Left wall
    makeBoundaryWall(registry, 
                     0.0, 
                     sizeM*0.5, 
                     halfWall, 
                     sizeM*0.5, 
                     scenarioConfig.fluidStaticFriction, 
                     scenarioConfig.fluidDynamicFriction,
                     scenarioConfig.wallMass);
    // Right wall
    makeBoundaryWall(registry, 
                     sizeM, 
                     sizeM*0.5, 
                     halfWall, 
                     sizeM*0.5,
                     scenarioConfig.fluidStaticFriction, 
                     scenarioConfig.fluidDynamicFriction,
                     scenarioConfig.wallMass);
    // Bottom wall
    makeBoundaryWall(registry, 
                     sizeM*0.5, 
                     0.0, 
                     sizeM*0.5, 
                     halfWall,
                     scenarioConfig.fluidStaticFriction, 
                     scenarioConfig.fluidDynamicFriction,
                     scenarioConfig.wallMass);
    // Top wall
    makeBoundaryWall(registry, 
                     sizeM*0.5, 
                     sizeM, 
                     sizeM*0.5, 
                     halfWall,
                     scenarioConfig.fluidStaticFriction, 
                     scenarioConfig.fluidDynamicFriction,
                     scenarioConfig.wallMass);

    // 2) Spawn fluid particles using a grid-based layout with a small jitter 
    // to prevent particles from aligning perfectly.
    int numParticles = scenarioConfig.fluidParticleCount;
    double x_min = sizeM * scenarioConfig.fluidRegionMinX;
    double x_max = sizeM * scenarioConfig.fluidRegionMaxX;
    double y_min = sizeM * scenarioConfig.fluidRegionMinY;
    double y_max = sizeM * scenarioConfig.fluidRegionMaxY;
    double regionWidth = x_max - x_min;
    double regionHeight = y_max - y_min;

    // Choose grid resolution based on the particle count.
    // Here we choose the number of columns as the integer square root
    // and compute rows accordingly.
    int nCols = static_cast<int>(std::sqrt(numParticles));
    int nRows = (numParticles + nCols - 1) / nCols; // ceiling division

    // Compute spacing (with some margin from the region boundaries).
    double dx = regionWidth / (nCols + 1);
    double dy = regionHeight / (nRows + 1);

    // Use a small jitter to break the grid symmetry.
    std::default_random_engine generator{static_cast<unsigned int>(time(nullptr))};
    std::uniform_real_distribution<double> jitterDist(-0.1, 0.1);  // ±10% of a grid cell

    int count = 0;
    for (int row = 0; row < nRows && count < numParticles; row++) {
        for (int col = 0; col < nCols && count < numParticles; col++) {
            double jitterX = jitterDist(generator) * dx;
            double jitterY = jitterDist(generator) * dy;
            double x = x_min + (col + 1) * dx + jitterX;
            double y = y_min + (row + 1) * dy + jitterY;

            auto e = registry.create();
            registry.emplace<Components::Position>(e, x, y);
            registry.emplace<Components::Velocity>(e, 0.0, 0.0);
            registry.emplace<Components::Mass>(e, scenarioConfig.fluidParticleMass);
            registry.emplace<Components::ParticlePhase>(e, Components::Phase::Liquid);
            registry.emplace<Components::Material>(e, 
                                                  scenarioConfig.fluidStaticFriction, 
                                                  scenarioConfig.fluidDynamicFriction);

            // Use a circle shape for fluid particles
            double const r = 0.02; 
            registry.emplace<Components::Shape>(e, Components::ShapeType::Circle, r);
            registry.emplace<CircleShape>(e, r);

            // SPH-related properties
            registry.emplace<Components::SmoothingLength>(e, 0.06);
            registry.emplace<Components::SpeedOfSound>(e, 1000.0);
            registry.emplace<Components::SPHTemp>(e);

            // A random bluish color (tinted by count)
            registry.emplace<Components::Color>(e, 20, 20 + (count % 50), 200 + (count % 55));
            count++;
        }
    }
    std::cerr << "Created " << count << " fluid particles in scenario." << std::endl;
}