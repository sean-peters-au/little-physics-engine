/**
 * @file simple_fluid_scenario.cpp
 * @brief A scenario placing a "tank" of fluid particles with four boundary walls.
 */

#include <random>
#include <cmath>
#include <ctime>
#include <iostream>

#include "scenarios/simple_fluid.hpp"
#include "core/constants.hpp"
#include "entities/entity_components.hpp"
#include "math/polygon.hpp"

/**
 * @brief Helper: create a static boundary wall with infinite mass
 */
static void makeBoundaryWall(entt::registry &registry,
                             double cx, double cy,
                             double halfW, double halfH,
                             double wallMass,
                             double staticFriction,
                             double dynamicFriction)
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

    // Material friction
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
ScenarioSystemConfig SimpleFluidScenario::getSystemsConfig() const
{
    ScenarioSystemConfig config;
    
    // Configure shared parameters
    config.sharedConfig.MetersPerPixel = 1e-2;
    config.sharedConfig.UniverseSizeMeters = SimulatorConstants::ScreenLength * config.sharedConfig.MetersPerPixel;
    config.sharedConfig.SecondsPerTick = 1.0 / SimulatorConstants::StepsPerSecond;
    config.sharedConfig.TimeAcceleration = 1.0;
    config.sharedConfig.GridSize = 50;
    config.sharedConfig.CellSizePixels = static_cast<double>(SimulatorConstants::ScreenLength) / config.sharedConfig.GridSize;

    config.sharedConfig.GravitationalSoftener = 0.0;
    config.sharedConfig.DragCoeff = 0.0;
    config.sharedConfig.ParticleDensity = scenarioEntityConfig.fluidRestDensity;  // Using config for rest density
    
    return config;
}

void SimpleFluidScenario::createEntities(entt::registry &registry) const
{
    // Obtain scenario configuration so we stay consistent with getConfig().
    ScenarioSystemConfig scenarioSystemConfig = getSystemsConfig();
    SharedSystemConfig sharedConfig = scenarioSystemConfig.sharedConfig;

    double const sizeM = sharedConfig.UniverseSizeMeters;

    // 1) Create bounding walls
    double halfWall = scenarioEntityConfig.wallThickness * 0.5;
    // Left wall
    makeBoundaryWall(registry, 0.0, sizeM*0.5, halfWall, sizeM*0.5,
                     scenarioEntityConfig.wallMass, 
                     scenarioEntityConfig.fluidStaticFriction,
                     scenarioEntityConfig.fluidDynamicFriction);
    // Right wall
    makeBoundaryWall(registry, sizeM, sizeM*0.5, halfWall, sizeM*0.5,
                     scenarioEntityConfig.wallMass,
                     scenarioEntityConfig.fluidStaticFriction,
                     scenarioEntityConfig.fluidDynamicFriction);
    // Bottom wall
    makeBoundaryWall(registry, sizeM*0.5, 0.0, sizeM*0.5, halfWall,
                     scenarioEntityConfig.wallMass,
                     scenarioEntityConfig.fluidStaticFriction,
                     scenarioEntityConfig.fluidDynamicFriction);
    // Top wall
    makeBoundaryWall(registry, sizeM*0.5, sizeM, sizeM*0.5, halfWall,
                     scenarioEntityConfig.wallMass,
                     scenarioEntityConfig.fluidStaticFriction,
                     scenarioEntityConfig.fluidDynamicFriction);

    // 2) Spawn fluid particles using a grid-based layout with a small jitter 
    // to prevent particles from aligning perfectly.
    int numParticles = scenarioEntityConfig.fluidParticleCount;
    
    double x_min = sizeM * scenarioEntityConfig.fluidRegionMinX;
    double x_max = sizeM * scenarioEntityConfig.fluidRegionMaxX;
    double y_min = sizeM * scenarioEntityConfig.fluidRegionMinY;
    double y_max = sizeM * scenarioEntityConfig.fluidRegionMaxY;
    double regionWidth = x_max - x_min;
    double regionHeight = y_max - y_min;

    // Choose grid resolution based on the particle count.
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
            registry.emplace<Components::Mass>(e, scenarioEntityConfig.fluidParticleMass);
            registry.emplace<Components::ParticlePhase>(e, Components::Phase::Liquid);
            registry.emplace<Components::Material>(e, 
                                                  scenarioEntityConfig.fluidStaticFriction, 
                                                  scenarioEntityConfig.fluidDynamicFriction);

            // Use a circle shape for fluid particles
            double const r = 0.02; 
            registry.emplace<Components::Shape>(e, Components::ShapeType::Circle, r);
            registry.emplace<CircleShape>(e, r);

            // SPH-related properties
            registry.emplace<Components::SpeedOfSound>(e, 1000.0);
            registry.emplace<Components::SPHTemp>(e);

            // A random bluish color (tinted by count)
            registry.emplace<Components::Color>(e, 20, 20 + (count % 50), 200 + (count % 55));
            count++;
        }
    }
    std::cerr << "Created " << count << " fluid particles in scenario." << std::endl;
}