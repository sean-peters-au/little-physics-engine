/**
 * @file simple_fluid_scenario.cpp
 * @brief A scenario placing a "tank" of fluid particles with four boundary walls.
 */

#include "nbody/scenarios/simple_fluid.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sph.hpp"
#include "nbody/math/polygon.hpp"
#include <random>
#include <cmath>
#include <ctime>
#include <iostream>

static constexpr int    KFluidParticleCount = 500;    // number of fluid particles
static constexpr double KFluidRestDensity   = 1000.0; // typical for water (kg/m^3 in a scaled sense)
static constexpr double KFluidParticleMass  = 1.0;    // you can scale this as needed

static constexpr double KWallThickness      = 0.1;    // thickness of bounding walls
static constexpr double KWallMass           = 1e30;   // effectively infinite

static constexpr double KFluidStaticFriction   = 0.0; 
static constexpr double KFluidDynamicFriction  = 0.0;

/**
 * @brief Helper: create a static boundary wall with infinite mass
 */
static void makeBoundaryWall(entt::registry &registry,
                             double cx, double cy,
                             double halfW, double halfH)
{
    auto wallEnt = registry.create();
    registry.emplace<Components::Position>(wallEnt, cx, cy);
    registry.emplace<Components::Velocity>(wallEnt, 0.0, 0.0);
    registry.emplace<Components::Mass>(wallEnt, KWallMass);
    registry.emplace<Components::Boundary>(wallEnt);

    // Mark it as a "solid" boundary so it doesn't move
    registry.emplace<Components::ParticlePhase>(wallEnt, Components::Phase::Solid);

    // Sleep so we skip certain updates
    auto &sleepC = registry.emplace<Components::Sleep>(wallEnt);
    sleepC.asleep = true;
    sleepC.sleepCounter = 9999999;

    // Material friction (though for fluid, friction may be minimal)
    registry.emplace<Components::Material>(wallEnt, KFluidStaticFriction, KFluidDynamicFriction);

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
ScenarioConfig SimpleFluidScenario::getConfig() const
{
    ScenarioConfig cfg;
    cfg.MetersPerPixel = 1e-2;
    cfg.UniverseSizeMeters = SimulatorConstants::ScreenLength * cfg.MetersPerPixel;
    cfg.SecondsPerTick = 1.0 / SimulatorConstants::StepsPerSecond;
    cfg.TimeAcceleration = 1.0;
    cfg.GridSize = 50;
    cfg.CellSizePixels = static_cast<double>(SimulatorConstants::ScreenLength) / cfg.GridSize;

    cfg.ParticleCount = KFluidParticleCount;
    cfg.ParticleMassMean = KFluidParticleMass;
    cfg.ParticleMassStdDev = 0.0; 
    cfg.GravitationalSoftener = 0.0;
    cfg.CollisionCoeffRestitution = 0.0; // for fluid, often near inelastic
    cfg.DragCoeff = 0.0;
    cfg.ParticleDensity = KFluidRestDensity;
    cfg.InitialVelocityFactor = 0.0; // no initial random velocity

    // We enable the "FLUID" system (defined below), plus movement, boundary, etc.
    cfg.activeSystems = {
        Systems::SystemType::MOVEMENT,
        Systems::SystemType::FLUID,   // <-- our custom fluid solver
        Systems::SystemType::BOUNDARY,
        // Potentially also SLEEP, or DAMPENING, or collisions, etc.
    };

    return cfg;
}

void SimpleFluidScenario::createEntities(entt::registry &registry) const
{
    double const sizeM = SimulatorConstants::UniverseSizeMeters;

    // 1) Create bounding walls
    double halfWall = KWallThickness * 0.5;
    // Left wall
    makeBoundaryWall(registry, 0.0, sizeM*0.5, halfWall, sizeM*0.5);
    // Right wall
    makeBoundaryWall(registry, sizeM, sizeM*0.5, halfWall, sizeM*0.5);
    // Bottom wall
    makeBoundaryWall(registry, sizeM*0.5, 0.0, sizeM*0.5, halfWall);
    // Top wall
    makeBoundaryWall(registry, sizeM*0.5, sizeM, sizeM*0.5, halfWall);

    // 2) Spawn fluid particles in a block in the lower half of the container
    std::default_random_engine generator{static_cast<unsigned int>(time(nullptr))};
    std::uniform_real_distribution<double> xDist(sizeM*0.2, sizeM*0.8);
    std::uniform_real_distribution<double> yDist(sizeM*0.2, sizeM*0.5);

    for(int i = 0; i < KFluidParticleCount; i++) {
        auto e = registry.create();

        // Position in a random region
        double x = xDist(generator);
        double y = yDist(generator);
        registry.emplace<Components::Position>(e, x, y);
        registry.emplace<Components::Velocity>(e, 0.0, 0.0);

        // Mass, phase, shape
        registry.emplace<Components::Mass>(e, KFluidParticleMass);
        registry.emplace<Components::ParticlePhase>(e, Components::Phase::Liquid);

        // Minimal friction
        registry.emplace<Components::Material>(e, KFluidStaticFriction, KFluidDynamicFriction);

        // We'll treat fluid as small circles for collision
        double const r = 0.02; 
        registry.emplace<Components::Shape>(e, Components::ShapeType::Circle, r);
        registry.emplace<CircleShape>(e, r);

        // For SPH computations, store smoothing length etc. 
        registry.emplace<Components::SmoothingLength>(e, 0.06); // example smoothing length
        registry.emplace<Components::SpeedOfSound>(e, 1000.0);  // speed of sound (for pressure eq.)
        registry.emplace<Components::SPHTemp>(e);               // to store density/pressure each frame

        // A random color tinted bluish
        registry.emplace<Components::Color>(e, 20, 20 + (i % 50), 200 + (i % 55));
    }
    std::cerr << "Created " << KFluidParticleCount << " fluid particles in scenario." << std::endl;
}