/**
 * @file fluid_and_polygons.cpp
 * @brief Test scenario combining a tank of fluid particles and polygon obstacles.
 *
 * This scenario creates the four static boundary walls, spawns a grid-based tank
 * of fluid particles (with SPH properties), and adds several randomly shaped polygon
 * obstacles into the interior. The obstacles are solid and interact with the fluid.
 */

#include <random>
#include <cmath>
#include <ctime>
#include <iostream>
#include <vector>

#include "nbody/scenarios/fluid_and_polygons.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sph.hpp"
#include "nbody/math/polygon.hpp"

// --- Constants for the fluid portion ---
static constexpr int    KFluidParticleCount   = 1000;
static constexpr double KFluidParticleMass    = 20.0;
static constexpr double KFluidRestDensity     = 1000.0;
static constexpr double KFluidStaticFriction  = 0.0;
static constexpr double KFluidDynamicFriction = 0.0;
static constexpr double KFluidSmoothingLength = 0.06;
static constexpr double KSpeedOfSound         = 1000.0;

// --- Constants for the polygon obstacles ---
static constexpr int    KObstacleCount        = 5;
static constexpr double KObstacleMinSize      = 0.2;
static constexpr double KObstacleMaxSize      = 0.4;
static constexpr double KObstacleMass         = 5.0;
static constexpr double KObstacleStaticFriction  = 0.3;
static constexpr double KObstacleDynamicFriction = 0.1;

// --- Constants for boundary walls ---
static constexpr double KWallThickness        = 0.1;
static constexpr double KWallMass             = 1e30;


//------------------------------------------------------------------------------
// Helper: Build a random polygon
PolygonShape buildRandomPolygon2(std::default_random_engine &gen, double sz)
{
    PolygonShape poly;
    poly.type = Components::ShapeType::Polygon;
    
    std::uniform_int_distribution<int> sideDist(3, 7);
    std::uniform_real_distribution<double> radiusDist(0.5 * sz, sz);
    
    int const sides = sideDist(gen);
    double const angleStep = 2.0 * M_PI / static_cast<double>(sides);
    
    double angle = 0.0;
    for (int i = 0; i < sides; i++) {
        double const r = radiusDist(gen);
        double const x = r * std::cos(angle);
        double const y = r * std::sin(angle);
        poly.vertices.emplace_back(x, y);
        angle += angleStep;
    }
    return poly;
}

//------------------------------------------------------------------------------
// Helper: Calculate the moment of inertia for a polygon with uniform density.
double calculatePolygonInertia2(const std::vector<Vector>& vertices, double mass)
{
    double numerator = 0.0;
    double denominator = 0.0;
    int const n = vertices.size();
    for (int i = 0; i < n; i++) {
        int const j = (i + 1) % n;
        double const cross = vertices[i].cross(vertices[j]);
        numerator += cross * (vertices[i].dotProduct(vertices[i]) +
                              vertices[i].dotProduct(vertices[j]) +
                              vertices[j].dotProduct(vertices[j]));
        denominator += cross;
    }
    // For uniform density polygon
    return (mass * numerator) / (6.0 * denominator);
}

//------------------------------------------------------------------------------
// Helper: Create a static boundary wall with infinite mass.
// Similar to the implementation in simple_fluid.cpp.
static void makeBoundaryWall(entt::registry &registry,
                             double cx, double cy,
                             double halfW, double halfH)
{
    auto wallEnt = registry.create();
    registry.emplace<Components::Position>(wallEnt, cx, cy);
    registry.emplace<Components::Velocity>(wallEnt, 0.0, 0.0);
    registry.emplace<Components::Mass>(wallEnt, KWallMass);
    registry.emplace<Components::Boundary>(wallEnt);
    registry.emplace<Components::ParticlePhase>(wallEnt, Components::Phase::Solid);
    
    // Mark the wall as asleep to avoid unnecessary updates.
    auto &sleepC = registry.emplace<Components::Sleep>(wallEnt);
    sleepC.asleep = true;
    sleepC.sleepCounter = 9999999;
    
    // Set wall material (friction values can be tuned if needed)
    registry.emplace<Components::Material>(wallEnt, KFluidStaticFriction, KFluidDynamicFriction);
    
    // Build a rectangle polygon shape with CCW vertices.
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

//------------------------------------------------------------------------------
// FluidAndPolygonsScenario::getConfig
ScenarioConfig FluidAndPolygonsScenario::getConfig() const
{
    ScenarioConfig cfg;
    cfg.MetersPerPixel = 1e-2;
    cfg.UniverseSizeMeters = SimulatorConstants::ScreenLength * cfg.MetersPerPixel;
    cfg.SecondsPerTick = 1.0 / SimulatorConstants::StepsPerSecond;
    cfg.TimeAcceleration = 1.0;
    cfg.GridSize = 50;
    cfg.CellSizePixels = static_cast<double>(SimulatorConstants::ScreenLength) / cfg.GridSize;
    
    // The fluid particle count is set here (obstacles are added in addition).
    cfg.ParticleCount = KFluidParticleCount;
    cfg.ParticleMassMean = KFluidParticleMass;
    cfg.ParticleMassStdDev = 0.0;
    cfg.GravitationalSoftener = 0.0;
    cfg.CollisionCoeffRestitution = 0.0; // fluid is near inelastic
    cfg.DragCoeff = 0.0;
    cfg.ParticleDensity = KFluidRestDensity;
    cfg.InitialVelocityFactor = 0.0; // no initial random velocity for fluid
    
    // Enable both fluid and rigid-body systems.
    cfg.activeSystems = {
        Systems::SystemType::FLUID,
        Systems::SystemType::BASIC_GRAVITY,
        Systems::SystemType::COLLISION,
        Systems::SystemType::DAMPENING,
        Systems::SystemType::SLEEP,
        Systems::SystemType::ROTATION,
        Systems::SystemType::MOVEMENT,
        Systems::SystemType::BOUNDARY,
    };
    
    return cfg;
}

//------------------------------------------------------------------------------
// FluidAndPolygonsScenario::createEntities
void FluidAndPolygonsScenario::createEntities(entt::registry &registry) const
{
    std::cout << "FluidAndPolygonScenario" << std::endl;
    // Use the universe size from SimulatorConstants.
    double const universeSizeM = SimulatorConstants::UniverseSizeMeters;
    
    // 1) Create boundary walls.
    double const halfWall = KWallThickness * 0.5;
    // Left wall
    makeBoundaryWall(registry, 0.0, universeSizeM * 0.5, halfWall, universeSizeM * 0.5);
    // Right wall
    makeBoundaryWall(registry, universeSizeM, universeSizeM * 0.5, halfWall, universeSizeM * 0.5);
    // Bottom wall
    makeBoundaryWall(registry, universeSizeM * 0.5, 0.0, universeSizeM * 0.5, halfWall);
    // Top wall
    makeBoundaryWall(registry, universeSizeM * 0.5, universeSizeM, universeSizeM * 0.5, halfWall);
    
    // 2) Spawn fluid particles (as in simple_fluid.cpp) in a rectangular "tank"
    double x_min = universeSizeM * 0.3;
    double x_max = universeSizeM * 0.7;
    double y_min = universeSizeM * 0.3;
    double y_max = universeSizeM * 0.7;
    double regionWidth  = x_max - x_min;
    double regionHeight = y_max - y_min;
    
    int nCols = static_cast<int>(std::sqrt(KFluidParticleCount));
    int nRows = (KFluidParticleCount + nCols - 1) / nCols; // ceiling division
    double dx = regionWidth / (nCols + 1);
    double dy = regionHeight / (nRows + 1);
    
    std::default_random_engine generator{static_cast<unsigned int>(time(nullptr))};
    std::uniform_real_distribution<double> jitterDist(-0.1, 0.1); // ±10% jitter
    
    int count = 0;
    for (int row = 0; row < nRows && count < KFluidParticleCount; row++) {
        for (int col = 0; col < nCols && count < KFluidParticleCount; col++) {
            double jitterX = jitterDist(generator) * dx;
            double jitterY = jitterDist(generator) * dy;
            double x = x_min + (col + 1) * dx + jitterX;
            double y = y_min + (row + 1) * dy + jitterY;
            
            auto e = registry.create();
            registry.emplace<Components::Position>(e, x, y);
            registry.emplace<Components::Velocity>(e, 0.0, 0.0);
            registry.emplace<Components::Mass>(e, KFluidParticleMass);
            registry.emplace<Components::ParticlePhase>(e, Components::Phase::Liquid);
            registry.emplace<Components::Material>(e, KFluidStaticFriction, KFluidDynamicFriction);
            
            // Use a circle shape for the fluid particle.
            double const r = 0.02; 
            registry.emplace<Components::Shape>(e, Components::ShapeType::Circle, r);
            registry.emplace<CircleShape>(e, r);
            
            // Add SPH-related properties.
            registry.emplace<Components::SmoothingLength>(e, KFluidSmoothingLength);
            registry.emplace<Components::SpeedOfSound>(e, KSpeedOfSound);
            registry.emplace<Components::SPHTemp>(e);
            
            // A bluish tint for the fluid particles.
            registry.emplace<Components::Color>(e, 20, 20 + (count % 50), 200 + (count % 55));
            count++;
        }
    }
    std::cerr << "Created " << count << " fluid particles." << std::endl;
    
    // 3) Create polygon obstacles that will interact with the fluid.
    // Place obstacles evenly along a horizontal line in the middle of the fluid tank.
    double obs_y = y_min + regionHeight * 0.5;
    double spacingObstacles = regionWidth / (KObstacleCount + 1);
    
    for (int i = 1; i <= KObstacleCount; i++) {
        double obs_x = x_min + i * spacingObstacles;
        
        // Choose a random size for the obstacle.
        std::uniform_real_distribution<double> sizeDist(KObstacleMinSize, KObstacleMaxSize);
        double size = sizeDist(generator);
        
        // Create a random polygon shape for the obstacle.
        PolygonShape poly = buildRandomPolygon2(generator, size);
        double inertia = calculatePolygonInertia2(poly.vertices, KObstacleMass);
        
        auto entity = registry.create();
        registry.emplace<Components::Position>(entity, obs_x, obs_y);
        registry.emplace<Components::Velocity>(entity, 0.0, 0.0);
        registry.emplace<Components::Mass>(entity, KObstacleMass);
        registry.emplace<Components::ParticlePhase>(entity, Components::Phase::Solid);
        registry.emplace<Components::Material>(entity, KObstacleStaticFriction, KObstacleDynamicFriction);
        
        registry.emplace<Components::Shape>(entity, Components::ShapeType::Polygon, size);
        registry.emplace<PolygonShape>(entity, poly);
        
        // Set the angular properties.
        registry.emplace<Components::AngularPosition>(entity, 0.0);
        registry.emplace<Components::AngularVelocity>(entity, 0.0);
        registry.emplace<Components::Inertia>(entity, inertia);
        
        // Give the obstacle a distinctive color (e.g., reddish).
        registry.emplace<Components::Color>(entity, 200, 50, 50);
        
    }
}