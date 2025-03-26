/**
 * @file planetary_ocean.cpp
 * @brief Implementation of a scenario with a planetary body and fluid oceans
 */

#include <random>
#include <cmath>
#include <ctime>
#include <iostream>

#include "scenarios/planetary_ocean.hpp"
#include "core/constants.hpp"
#include "entities/entity_components.hpp"
#include "math/polygon.hpp"

ScenarioSystemConfig PlanetaryOceanScenario::getSystemsConfig() const
{
    ScenarioSystemConfig config;
    
    // Planet size and scaling - make planet smaller on screen to emphasize fluid
    double planetRadiusPixels = 70.0;  // 140 pixel diameter
    config.sharedConfig.MetersPerPixel = scenarioEntityConfig.planetRadius / planetRadiusPixels;
    config.sharedConfig.UniverseSizeMeters = SimulatorConstants::ScreenLength * config.sharedConfig.MetersPerPixel;
    
    // Time settings
    config.sharedConfig.SecondsPerTick = 1.0;
    config.sharedConfig.TimeAcceleration = 10.0;
    
    // Grid settings
    config.sharedConfig.GridSize = 100;
    config.sharedConfig.CellSizePixels = static_cast<double>(SimulatorConstants::ScreenLength) / config.sharedConfig.GridSize;

    // Physical parameters
    config.sharedConfig.GravitationalSoftener = scenarioEntityConfig.planetRadius * 0.01;
    config.sharedConfig.CollisionCoeffRestitution = 0.1;
    config.sharedConfig.DragCoeff = 0.0;
    config.sharedConfig.ParticleDensity = scenarioEntityConfig.fluidRestDensity;

    // Configure fluid parameters specifically for planetary scale
    config.fluidConfig.gravity = 9.81f;                // Standard Earth gravity
    config.fluidConfig.restDensity = 0.5f;             // Lower density for this scenario
    config.fluidConfig.stiffness = 10.0f;              // Lower stiffness for planetary scale
    config.fluidConfig.viscosity = 0.02f;              // Lower viscosity for ocean
    
    // Fluid position solver parameters
    config.fluidConfig.positionSolver.safetyMargin = 0.01f;
    config.fluidConfig.positionSolver.relaxFactor = 0.95f;
    config.fluidConfig.positionSolver.maxCorrection = 0.2f;
    config.fluidConfig.positionSolver.minPositionChange = 1e-4f;
    
    // Fluid impulse solver parameters
    config.fluidConfig.impulseSolver.buoyancyStrength = 0.5f;
    config.fluidConfig.impulseSolver.fluidForceScale = 200.0f;
    config.fluidConfig.impulseSolver.depthScale = 0.1f;

    // Grid parameters for planetary scale simulation
    config.fluidConfig.gridConfig.gridEpsilon = 1e-8f;            // Smaller epsilon for planetary scale
    config.fluidConfig.gridConfig.defaultSmoothingLength = 5000.0f; // Much larger smoothing length
    config.fluidConfig.gridConfig.boundaryOffset = 100.0f;          // Larger offset from boundaries
    
    // Numerical stability parameters for planetary scale
    config.fluidConfig.numericalConfig.minDistanceThreshold = 1e-10f;
    config.fluidConfig.numericalConfig.minDensityThreshold = 1e-8f;
    
    return config;
}

void PlanetaryOceanScenario::createEntities(entt::registry &registry) const
{
    // Get system configuration
    ScenarioSystemConfig scenarioSystemConfig = getSystemsConfig();
    SharedSystemConfig sharedConfig = scenarioSystemConfig.sharedConfig;
    double const sizeM = sharedConfig.UniverseSizeMeters;
    
    // Center coordinates in meters
    double centerX = sizeM * 0.5;
    double centerY = sizeM * 0.5;
    
    // 1. Create planet
    auto planet = registry.create();
    registry.emplace<Components::Position>(planet, centerX, centerY);
    registry.emplace<Components::Velocity>(planet, 0.0, 0.0);
    registry.emplace<Components::Mass>(planet, scenarioEntityConfig.planetMass);
    registry.emplace<Components::ParticlePhase>(planet, Components::Phase::Solid);
    
    // Planet visual representation
    double const planetRadius = scenarioEntityConfig.planetRadius;
    registry.emplace<Components::Shape>(planet, Components::ShapeType::Circle, planetRadius);
    registry.emplace<CircleShape>(planet, planetRadius);
    
    // Green planet color
    registry.emplace<Components::Color>(planet, 30, 150, 50);
    
    // 2. Create ocean particles around the planet
    createOceanParticles(registry, planet);
    
    std::cerr << "Planetary ocean scenario ready with " << scenarioEntityConfig.oceanParticleCount
              << " ocean particles." << std::endl;
}

void PlanetaryOceanScenario::createOceanParticles(
    entt::registry &registry,
    entt::entity planetEntity) const
{
    // Get center position of the planet
    const auto& planetPos = registry.get<Components::Position>(planetEntity);
    double centerX = planetPos.x;
    double centerY = planetPos.y;
    
    // Ocean parameters
    double const planetRadius = scenarioEntityConfig.planetRadius;
    double const oceanDepth = scenarioEntityConfig.oceanDepth;
    double const numLayers = scenarioEntityConfig.oceanLayers;
    int const numParticles = scenarioEntityConfig.oceanParticleCount;
    
    // Use the explicit particle radius from config
    double particleRadius = scenarioEntityConfig.fluidParticleRadius;
    
    // Calculate number of particles per layer
    double baseRadius = planetRadius + (oceanDepth / numLayers * 0.5); // Distance to first layer
    double baseCircumference = 2.0 * SimulatorConstants::Pi * baseRadius;
    int particlesPerLayer = numParticles / static_cast<int>(numLayers);
    
    // Create particles with some randomness to avoid perfect symmetry
    std::default_random_engine generator{static_cast<unsigned int>(time(nullptr))};
    std::uniform_real_distribution<double> jitterDist(-0.2, 0.2);
    
    int created = 0;
    
    // Create particles in layers
    for (int layer = 0; layer < numLayers; layer++) {
        // Calculate this layer's radius
        double layerRadius = planetRadius + (0.5 + layer) * (oceanDepth / numLayers);
        
        // Distribute particles evenly around this layer
        double angleStep = 2.0 * SimulatorConstants::Pi / particlesPerLayer;
        
        for (int i = 0; i < particlesPerLayer && created < numParticles; i++) {
            double angle = i * angleStep;
            
            // Add small jitter to radius and angle
            double jitteredRadius = layerRadius * (1.0 + jitterDist(generator) * 0.1);
            double jitteredAngle = angle + jitterDist(generator) * angleStep * 0.2;
            
            double x = centerX + jitteredRadius * std::cos(jitteredAngle);
            double y = centerY + jitteredRadius * std::sin(jitteredAngle);
            
            auto e = registry.create();
            registry.emplace<Components::Position>(e, x, y);
            
            // No initial velocity - will be pulled by gravity
            registry.emplace<Components::Velocity>(e, 0.0, 0.0);
            
            // Fluid properties
            registry.emplace<Components::Mass>(e, scenarioEntityConfig.oceanParticleMass);
            registry.emplace<Components::ParticlePhase>(e, Components::Phase::Liquid);
            registry.emplace<Components::Material>(e, 
                                                 scenarioEntityConfig.fluidStaticFriction,
                                                 scenarioEntityConfig.fluidDynamicFriction);
            
            // Fluid particle shape
            registry.emplace<Components::Shape>(e, Components::ShapeType::Circle, particleRadius);
            registry.emplace<CircleShape>(e, particleRadius);
            
            // SPH-related properties for fluid simulation
            registry.emplace<Components::SmoothingLength>(e, oceanDepth / numLayers * 2.0);
            registry.emplace<Components::SpeedOfSound>(e, 1000.0);
            registry.emplace<Components::SPHTemp>(e);
            
            // Bright blue color for improved visibility
            registry.emplace<Components::Color>(e, 0, 150, 255);
            
            created++;
        }
    }
} 