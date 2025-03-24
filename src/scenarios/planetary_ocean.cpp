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

SystemConfig PlanetaryOceanScenario::getConfig() const
{
    SystemConfig cfg;
    
    // Planet size and scaling - make planet smaller on screen to emphasize fluid
    double planetRadiusPixels = 70.0;  // 140 pixel diameter
    cfg.MetersPerPixel = scenarioConfig.planetRadius / planetRadiusPixels;
    cfg.UniverseSizeMeters = SimulatorConstants::ScreenLength * cfg.MetersPerPixel;
    
    // Time settings
    cfg.SecondsPerTick = 1.0;
    cfg.TimeAcceleration = 10.0;
    
    // Grid settings
    cfg.GridSize = 100;
    cfg.CellSizePixels = static_cast<double>(SimulatorConstants::ScreenLength) / cfg.GridSize;

    // Physical parameters
    cfg.GravitationalSoftener = scenarioConfig.planetRadius * 0.01;
    cfg.CollisionCoeffRestitution = 0.1;
    cfg.DragCoeff = 0.0;
    cfg.ParticleDensity = scenarioConfig.fluidRestDensity;

    return cfg;
}

void PlanetaryOceanScenario::createEntities(entt::registry &registry) const
{
    // Get system configuration
    SystemConfig sysConfig = getConfig();
    double const sizeM = sysConfig.UniverseSizeMeters;
    
    // Center coordinates in meters
    double centerX = sizeM * 0.5;
    double centerY = sizeM * 0.5;
    
    // 1. Create planet
    auto planet = registry.create();
    registry.emplace<Components::Position>(planet, centerX, centerY);
    registry.emplace<Components::Velocity>(planet, 0.0, 0.0);
    registry.emplace<Components::Mass>(planet, scenarioConfig.planetMass);
    registry.emplace<Components::ParticlePhase>(planet, Components::Phase::Solid);
    
    // Planet visual representation
    double const planetRadius = scenarioConfig.planetRadius;
    registry.emplace<Components::Shape>(planet, Components::ShapeType::Circle, planetRadius);
    registry.emplace<CircleShape>(planet, planetRadius);
    
    // Green planet color
    registry.emplace<Components::Color>(planet, 30, 150, 50);
    
    // 2. Create ocean particles around the planet
    createOceanParticles(registry, planet);
    
    std::cerr << "Planetary ocean scenario ready with " << scenarioConfig.oceanParticleCount
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
    double const planetRadius = scenarioConfig.planetRadius;
    double const oceanDepth = scenarioConfig.oceanDepth;
    double const numLayers = scenarioConfig.oceanLayers;
    int const numParticles = scenarioConfig.oceanParticleCount;
    
    // Use the explicit particle radius from config
    double particleRadius = scenarioConfig.fluidParticleRadius;
    
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
            registry.emplace<Components::Mass>(e, scenarioConfig.oceanParticleMass);
            registry.emplace<Components::ParticlePhase>(e, Components::Phase::Liquid);
            registry.emplace<Components::Material>(e, 
                                                 scenarioConfig.fluidStaticFriction,
                                                 scenarioConfig.fluidDynamicFriction);
            
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