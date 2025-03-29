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
    config.sharedConfig.MetersPerPixel = scenarioEntityConfig.planetRadius / planetRadiusPixels * 2;
    config.sharedConfig.UniverseSizeMeters = SimulatorConstants::ScreenLength * config.sharedConfig.MetersPerPixel;
    
    config.sharedConfig.SecondsPerTick = 1.0 / SimulatorConstants::StepsPerSecond; // Use standard timestep
    config.sharedConfig.TimeAcceleration = 5.0;  // No acceleration until stable
    
    // Grid settings
    config.sharedConfig.GridSize = 100;
    config.sharedConfig.CellSizePixels = static_cast<double>(SimulatorConstants::ScreenLength) / config.sharedConfig.GridSize;

    // Physical parameters
    config.sharedConfig.GravitationalSoftener = scenarioEntityConfig.planetRadius * 0.01;
    config.sharedConfig.DragCoeff = 0.0;
    config.sharedConfig.ParticleDensity = scenarioEntityConfig.fluidRestDensity;

    // Configure fluid parameters specifically for planetary scale
    config.fluidConfig.gravity = 0.0f;                 // No gravity - we rely on n-body gravitational system
    config.fluidConfig.restDensity = 100.0f;
    config.fluidConfig.stiffness = 1000.0f;
    config.fluidConfig.viscosity = 0.5f;
    
    // Fluid position solver parameters - properly scaled for planetary radius
    config.fluidConfig.positionSolver.safetyMargin = static_cast<float>(scenarioEntityConfig.fluidParticleRadius); // Match particle radius
    config.fluidConfig.positionSolver.relaxFactor = 0.5f;         // Strong but not too aggressive correction
    config.fluidConfig.positionSolver.maxCorrection = static_cast<float>(scenarioEntityConfig.fluidParticleRadius);      // Allow correction up to particle radius
    config.fluidConfig.dampingFactor = 1.0f;                       // No damping for celestial bodies
    config.fluidConfig.positionSolver.velocityDamping = 0.00001f;
    
    // Fluid impulse solver parameters
    config.fluidConfig.impulseSolver.buoyancyStrength = 0.0f;      // No buoyancy needed with gravity
    config.fluidConfig.impulseSolver.fluidForceScale = 100.0f;     
    config.fluidConfig.impulseSolver.depthScale = 0.1f;

    // Grid parameters for planetary scale simulation
    config.fluidConfig.gridConfig.gridEpsilon = 1e-8f;             // Smaller epsilon for planetary scale
    config.fluidConfig.gridConfig.smoothingLength = static_cast<float>(scenarioEntityConfig.fluidParticleRadius * 4.0); // Reduce h to 4x particle radius
    config.fluidConfig.gridConfig.boundaryOffset = 100.0f;         // Larger offset from boundaries
    
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
    
    // Center coordinates of the universe in meters
    double centerX = sizeM * 0.5;
    double centerY = sizeM * 0.5;
    
    // Distance between planets
    double binaryDistance = scenarioEntityConfig.binaryDistance;
    
    // Calculate center of mass position (same as universe center in this case with equal masses)
    double comX = centerX;
    double comY = centerY;
    
    // Position planets on either side of the center of mass
    double halfDist = binaryDistance * 0.5;
    double planet1X = comX - halfDist;
    double planet1Y = comY;
    double planet2X = comX + halfDist;
    double planet2Y = comY;
    
    // Gravitational constant for orbital velocity calculation
    const double G = 6.67430e-11;
    
    // Calculate orbital velocities for circular orbit around COM
    double planet1Mass = scenarioEntityConfig.planetMass;
    double planet2Mass = scenarioEntityConfig.moonMass;
    
    // For binary systems, each body orbits the center of mass
    // The correct formula is v = sqrt(G * (M1 + M2) * m2 / (m1 + m2) / r)
    // Since m1 = m2 in our case, this simplifies to v = sqrt(G * M_total / (2*r))
    // Where r is the distance from center of mass (halfDist)
    double totalMass = planet1Mass + planet2Mass;
    double orbitalVelocity = std::sqrt(G * totalMass / binaryDistance) / 6;
    
    // 1. Create first planet (left side) with orbital velocity
    auto planet1 = createPlanet(registry, planet1X, planet1Y);
    
    // Add orbital velocity to planet 1 (up direction)
    auto& planet1Vel = registry.get<Components::Velocity>(planet1);
    planet1Vel.y = -orbitalVelocity;
    
    // 2. Create second planet (right side) with orbital velocity
    const Components::Position planet1Pos = registry.get<Components::Position>(planet1);
    auto planet2 = createMoon(registry, planet1Pos);
    
    // 3. Create ocean particles around both planets
    // createOcean(registry, planet1);
    createOcean(registry, planet2);
    
    std::cerr << "Binary planet system created with planet masses: " << planet1Mass 
              << " kg, orbital velocity: " << orbitalVelocity 
              << " m/s, and " << (2 * scenarioEntityConfig.oceanParticleCount)
              << " total ocean particles." << std::endl;
}

entt::entity PlanetaryOceanScenario::createMoon(
    entt::registry &registry,
    const Components::Position &planetPos) const
{
    // Moon parameters from config
    double moonRadius = scenarioEntityConfig.moonRadius;
    double moonMass = scenarioEntityConfig.moonMass;
    double binaryDistance = scenarioEntityConfig.binaryDistance;
    
    // Position the moon to the right of the first planet
    double moonX = planetPos.x + binaryDistance;
    double moonY = planetPos.y;
    
    // Calculate orbital velocity (v = sqrt(G * M / r))
    // G is gravitational constant 6.67430e-11
    const double G = 6.67430e-11;
    double planetMass = scenarioEntityConfig.planetMass;
    
    // For binary systems with equal masses orbiting around center of mass
    double totalMass = planetMass + moonMass;
    double orbitalVelocity = std::sqrt(G * totalMass / binaryDistance) / 6;
    
    // Create moon entity
    auto moonEntity = registry.create();
    
    // Position and velocity (perpendicular to radius for circular orbit)
    registry.emplace<Components::Position>(moonEntity, moonX, moonY);
    registry.emplace<Components::Velocity>(moonEntity, 0.0, orbitalVelocity); // Moving up (opposite from planet1)
    registry.emplace<Components::Mass>(moonEntity, moonMass);
    registry.emplace<Components::ParticlePhase>(moonEntity, Components::Phase::Solid);
    
    // Visual representation
    registry.emplace<Components::Shape>(moonEntity, Components::ShapeType::Circle, moonRadius);
    registry.emplace<CircleShape>(moonEntity, moonRadius);
    
    // Blue color for the second planet
    registry.emplace<Components::Color>(moonEntity, 30, 100, 200);
    
    std::cerr << "Second planet created with orbital velocity: " << orbitalVelocity << " m/s" << std::endl;
    
    return moonEntity;
}

entt::entity PlanetaryOceanScenario::createPlanet(
    entt::registry &registry,
    double centerX,
    double centerY) const
{
    // Create planet entity
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
    
    return planet;
}

void PlanetaryOceanScenario::createOcean(
    entt::registry &registry,
    entt::entity planetEntity) const
{
    // Get center position of the planet
    const auto& planetPos = registry.get<Components::Position>(planetEntity);
    const auto& planetVel = registry.get<Components::Velocity>(planetEntity); // Get planet velocity
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
    // Start directly from planet surface rather than with a gap
    double baseRadius = planetRadius; // Start from surface
    double baseCircumference = 2.0 * SimulatorConstants::Pi * baseRadius;
    int particlesPerLayer = numParticles / static_cast<int>(numLayers);
    
    // Create particles with some randomness to avoid perfect symmetry
    std::default_random_engine generator{static_cast<unsigned int>(time(nullptr))};
    std::uniform_real_distribution<double> jitterDist(-0.2, 0.2);
    
    int created = 0;
    
    // Create particles in layers
    for (int layer = 0; layer < numLayers; layer++) {
        // Calculate this layer's radius - start from surface and move outward
        // Use a smaller fraction of the ocean depth for each layer
        double layerRadius = planetRadius + (layer * oceanDepth / numLayers);
        
        // Distribute particles evenly around this layer
        double angleStep = 2.0 * SimulatorConstants::Pi / particlesPerLayer;
        
        for (int i = 0; i < particlesPerLayer && created < numParticles; i++) {
            double angle = i * angleStep;
            
            // Add small jitter to radius and angle
            double jitteredRadius = layerRadius * (1.0 + jitterDist(generator) * 0.05); // Reduced jitter
            double jitteredAngle = angle + jitterDist(generator) * angleStep * 0.1; // Reduced jitter
            
            double x = centerX + jitteredRadius * std::cos(jitteredAngle);
            double y = centerY + jitteredRadius * std::sin(jitteredAngle);
            
            auto e = registry.create();
            registry.emplace<Components::Position>(e, x, y);
            
            // Give fluid initial velocity matching the planet
            registry.emplace<Components::Velocity>(e, planetVel.x, planetVel.y);
            
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
            registry.emplace<Components::SpeedOfSound>(e, 1000.0);
            registry.emplace<Components::SPHTemp>(e);
            
            // Bright blue color for improved visibility
            registry.emplace<Components::Color>(e, 0, 150, 255);
            
            created++;
        }
    }
} 