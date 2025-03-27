/**
 * @file planetary_ocean.hpp
 * @brief Declaration of a scenario with a planetary body and fluid oceans
 */

#pragma once

#include "scenarios/i_scenario.hpp"
#include "systems/shared_system_config.hpp"

/**
 * @struct PlanetaryOceanConfig
 * @brief Simple configuration for a planet with fluid around its perimeter
 */
struct PlanetaryOceanConfig {
    // Planet parameters
    double planetRadius = 60000.0;    // Planet radius in meters
    double planetMass = 7.0e24;        // Planet mass
    
    // Ocean parameters
    int oceanParticleCount = 1000;     // Number of ocean particles
    double oceanParticleMass = 1.0e12; // Mass per particle
    double oceanDepth = 10000.0;       // Ocean depth is ~16% of planet radius
    int oceanLayers = 2;               // Number of fluid layers
    double fluidParticleRadius = 2000.0;
    
    // Binary system parameters - using equal masses and radii
    double moonRadius = 60000.0;       // Same radius as planet
    double moonMass = 2.0e24;          // Same mass as planet
    double binaryDistance = 200000.0;  // Distance between centers (about 4x planet radius)
    
    // Fluid properties
    double fluidRestDensity = 1000.0;  // Density of water (kg/m³)
    double fluidStaticFriction = 0.0;  // Water has minimal friction
    double fluidDynamicFriction = 0.0; // Water has minimal friction
};

/**
 * @class PlanetaryOceanScenario
 * @brief A simple scenario with a planet and fluid ocean around its perimeter
 */
class PlanetaryOceanScenario : public IScenario {
public:
    PlanetaryOceanScenario() = default;
    ~PlanetaryOceanScenario() override = default;

    ScenarioSystemConfig getSystemsConfig() const override;
    void createEntities(entt::registry &registry) const override;
    
private:
    void createOcean(
        entt::registry &registry,
        entt::entity planetEntity) const;
        
    entt::entity createPlanet(
        entt::registry &registry,
        double centerX,
        double centerY) const;
        
    entt::entity createMoon(
        entt::registry &registry,
        const Components::Position &planetPos) const;
        
    PlanetaryOceanConfig scenarioEntityConfig;
}; 