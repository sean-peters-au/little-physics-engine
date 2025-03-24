/**
 * @file planetary_ocean.hpp
 * @brief Declaration of a scenario with a planetary body and fluid oceans
 */

#pragma once

#include "scenarios/i_scenario.hpp"
#include "systems/system_config.hpp"

/**
 * @struct PlanetaryOceanConfig
 * @brief Simple configuration for a planet with fluid around its perimeter
 */
struct PlanetaryOceanConfig {
    // Planet parameters
    double planetRadius = 200000.0;    // Planet radius in meters
    double planetMass = 1.0e20;        // Just enough mass for gravity
    
    // Ocean parameters
    int oceanParticleCount = 1000;     // Number of ocean particles
    double oceanParticleMass = 1.0e12; // Mass per particle
    double oceanDepth = 50000.0;       // Depth of ocean (25% of planet radius)
    int oceanLayers = 2;               // Number of fluid layers
    double fluidParticleRadius = 2000.0;
    
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

    SystemConfig getConfig() const override;
    void createEntities(entt::registry &registry) const override;
    
private:
    void createOceanParticles(
        entt::registry &registry,
        entt::entity planetEntity) const;
        
    PlanetaryOceanConfig scenarioConfig;
}; 