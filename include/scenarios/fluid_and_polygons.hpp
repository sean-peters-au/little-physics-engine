#pragma once

#include "core/constants.hpp"
#include "systems/shared_system_config.hpp"
#include "scenarios/i_scenario.hpp"
#include "entt/entt.hpp"

/**
 * @struct FluidAndPolygonsConfig
 * @brief Configuration parameters specific to the fluid and polygons scenario
 */
struct FluidAndPolygonsConfig {
    // Fluid parameters
    int fluidParticleCount = 1000;     // Number of fluid particles
    double fluidParticleMass = 0.005;  // Mass per fluid particle
    double fluidRestDensity = 100.0;  // Typical rest density for water
    double InitialVelocityFactor = 1.0;
    
    // Polygon parameters
    int polygonCount = 3;              // How many random polygons to spawn
    double polygonMassMean = 5;        // Typical mass for polygons
    double polygonMassStdDev = 0.2;    // Variation in polygon mass
    
    // Friction parameters
    double floorStaticFriction = 0.6;  
    double floorDynamicFriction = 0.4;
    double wallStaticFriction = 0.2;
    double wallDynamicFriction = 0.1;
    double polyStaticFriction = 0.3;
    double polyDynamicFriction = 0.1;
    double fluidStaticFriction = 0.0;
    double fluidDynamicFriction = 0.0;
    
    // Wall parameters
    double wallThickness = 0.1;        // Thickness of bounding walls
    double wallMass = 1e30;            // Effectively infinite mass
    
    // Velocity parameters
    double initialVelocityScale = 0.5;
};

/**
 * @brief Test scenario that combines a tank of fluid particles with
 *        polygon obstacles.
 */
class FluidAndPolygonsScenario : public IScenario {
public:
    /**
     * @brief Retrieves the configuration for the test fluid-polygons scenario.
     *
     * @return ScenarioSystemConfig with simulation parameters.
     */
    ScenarioSystemConfig getSystemsConfig() const;

    /**
     * @brief Creates all entities for the scenario, including boundary walls,
     *        fluid particles, and polygon obstacles.
     *
     * @param registry The entt registry into which the entities will be inserted.
     */
    void createEntities(entt::registry &registry) const;
    
private:
    FluidAndPolygonsConfig scenarioEntityConfig;
};
