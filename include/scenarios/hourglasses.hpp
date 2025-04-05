#pragma once

#include "core/constants.hpp"
#include "systems/shared_system_config.hpp"
#include "scenarios/i_scenario.hpp"
#include "entt/entt.hpp"

/**
 * @struct HourglassesConfig
 * @brief Configuration parameters for the hourglasses scenario
 */
struct HourglassesConfig {
    // Fluid parameters
    int fluidParticleCount = 250;     // Number of fluid particles
    double fluidParticleMass = 1.0;   // Mass per fluid particle (increased for better dynamics)
    double fluidRestDensity = 100.0;  // Rest density for fluid
    double fluidParticleSize = 0.05;  // Size of fluid particles (diameter)
    
    // Hexagon parameters
    int hexagonCount = 60;           // Fewer hexagons for better flow
    double hexagonSize = 0.06;        // Larger size of each hexagon
    double hexagonMass = 1.0;         // Mass of each hexagon
    
    // Hourglass parameters
    double hourglassHeight = 4.0;     // Height of each hourglass
    double hourglassTopWidth = 2.0;   // Width of hourglass top chamber
    double hourglassNeckWidth = 0.13;  // Width of hourglass neck
    double hourglassWallThickness = 0.2; // Thickness of hourglass walls
    
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
    double wallMass = 1e30;            // Effectively infinite mass for fixed boundaries
};

/**
 * @brief Scenario that creates two hourglasses side by side - one filled with fluid,
 *        the other filled with small hexagons, to compare their flow characteristics.
 */
class HourglassesScenario : public IScenario {
public:
    /**
     * @brief Retrieves the configuration for the scenario.
     *
     * @return ScenarioSystemConfig with simulation parameters.
     */
    ScenarioSystemConfig getSystemsConfig() const;

    /**
     * @brief Creates all entities for the scenario.
     *
     * @param registry The entt registry into which the entities will be inserted.
     */
    void createEntities(entt::registry &registry) const;
    
private:
    /**
     * @brief Helper function to create a single hourglass boundary
     * 
     * @param registry Registry to add entities to
     * @param centerX X-position of the hourglass center
     * @param centerY Y-position of the hourglass center
     */
    void createHourglass(entt::registry &registry, double centerX, double centerY) const;
    
    HourglassesConfig scenarioEntityConfig;
}; 