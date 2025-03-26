/**
 * @file simple_fluid_scenario.hpp
 * @brief Declaration of a simple fluid scenario with boundary walls
 */

#pragma once

#include "scenarios/i_scenario.hpp"
#include "systems/shared_system_config.hpp"

/**
 * @struct SimpleFluidConfig
 * @brief Configuration parameters specific to the simple fluid scenario
 */
struct SimpleFluidConfig {
    // Fluid parameters
    int fluidParticleCount = 1000;     // Number of fluid particles
    double fluidParticleMass = 0.005;   // Mass per fluid particle
    double fluidRestDensity = 1000.0;  // Typical rest density for water
    
    // Wall parameters
    double wallThickness = 0.1;        // Thickness of bounding walls
    double wallMass = 1e30;            // Effectively infinite mass
    
    // Friction parameters
    double fluidStaticFriction = 0.0;
    double fluidDynamicFriction = 0.0;
    
    // Fluid layout parameters
    double fluidRegionMinX = 0.3;      // Region bounds as fraction of screen size
    double fluidRegionMaxX = 0.7;
    double fluidRegionMinY = 0.3;
    double fluidRegionMaxY = 0.7;
};

/**
 * @class SimpleFluidScenario
 * 
 * Spawns a 2D box with boundary walls and a region of fluid (Liquid phase) 
 * particles. Demonstrates a basic SPH fluid setup without rigid-body coupling.
 */
class SimpleFluidScenario : public IScenario {
public:
    SimpleFluidScenario() = default;
    ~SimpleFluidScenario() override = default;

    ScenarioSystemConfig getSystemsConfig() const override;
    void createEntities(entt::registry &registry) const override;
    
private:
    SimpleFluidConfig scenarioEntityConfig;
};
