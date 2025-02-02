/**
 * @file simple_fluid_scenario.hpp
 * @brief Declaration of a simple fluid scenario with boundary walls
 */

#ifndef SIMPLE_FLUID_SCENARIO_HPP
#define SIMPLE_FLUID_SCENARIO_HPP

#include "nbody/core/i_scenario.hpp"

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

    ScenarioConfig getConfig() const override;
    void createEntities(entt::registry &registry) const override;
};

#endif // SIMPLE_FLUID_SCENARIO_HPP