/**
 * @file simple_fluid_scenario.hpp
 * @brief Declaration of a simple fluid scenario with boundary walls
 */

#pragma once

#include "nbody/scenarios/i_scenario.hpp"

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
