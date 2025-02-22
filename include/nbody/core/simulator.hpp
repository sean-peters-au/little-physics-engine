/**
 * @file simulator.hpp
 * @brief Main simulator class that manages an ECS registry and scenario lifecycle.
 */

#pragma once

#include <memory>
#include <entt/entt.hpp>
#include "nbody/core/constants.hpp"
#include "nbody/scenarios/i_scenario.hpp"
#include "nbody/systems/fluid/fluid.hpp"

/**
 * @class ECSSimulator
 * @brief Main simulator that manages an ECS registry and scenario lifecycle.
 */
class ECSSimulator {
private:
    entt::registry registry;
    // We hold a scenario pointer if you want to keep track of the current scenario object
    std::unique_ptr<IScenario> scenarioPtr;
    SimulatorConstants::SimulationType currentScenario = SimulatorConstants::SimulationType::KEPLERIAN_DISK;
    std::unique_ptr<Systems::FluidSystem> fluidSystem;

public:
    ECSSimulator();

    /**
     * @brief Additional initialization steps after scenario reset
     */
    void init();

    /**
     * @brief Steps the ECS systems for one tick
     */
    void tick();

    /**
     * @brief Sets the scenario type (Keplerian, Isothermal, etc.)
     */
    void setScenario(SimulatorConstants::SimulationType scenario);

    /**
     * @brief Resets the ECS with the newly selected scenario
     */
    void reset();

    /**
     * @brief Access to the ECS registry
     */
    entt::registry& getRegistry() { return registry; }
    const entt::registry& getRegistry() const { return registry; }

    /**
     * @brief (Optional) If we stored scenarioPtr, we can expose it
     */
    IScenario& getCurrentScenario() const { return *scenarioPtr; }
};
