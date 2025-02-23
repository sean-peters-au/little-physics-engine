/**
 * @file scenario_manager.hpp
 * @brief Handles building scenario lists and updating the simulator for a chosen scenario.
 */

#ifndef SCENARIO_MANAGER_HPP
#define SCENARIO_MANAGER_HPP

#include <vector>
#include <string>
#include <utility>

#include "nbody/core/simulator.hpp"

/**
 * @class ScenarioManager
 * @brief Manages scenario selection, building scenario lists, and applying scenario settings to ECSSimulator.
 */
class ScenarioManager
{
public:
    /**
     * @brief Builds an internal list of all available scenarios.
     */
    void buildScenarioList();

    /**
     * @brief Returns the built list of scenario (type, name) pairs.
     */
    const std::vector<std::pair<SimulatorConstants::SimulationType,std::string>>& getScenarioList() const;

    /**
     * @brief Sets the initially active scenario.
     */
    void setInitialScenario(SimulatorConstants::SimulationType scenario);

    /**
     * @brief Returns the scenario currently selected.
     */
    SimulatorConstants::SimulationType getCurrentScenario() const;

    /**
     * @brief Applies a new scenario and resets the simulator accordingly.
     */
    void updateSimulatorState(ECSSimulator& simulator, SimulatorConstants::SimulationType scenario);

private:
    std::vector<std::pair<SimulatorConstants::SimulationType, std::string>> scenarioList;
    SimulatorConstants::SimulationType currentScenario;
};

#endif // SCENARIO_MANAGER_HPP
