/**
 * @fileoverview scenario_manager.hpp
 * @brief Manages scenario selection, maintains a list of available scenarios, and creates scenario objects.
 */

#ifndef SCENARIO_MANAGER_HPP
#define SCENARIO_MANAGER_HPP

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nbody/core/constants.hpp"
#include "nbody/scenarios/i_scenario.hpp"

/**
 * @class ScenarioManager
 * @brief Catalog of available scenarios and a factory to create them.
 */
class ScenarioManager {
 public:
  /**
   * @brief Builds an internal list of all available scenarios.
   */
  void buildScenarioList();

  /**
   * @brief Returns the list of scenarios built by buildScenarioList().
   * @return A constant reference to the scenario list.
   */
  const std::vector<std::pair<SimulatorConstants::SimulationType, std::string>>&
  getScenarioList() const;

  /**
   * @brief Sets the scenario that is considered current.
   * @param scenario The chosen scenario type.
   */
  void setInitialScenario(SimulatorConstants::SimulationType scenario);

  /**
   * @brief Returns the currently selected scenario type.
   * @return The scenario type that is currently active.
   */
  SimulatorConstants::SimulationType getCurrentScenario() const;

  /**
   * @brief Creates a new scenario object of the specified type.
   * @param scenarioType The chosen scenario type.
   * @return A unique_ptr to a newly constructed scenario.
   */
  std::unique_ptr<IScenario> createScenario(
      SimulatorConstants::SimulationType scenarioType) const;

 private:
  std::vector<std::pair<SimulatorConstants::SimulationType, std::string>> scenarioList;
  SimulatorConstants::SimulationType currentScenario =
      SimulatorConstants::SimulationType::KEPLERIAN_DISK;
};

#endif  // SCENARIO_MANAGER_HPP