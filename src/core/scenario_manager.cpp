/**
 * @fileoverview scenario_manager.cpp
 * @brief Implementation of ScenarioManager.
 */


#include <iostream>
#include <vector>

#include "nbody/core/scenario_manager.hpp"
#include "nbody/scenarios/fluid_and_polygons.hpp"
#include "nbody/scenarios/keplerian_disk.hpp"
#include "nbody/scenarios/random_polygons.hpp"
#include "nbody/scenarios/simple_fluid.hpp"

void ScenarioManager::buildScenarioList() {
  scenarioList.clear();
  for (auto s : SimulatorConstants::getAllScenarios()) {
    scenarioList.emplace_back(s, SimulatorConstants::getScenarioName(s));
  }
}

const std::vector<std::pair<SimulatorConstants::SimulationType, std::string>>&
ScenarioManager::getScenarioList() const {
  return scenarioList;
}

void ScenarioManager::setInitialScenario(SimulatorConstants::SimulationType scenario) {
  currentScenario = scenario;
}

SimulatorConstants::SimulationType ScenarioManager::getCurrentScenario() const {
  return currentScenario;
}

std::unique_ptr<IScenario> ScenarioManager::createScenario(
    SimulatorConstants::SimulationType scenarioType) const {
  switch (scenarioType) {
    case SimulatorConstants::SimulationType::KEPLERIAN_DISK:
      return std::make_unique<KeplerianDiskScenario>();

    case SimulatorConstants::SimulationType::RANDOM_POLYGONS:
      return std::make_unique<RandomPolygonsScenario>();

    case SimulatorConstants::SimulationType::SIMPLE_FLUID:
      return std::make_unique<SimpleFluidScenario>();

    case SimulatorConstants::SimulationType::FLUID_AND_POLYGONS:
      return std::make_unique<FluidAndPolygonsScenario>();

    default:
      return std::make_unique<RandomPolygonsScenario>();
  }
}