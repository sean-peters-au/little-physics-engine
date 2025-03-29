/**
 * @fileoverview scenario_manager.cpp
 * @brief Implementation of ScenarioManager.
 */


#include <iostream>
#include <vector>

#include "scenario_manager.hpp"
#include "scenarios/fluid_and_polygons.hpp"
#include "scenarios/keplerian_disk.hpp"
#include "scenarios/random_polygons.hpp"
#include "scenarios/simple_fluid.hpp"
#include "scenarios/hourglasses.hpp"
#include "scenarios/planetary_ocean.hpp"
#include "scenarios/galton_board.hpp"

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

    case SimulatorConstants::SimulationType::HOURGLASSES:
      return std::make_unique<HourglassesScenario>();

    case SimulatorConstants::SimulationType::PLANETARY_OCEAN:
      return std::make_unique<PlanetaryOceanScenario>();

    case SimulatorConstants::SimulationType::GALTON_BOARD:
      return std::make_unique<GaltonBoardScenario>();

    default:
      return std::make_unique<RandomPolygonsScenario>();
  }
}