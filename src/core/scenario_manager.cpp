/**
 * @file scenario_manager.cpp
 * @brief Implementation of ScenarioManager.
 */

#include <iostream>

#include "nbody/core/constants.hpp"
#include "nbody/core/scenario_manager.hpp"
#include "nbody/components/sim.hpp"

void ScenarioManager::buildScenarioList()
{
    scenarioList.clear();
    for (auto s : SimulatorConstants::getAllScenarios())
    {
        scenarioList.emplace_back(s, SimulatorConstants::getScenarioName(s));
    }
}

const std::vector<std::pair<SimulatorConstants::SimulationType,std::string>>&
ScenarioManager::getScenarioList() const
{
    return scenarioList;
}

void ScenarioManager::setInitialScenario(SimulatorConstants::SimulationType scenario)
{
    currentScenario = scenario;
}

SimulatorConstants::SimulationType ScenarioManager::getCurrentScenario() const
{
    return currentScenario;
}

void ScenarioManager::updateSimulatorState(ECSSimulator& simulator,
                                          SimulatorConstants::SimulationType scenario)
{
    currentScenario = scenario;
    SimulatorConstants::initializeConstants(scenario);

    simulator.setScenario(scenario);

    auto& registry = simulator.getRegistry();
    auto stateView = registry.view<Components::SimulatorState>();
    if (!stateView.empty())
    {
        auto& state = registry.get<Components::SimulatorState>(stateView.front());
        state.baseTimeAcceleration = SimulatorConstants::TimeAcceleration;
    }

    simulator.reset();
}
