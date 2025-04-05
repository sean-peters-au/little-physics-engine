/**
 * @fileoverview sim_manager.cpp
 * @brief Implementation of SimManager.
 */

#include <iostream>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/System/Time.hpp>

#include "sim_manager.hpp"
#include "presentation_manager.hpp"
#include "entities/sim_components.hpp"
#include "core/constants.hpp"
#include "core/profile.hpp"
#include "renderer_types.hpp"
#include "scenario_manager.hpp"
#include "sim.hpp"

// Singleton instance getter
SimManager& SimManager::getInstance() {
    static SimManager instance;
    return instance;
}

// Private Constructor
SimManager::SimManager()
    : presentationManagerInstance(PresentationManager::getInstance()),
      simulatorInstance(ECSSimulator::getInstance()),
      scenarioManager(),
      running(true),
      paused(false),
      stepFrame(false),
      timeSinceLastProfilerPrint(sf::Time::Zero),
      profilerPrintInterval(sf::seconds(10.f))
{
}

// Main application loop
void SimManager::run() {
    if (!init()) {
        return;
    }

    sf::Clock clock;
    timeSinceLastProfilerPrint = sf::Time::Zero;
    running = true;

    while (presentationManagerInstance.isWindowOpen()) {
        presentationManagerInstance.handleEvents();
        if (!presentationManagerInstance.isWindowOpen()) {
            running = false;
            break;
        }

        sf::Time dt = clock.restart();

        tick();
        
        render(dt.asSeconds() > 0 ? 1.f / dt.asSeconds() : 0);

        timeSinceLastProfilerPrint += dt;
        if (timeSinceLastProfilerPrint >= profilerPrintInterval) {
            Profiling::Profiler::printStats();
            Profiling::Profiler::reset();
            timeSinceLastProfilerPrint = sf::Time::Zero;
        }
    }
}

// Initialization
bool SimManager::init() {
  if (!presentationManagerInstance.init()) {
    std::cerr << "PresentationManager initialization failed." << std::endl;
    return false;
  }
  scenarioManager.buildScenarioList();
  scenarioManager.setInitialScenario(SimulatorConstants::SimulationType::KEPLERIAN_DISK);
  selectScenario(scenarioManager.getCurrentScenario());
  return true;
}

// Tick simulation
void SimManager::tick() {
  PROFILE_SCOPE("SimManager::tick");

  if (!paused || stepFrame) {
    simulatorInstance.tick();
    stepFrame = false;
  }
}

// Render frame
void SimManager::render(float fps) {
  presentationManagerInstance.renderFrame(fps);
}

// Toggle pause state
void SimManager::togglePause() { paused = !paused; }

// Reset simulation state
void SimManager::resetSimulator() {
  simulatorInstance.reset();
  paused = false;
}

// Step one frame when paused
void SimManager::stepOnce() { stepFrame = true; }

// Set simulation time scale
void SimManager::setTimeScale(double multiplier) {
  auto& registry = simulatorInstance.getRegistry();
  auto view = registry.view<Components::SimulatorState>();
  if (!view.empty()) {
    auto& st = registry.get<Components::SimulatorState>(view.front());
    st.timeScale = multiplier;
  }
}

// Set rendering color scheme
void SimManager::setColorScheme(ColorScheme scheme) {
  presentationManagerInstance.setColorScheme(scheme);
}

// Load and switch simulation scenario
void SimManager::selectScenario(SimulatorConstants::SimulationType scenario) {
  scenarioManager.setInitialScenario(scenario);
  auto scenarioPtr = scenarioManager.createScenario(scenario);
  ScenarioSystemConfig config = scenarioPtr->getSystemsConfig();

  simulatorInstance.applyConfig(config);
  presentationManagerInstance.updateCoordinates(config.sharedConfig);
  simulatorInstance.loadScenario(std::move(scenarioPtr));
  simulatorInstance.reset();

  paused = false;
}

// Getter for current scenario type
SimulatorConstants::SimulationType SimManager::getCurrentScenarioType() const {
    return scenarioManager.getCurrentScenario();
}

// Getter for available scenario list
const std::vector<std::pair<SimulatorConstants::SimulationType, std::string>>& SimManager::getScenarioList() const {
    return scenarioManager.getScenarioList();
}