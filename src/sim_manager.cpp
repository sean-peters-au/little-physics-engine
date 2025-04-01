/**
 * @fileoverview sim_manager.cpp
 * @brief Implementation of SimManager, which coordinates subsystems and user interaction.
 */


#include <iostream>

#include <SFML/Window/Event.hpp>

#include "sim_manager.hpp"
#include "presentation_manager.hpp"
#include "entities/sim_components.hpp"
#include "core/constants.hpp"
#include "core/profile.hpp"
#include "renderer_types.hpp"
#include "scenario_manager.hpp"

// Singleton instance getter implementation
SimManager& SimManager::getInstance() {
    static SimManager instance;
    return instance;
}

// Private Constructor Implementation
SimManager::SimManager()
    // Get instances of other singletons
    : presentationManagerInstance(PresentationManager::getInstance()),
      simulatorInstance(ECSSimulator::getInstance()),
      // scenarioManager initialized normally
      scenarioManager(),
      running(true),
      paused(false),
      stepFrame(false)
{
  // No setup needed from UIManager anymore
}

// Run method containing the main loop
void SimManager::run() {
    if (!init()) {
        return; // Initialization failed
    }

    sf::Clock clock; // For FPS calculation
    running = true;

    while (presentationManagerInstance.isWindowOpen()) { // Check window state via PresentationManager
        // --- Event Handling (now done by PresentationManager) ---
        presentationManagerInstance.handleEvents();
        if (!presentationManagerInstance.isWindowOpen()) { // Re-check after handling events
            running = false; // Ensure loop terminates if window closed by event
            break;
        }

        // --- Simulation Tick --- 
        tick(); // Use internal tick method

        // --- Rendering --- 
        float fps = 1.f / clock.restart().asSeconds();
        render(fps); // Use internal render method
    }

    // Optional: Add cleanup here if needed
}

// init, handleEvents, tick, render implementations need to use the singleton instances
bool SimManager::init() {
  // Use singleton instance
  if (!presentationManagerInstance.init()) {
    std::cerr << "PresentationManager initialization failed." << std::endl;
    return false;
  }

  scenarioManager.buildScenarioList();
  // Default scenario
  scenarioManager.setInitialScenario(SimulatorConstants::SimulationType::KEPLERIAN_DISK);

  // Use method to select scenario which uses singleton instances internally
  selectScenario(scenarioManager.getCurrentScenario());

  return true;
}

void SimManager::tick() {
  if (!paused || stepFrame) {
    // Use ECSSimulator singleton instance
    simulatorInstance.tick();
    stepFrame = false;
  }
}

void SimManager::render(float fps) {
  // Delegate frame rendering entirely to PresentationManager
  presentationManagerInstance.renderFrame(fps);
}

// togglePause, resetSimulator, stepOnce remain mostly the same (modify internal state)
void SimManager::togglePause() { paused = !paused; }

void SimManager::resetSimulator() {
  // Use ECSSimulator singleton instance
  simulatorInstance.reset();
  paused = false;
}

void SimManager::stepOnce() { stepFrame = true; }

// setTimeScale uses ECSSimulator singleton instance
void SimManager::setTimeScale(double multiplier) {
  auto& registry = simulatorInstance.getRegistry(); // Use instance
  auto view = registry.view<Components::SimulatorState>();
  if (!view.empty()) {
    auto& st = registry.get<Components::SimulatorState>(view.front());
    st.timeScale = multiplier;
  }
}

// setColorScheme uses PresentationManager singleton instance
void SimManager::setColorScheme(ColorScheme scheme) {
  presentationManagerInstance.setColorScheme(scheme); // Use instance
}

// selectScenario uses singleton instances
void SimManager::selectScenario(SimulatorConstants::SimulationType scenario) {
  scenarioManager.setInitialScenario(scenario);
  auto scenarioPtr = scenarioManager.createScenario(scenario);
  ScenarioSystemConfig config = scenarioPtr->getSystemsConfig();

  // Use instances
  simulatorInstance.applyConfig(config);
  presentationManagerInstance.updateCoordinates(config.sharedConfig);
  simulatorInstance.loadScenario(std::move(scenarioPtr));
  simulatorInstance.reset();

  paused = false;
}

// Add Getter Implementations
SimulatorConstants::SimulationType SimManager::getCurrentScenarioType() const {
    return scenarioManager.getCurrentScenario(); // Delegate to ScenarioManager
}

const std::vector<std::pair<SimulatorConstants::SimulationType, std::string>>& SimManager::getScenarioList() const {
    return scenarioManager.getScenarioList(); // Delegate to ScenarioManager
}