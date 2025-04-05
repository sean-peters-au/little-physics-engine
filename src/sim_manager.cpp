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
      stepFrame(false)
{
}

// Main application loop
void SimManager::run() {
    if (!init()) {
        return;
    }

    sf::Clock frameClock; // Used for delta time calculation
    // Initialize timers/accumulators
    timeSinceLastProfilerPrint = sf::Time::Zero;
    statsAccumulator = sf::Time::Zero;
    sf::Time simulationAccumulator = sf::Time::Zero; // Accumulator for simulation ticks
    const sf::Time fixedTickDt = sf::seconds(1.f / targetTPS); // Fixed time step duration

    frameCount = 0;
    tickCount = 0;
    statsClock.restart(); // Start clock for stats interval

    running = true;
    while (presentationManagerInstance.isWindowOpen()) {
        // --- Time Management ---
        sf::Time dt = frameClock.restart(); // Real time elapsed since last frame
        simulationAccumulator += dt;
        statsAccumulator += dt;
        timeSinceLastProfilerPrint += dt;

        // --- Event Handling ---
        presentationManagerInstance.handleEvents();
        if (!presentationManagerInstance.isWindowOpen()) {
            running = false;
            break;
        }

        // --- Simulation Ticks (Fixed Timestep Loop) ---
        const int MAX_TICKS_PER_FRAME = 5; // Limit ticks to prevent spiral
        int ticksThisFrame = 0; 
        while (simulationAccumulator >= fixedTickDt && ticksThisFrame < MAX_TICKS_PER_FRAME) { // Add tick limit
            if (!paused || stepFrame) { 
                tick(); 
                tickCount++;
                ticksThisFrame++;
                stepFrame = false; 
            } else {
                break; 
            }
            simulationAccumulator -= fixedTickDt;
        }
        // Optional: If accumulator is still large after max ticks, log a warning or clamp it?
        // if (simulationAccumulator >= fixedTickDt) { 
        //    std::cout << "Warning: Simulation lagging behind real time!" << std::endl;
        //    simulationAccumulator = sf::Time::Zero; // Example: Discard remaining time to prevent future spiral
        // }

        // --- Rendering --- 
        // We pass actualFPS and actualTPS, which are updated periodically below
        render(actualFPS, actualTPS); // Render the current state
        frameCount++;

        // --- Stats Calculation (Periodic) ---
        if (statsAccumulator >= statsUpdateInterval) {
            float elapsedSeconds = statsAccumulator.asSeconds();
            actualFPS = (elapsedSeconds > 0) ? static_cast<float>(frameCount) / elapsedSeconds : 0.0f;
            actualTPS = (elapsedSeconds > 0) ? static_cast<float>(tickCount) / elapsedSeconds : 0.0f;

            // Reset for next interval
            frameCount = 0;
            tickCount = 0;
            statsAccumulator = sf::Time::Zero; // Reset interval timer
            // statsClock.restart(); // Not strictly needed if using accumulator
        }

        // --- Profiler Print Logic (Periodic) --- 
        if (timeSinceLastProfilerPrint >= profilerPrintInterval) {
            Profiling::Profiler::printStats();
            Profiling::Profiler::reset();
            timeSinceLastProfilerPrint = sf::Time::Zero; // Reset profiler timer
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
void SimManager::render(float currentActualFPS, float currentActualTPS) {
  presentationManagerInstance.renderFrame(currentActualFPS, currentActualTPS);
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