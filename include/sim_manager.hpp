/**
 * @fileoverview sim_manager.hpp
 * @brief High-level controller for simulation execution and UI orchestration.
 *        Owns the main application loop and references other manager singletons.
 */

#pragma once

#include <memory>
#include <SFML/System/Time.hpp>
#include <SFML/System/Clock.hpp>

#include "presentation_manager.hpp"
#include "scenario_manager.hpp"
#include "sim.hpp"
#include "renderer_types.hpp"
#include "core/constants.hpp"

/**
 * @class SimManager
 * @brief Orchestrates the main loop, simulation state, and scenario management.
 *        Implemented as a Singleton.
 */
class SimManager {
 public:
  // Delete copy/move constructors and assignment operators
  SimManager(const SimManager&) = delete;
  SimManager& operator=(const SimManager&) = delete;
  SimManager(SimManager&&) = delete;
  SimManager& operator=(SimManager&&) = delete;

  /** @brief Get the singleton instance. */
  static SimManager& getInstance();

  /** @brief Runs the main simulation loop (event handling, ticking, rendering). */
  void run();

  /**
   * @brief Initializes subsystems (calls init on PresentationManager, loads initial scenario).
   * @return true on success, false otherwise.
   */
  bool init();

  /**
   * @brief Toggles the simulation pause state.
   */
  void togglePause();

  /**
   * @brief Resets the simulation via ECSSimulator and resets pause state.
   */
  void resetSimulator();

  /**
   * @brief Advances the simulation by one frame if currently paused.
   */
  void stepOnce();

  /**
   * @brief Sets the simulation's time scale multiplier via ECSSimulator.
   * @param multiplier The new time scale factor.
   */
  void setTimeScale(double multiplier);

  /**
   * @brief Sets the rendering color scheme via PresentationManager.
   * @param scheme The desired color scheme.
   */
  void setColorScheme(ColorScheme scheme);

  /**
   * @brief Loads and initializes a new simulation scenario.
   * @param scenario The chosen scenario type.
   */
  void selectScenario(SimulatorConstants::SimulationType scenario);

  // Getters
  bool isPaused() const { return paused; }
  SimulatorConstants::SimulationType getCurrentScenarioType() const;
  const std::vector<std::pair<SimulatorConstants::SimulationType, std::string>>& getScenarioList() const;
  void resetPause() { paused = false; }

 private:
  // Private constructor/destructor for singleton
  SimManager();
  ~SimManager() = default;

  // Private helper methods called by run()
  void tick();
  void render(float actualFPS, float actualTPS);

  // Singleton references
  PresentationManager& presentationManagerInstance;
  ECSSimulator& simulatorInstance;

  // Owned subsystems/state
  ScenarioManager scenarioManager;
  bool running;
  bool paused;
  bool stepFrame;

  // Timer for profiler printing
  sf::Time timeSinceLastProfilerPrint;
  const sf::Time profilerPrintInterval = sf::seconds(10.f);

  // --- Members for decoupled loop and stats ---
  // Target rates
  const float targetTPS = static_cast<float>(SimulatorConstants::StepsPerSecond);
  const float targetFPS = 60.0f;  // Target frames per second (can adjust later)

  // Timing for stats calculation
  sf::Clock statsClock;
  sf::Time statsAccumulator;
  const sf::Time statsUpdateInterval = sf::seconds(0.5f);
  unsigned int frameCount = 0;
  unsigned int tickCount = 0;
  float actualFPS = 0.0f;
  float actualTPS = 0.0f;
  // --- End members ---
};