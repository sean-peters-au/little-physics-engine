/**
 * @fileoverview sim_manager.hpp
 * @brief High-level controller for simulation execution and UI orchestration.
 *        Owns the main application loop and references other manager singletons.
 */

#pragma once

#include <memory>

#include "presentation_manager.hpp"
#include "scenario_manager.hpp"
#include "sim.hpp"
#include "renderer_types.hpp"

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
  void render(float fps);
  bool handleEvents(); // Event polling, may be removed fully later

  // Singleton references
  PresentationManager& presentationManagerInstance;
  ECSSimulator& simulatorInstance;

  // Owned subsystems/state
  ScenarioManager scenarioManager;
  bool running;
  bool paused;
  bool stepFrame;
};