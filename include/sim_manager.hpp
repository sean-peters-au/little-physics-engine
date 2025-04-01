/**
 * @fileoverview sim_manager.hpp
 * @brief High-level controller for simulation execution and UI orchestration.
 */

#pragma once

#include <memory>

#include "presentation_manager.hpp"
#include "scenario_manager.hpp"
#include "sim.hpp"
#include "renderer_types.hpp"

/**
 * @class SimManager
 * @brief Orchestrates the main loop, owns subsystems, and manages scenario selection.
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

  // Run the main simulation loop
  void run(); // Add a run method if not present

  /**
   * @brief Initializes subsystems like the renderer, simulator, and scenario manager.
   * @return true on success, false otherwise.
   */
  bool init();

  /**
   * @brief Steps the simulation (unless paused).
   */
  void tick();

  /**
   * @brief Renders the simulation and UI.
   * @param fps The current frames-per-second.
   */
  void render(float fps);

  /**
   * @brief Toggles paused state from UI or keyboard input.
   */
  void togglePause();

  /**
   * @brief Resets the simulator from UI or keyboard input.
   */
  void resetSimulator();

  /**
   * @brief Steps one frame while paused.
   */
  void stepOnce();

  /**
   * @brief Sets the simulator's time scale multiplier.
   * @param multiplier The new time scale factor.
   */
  void setTimeScale(double multiplier);

  /**
   * @brief Changes the color scheme via the PresentationManager.
   * @param scheme The desired color scheme.
   */
  void setColorScheme(ColorScheme scheme);

  /**
   * @brief Handles user selection of a new scenario.
   * @param scenario The chosen scenario type.
   */
  void selectScenario(SimulatorConstants::SimulationType scenario);

  // Add getters needed by PresentationManager::renderUI
  bool isPaused() const { return paused; }
  SimulatorConstants::SimulationType getCurrentScenarioType() const;
  const std::vector<std::pair<SimulatorConstants::SimulationType, std::string>>& getScenarioList() const;
  void resetPause() { paused = false; } // Add helper needed by EventManager

  // Public members might become private if accessed via getInstance()
  // PresentationManager presentationManager; // Now accessed via getInstance()
  // ECSSimulator simulator; // Now accessed via getInstance()
  // ScenarioManager scenarioManager;
  // UIManager uiManager;

  // bool running; // Internal state for run() loop
  // bool paused;
  // bool stepFrame;

 private:
  // Make constructor private
  SimManager();
  // Destructor can remain default
  ~SimManager() = default;

  // Members needed for singleton operation
  PresentationManager& presentationManagerInstance;
  ECSSimulator& simulatorInstance;
  ScenarioManager scenarioManager; // Keep as member?

  bool running;
  bool paused;
  bool stepFrame;

  // Potentially move main loop logic here if handleEvents, tick, render become private

  // Private helpers (if tick/render moved from public)
  bool handleEvents(); // Keep private for now, called by run()
};