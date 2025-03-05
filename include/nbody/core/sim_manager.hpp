/**
 * @fileoverview sim_manager.hpp
 * @brief High-level controller for simulation execution and UI orchestration.
 */

#pragma once

#include <memory>

#include "nbody/arch/native/renderer_native.hpp"
#include "nbody/core/scenario_manager.hpp"
#include "nbody/core/simulator.hpp"
#include "nbody/core/ui_manager.hpp"

/**
 * @class SimManager
 * @brief Orchestrates the main loop, owns subsystems, and manages scenario selection.
 */
class SimManager {
 public:
  SimManager();

  /**
   * @brief Initializes subsystems like the renderer, simulator, and scenario manager.
   * @return true on success, false otherwise.
   */
  bool init();

  /**
   * @brief Processes window and UI events for the current frame.
   * @return false if the application should quit, true otherwise.
   */
  bool handleEvents();

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
   * @brief Changes the color scheme for the renderer.
   * @param scheme The desired color scheme.
   */
  void setColorScheme(Renderer::ColorScheme scheme);

  /**
   * @brief Handles user selection of a new scenario.
   * @param scenario The chosen scenario type.
   */
  void selectScenario(SimulatorConstants::SimulationType scenario);

  Renderer renderer;
  ECSSimulator simulator;
  ScenarioManager scenarioManager;
  UIManager uiManager;

  bool running;
  bool paused;
  bool stepFrame;
};