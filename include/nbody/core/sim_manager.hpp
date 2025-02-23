/**
 * @file sim_manager.hpp
 * @brief Manages the overall simulation cycle and orchestrates UI and scenario operations.
 */

#ifndef SIM_MANAGER_HPP
#define SIM_MANAGER_HPP

#include <memory>
#include <vector>

#include "nbody/core/simulator.hpp"
#include "nbody/arch/native/renderer_native.hpp"

#include "scenario_manager.hpp"
#include "ui_manager.hpp"

/**
 * @class SimManager
 * @brief High-level controller for simulation execution.
 *
 * SimManager owns:
 * - An ECSSimulator for physics.
 * - A Renderer for drawing.
 * - A ScenarioManager for scenario selection/configuration.
 * - A UIManager for handling UI interactions and rendering UI.
 */
class SimManager
{
public:
    /**
     * @brief Constructs the SimManager.
     */
    SimManager();

    /**
     * @brief Initializes subsystems (Renderer, simulator, etc.).
     * @return true on success, false on failure.
     */
    bool init();

    /**
     * @brief Handles all window/UI events for this frame.
     * @return false if we should quit, true otherwise.
     */
    bool handleEvents();

    /**
     * @brief Steps the simulation (unless paused).
     */
    void tick();

    /**
     * @brief Renders the simulation and UI.
     * @param fps Current frames-per-second to display.
     */
    void render(float fps);

    /**
     * @brief Toggles paused state from UI or key input.
     */
    void togglePause();

    /**
     * @brief Resets the simulator from UI or key input.
     */
    void resetSimulator();

    /**
     * @brief Step exactly one frame while paused.
     */
    void stepOnce();

    /**
     * @brief Sets the simulator’s time scale multiplier.
     * @param multiplier The new time scale factor.
     */
    void setTimeScale(double multiplier);

    /**
     * @brief Changes the color scheme for rendering.
     * @param scheme The chosen color scheme.
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

#endif // SIM_MANAGER_HPP
