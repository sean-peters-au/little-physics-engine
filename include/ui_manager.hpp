/**
 * @file ui_manager.hpp
 * @brief Manages UI state, event handling, and calls into Renderer for UI drawing.
 */

#pragma once

#include <vector>
#include <string>
#include <utility>
#include <SFML/Graphics.hpp>

#include "arch/native/renderer_native.hpp"
#include "core/constants.hpp"
#include "entities/sim_components.hpp"

// Forward declaration
class SimManager;

/**
 * @class UIManager
 * @brief Handles user interface interactions (mouse, highlights, clicks), and delegates logic to SimManager.
 */
class UIManager
{
public:
    UIManager();

    /**
     * @brief Called by SimManager to assign a callback interface for simulator control.
     */
    void setSimManager(SimManager* manager);

    /**
     * @brief Sets the scenario list, typically from ScenarioManager, for scenario button UI.
     */
    void setScenarioList(const std::vector<std::pair<SimulatorConstants::SimulationType, std::string>>& scenarios);

    /**
     * @brief Update which UI button is currently highlighted, based on mouse position.
     * @param window The SFML window.
     */
    void updateHighlights(sf::RenderWindow& window);

    /**
     * @brief Handle a mouse click at (x, y), calling back into SimManager as needed.
     * @param x Mouse X
     * @param y Mouse Y
     * @param paused Current paused state (affects "Next Frame" logic).
     */
    void handleClick(int x, int y, bool paused);

    /**
     * @brief Renders the UI via the provided renderer, including scenario buttons, pause/reset, etc.
     */
    void renderUI(Renderer& renderer,
                  const entt::registry& registry,
                  bool paused,
                  SimulatorConstants::SimulationType currentScenario);

private:
    SimManager* simManager;

    // Scenario list from scenario manager
    std::vector<std::pair<SimulatorConstants::SimulationType, std::string>> scenarioList;

    // Variables to track highlight states
    bool highlightPausePlay;
    bool highlightReset;
    SimulatorConstants::SimulationType highlightedScenario;

    // Track scenario, speed, color scheme buttons, etc. for rendering
    // (We re-use the same approach as the old renderer had: store them in UI)
    std::vector<Renderer::UIButton> scenarioButtons;
    std::vector<Renderer::UIButton> speedButtons;
    std::vector<Renderer::UIButton> colorSchemeButtons;
    Renderer::UIButton pausePlayButton;
    Renderer::UIButton resetButton;
    Renderer::UIButton nextFrameButton;
    Renderer::UIButton debugButton;

    /**
     * @brief Internal helper to reset highlight states, re-check them given (mouseX, mouseY).
     */
    void computeHighlights(int mouseX, int mouseY);
};
