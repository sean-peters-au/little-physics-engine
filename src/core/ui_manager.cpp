/**
 * @file ui_manager.cpp
 * @brief Implementation of UIManager: handles user-interface state and calls into SimManager.
 */

#include <cmath>

#include <SFML/Window/Mouse.hpp>

#include "nbody/core/constants.hpp"
#include "nbody/core/sim_manager.hpp"
#include "nbody/core/ui_manager.hpp"
#include "nbody/components/sim.hpp"

UIManager::UIManager()
    : simManager(nullptr)
    , highlightPausePlay(false)
    , highlightReset(false)
    , highlightedScenario(static_cast<SimulatorConstants::SimulationType>(-1))
{
}

void UIManager::setSimManager(SimManager* manager)
{
    simManager = manager;
}

void UIManager::setScenarioList(const std::vector<std::pair<SimulatorConstants::SimulationType, std::string>>& scenarios)
{
    scenarioList = scenarios;
}

void UIManager::updateHighlights(sf::RenderWindow& window)
{
    sf::Vector2i mousePos = sf::Mouse::getPosition(window);
    computeHighlights(mousePos.x, mousePos.y);
}

void UIManager::handleClick(int x, int y, bool paused)
{
    // Click checks
    // Pause/Play
    if (pausePlayButton.rect.contains({x, y}))
    {
        simManager->togglePause();
        return;
    }

    // Reset
    if (resetButton.rect.contains({x, y}))
    {
        simManager->resetSimulator();
        return;
    }

    // Scenario selection
    for (auto& btn : scenarioButtons)
    {
        if (btn.rect.contains({x, y}))
        {
            simManager->selectScenario(btn.scenario);
            return;
        }
    }

    // Speed buttons
    for (auto& btn : speedButtons)
    {
        if (btn.rect.contains({x, y}))
        {
            simManager->setTimeScale(btn.speedMultiplier);
            return;
        }
    }

    // Color scheme buttons
    for (auto& btn : colorSchemeButtons)
    {
        if (btn.rect.contains({x, y}))
        {
            if (btn.label == "Default")
            {
                simManager->setColorScheme(Renderer::ColorScheme::DEFAULT);
            }
            else if (btn.label == "Sleep")
            {
                simManager->setColorScheme(Renderer::ColorScheme::SLEEP);
            }
            else if (btn.label == "Temperature")
            {
                simManager->setColorScheme(Renderer::ColorScheme::TEMPERATURE);
            }
            return;
        }
    }

    // Next Frame button
    if (paused &&
        nextFrameButton.rect.contains({x, y}))
    {
        simManager->stepOnce();
        return;
    }

    // Debug toggle
    if (debugButton.rect.contains({x, y}))
    {
        Renderer& renderer = simManager->renderer;
        renderer.toggleDebugVisualization();
        return;
    }
}

void UIManager::renderUI(Renderer& renderer,
                         const entt::registry& registry,
                         bool paused,
                         SimulatorConstants::SimulationType currentScenario)
{
    scenarioButtons.clear();
    speedButtons.clear();
    colorSchemeButtons.clear();

    // We'll place the UI to the right of the simulation area
    int const panelX = static_cast<int>(SimulatorConstants::ScreenLength) + 10;
    int panelY = 10;

    // 1) Pause/Play
    std::string pauseLabel = paused ? "Play" : "Pause";
    pausePlayButton.rect = sf::IntRect(panelX, panelY, 60, 20);
    pausePlayButton.label = pauseLabel;
    pausePlayButton.isSpecialButton = true;
    pausePlayButton.scenario = currentScenario;

    renderer.drawButton(pausePlayButton, highlightPausePlay ? sf::Color(200,200,0) : sf::Color(100,100,100));
    renderer.renderText(pauseLabel, panelX + 5, panelY + 5);
    panelY += 25;

    // 2) Next Frame
    nextFrameButton.rect = sf::IntRect(panelX, panelY, 80, 20);
    nextFrameButton.label = "Next Frame";
    nextFrameButton.isSpecialButton = true;
    nextFrameButton.scenario = currentScenario;

    renderer.drawButton(nextFrameButton,
        paused ? sf::Color(100,100,100) : sf::Color(50,50,50),
        paused ? sf::Color::White : sf::Color(150,150,150)
    );
    panelY += 25;

    // 3) Reset
    resetButton.rect = sf::IntRect(panelX, panelY, 60, 20);
    resetButton.label = "Reset";
    resetButton.isSpecialButton = true;
    resetButton.scenario = currentScenario;

    renderer.drawButton(resetButton, highlightReset ? sf::Color(200,200,0) : sf::Color(100,100,100));
    renderer.renderText("Reset", panelX + 5, panelY + 5);
    panelY += 25;

    // 4) Speed
    renderer.renderText("Playback Speed:", panelX, panelY);
    panelY += 25;
    {
        std::vector<std::pair<double, std::string>> speeds = {
            {0.25, "0.25x"},
            {0.5,  "0.5x"},
            {1.0,  "1x"}
        };
        const Components::SimulatorState* simState = nullptr;
        auto stView = registry.view<Components::SimulatorState>();
        if (!stView.empty())
        {
            simState = &registry.get<Components::SimulatorState>(stView.front());
        }
        for (auto& sp : speeds)
        {
            Renderer::UIButton btn;
            btn.rect = sf::IntRect(panelX, panelY, 50, 20);
            btn.label = sp.second;
            btn.speedMultiplier = sp.first;
            btn.isSpecialButton = true;
            btn.scenario = currentScenario;

            sf::Color fillColor = sf::Color(100,100,100);
            if (simState && (std::fabs(simState->timeScale - sp.first) < 0.01))
            {
                fillColor = sf::Color(0,200,0);
            }
            renderer.drawButton(btn, fillColor);
            renderer.renderText(btn.label, panelX + 5, panelY + 3);
            speedButtons.push_back(btn);
            panelY += 25;
        }
    }
    panelY += 20;

    // 5) Color scheme
    renderer.renderText("Color Scheme:", panelX, panelY);
    panelY += 25;
    {
        std::vector<std::pair<Renderer::ColorScheme, std::string>> schemes = {
            {Renderer::ColorScheme::DEFAULT, "Default"},
            {Renderer::ColorScheme::SLEEP, "Sleep"},
            {Renderer::ColorScheme::TEMPERATURE, "Temperature"}
        };

        for (auto& scheme : schemes)
        {
            Renderer::UIButton btn;
            btn.rect = sf::IntRect(panelX, panelY, 100, 25);
            btn.label = scheme.second;
            btn.isSpecialButton = true;

            sf::Color fillColor = sf::Color(100,100,100);
            if (renderer.getColorScheme() == scheme.first)
            {
                fillColor = sf::Color(0,200,0);
            }
            renderer.drawButton(btn, fillColor);
            renderer.renderText(btn.label, panelX + 5, panelY + 3);
            colorSchemeButtons.push_back(btn);
            panelY += 25;
        }
    }
    panelY += 20;

    // 6) Debug
    renderer.renderText("Debug View:", panelX, panelY);
    panelY += 25;
    {
        debugButton.rect = sf::IntRect(panelX, panelY, 100, 25);
        // We read the actual debug state from renderer if you have a method or store it locally
        bool debugOn = renderer.isDebugVisualization();
        debugButton.label = debugOn ? "Debug: ON" : "Debug: OFF";
        debugButton.isSpecialButton = true;

        sf::Color fillColor = debugOn ? sf::Color(0,200,0) : sf::Color(100,100,100);
        renderer.drawButton(debugButton, fillColor);
        renderer.renderText(debugButton.label, panelX + 5, panelY + 3);
        panelY += 25;
    }
    panelY += 20;

    // 7) Scenarios
    renderer.renderText("Scenarios:", panelX, panelY);
    panelY += 25;
    for (auto& sc : scenarioList)
    {
        Renderer::UIButton btn;
        btn.rect = sf::IntRect(panelX, panelY, 120, 20);
        btn.label = sc.second;
        btn.isSpecialButton = false;
        btn.scenario = sc.first;

        bool isCurrent = (sc.first == currentScenario);
        bool isHover = (sc.first == highlightedScenario);
        sf::Color fillColor = sf::Color(100,100,100);
        if (isCurrent)
        {
            fillColor = sf::Color(0,200,0);
        }
        else if (isHover)
        {
            fillColor = sf::Color(200,200,0);
        }

        renderer.drawButton(btn, fillColor);
        renderer.renderText(btn.label, panelX + 5, panelY + 5);

        scenarioButtons.push_back(btn);
        panelY += 25;
    }
}

void UIManager::computeHighlights(int mouseX, int mouseY)
{
    highlightPausePlay = false;
    highlightReset = false;
    highlightedScenario = static_cast<SimulatorConstants::SimulationType>(-1);

    // Pause/Play
    if (pausePlayButton.rect.contains({mouseX, mouseY}))
    {
        highlightPausePlay = true;
    }

    // Reset
    if (resetButton.rect.contains({mouseX, mouseY}))
    {
        highlightReset = true;
    }

    // Scenarios
    for (auto& btn : scenarioButtons)
    {
        if (btn.rect.contains({mouseX, mouseY}))
        {
            highlightedScenario = btn.scenario;
            break;
        }
    }
}
