/**
 * @file ui_manager.cpp
 * @brief Implementation of UIManager: handles user-interface state and calls into SimManager.
 */

#include <cmath>

#include <SFML/Window/Mouse.hpp>

#include "core/constants.hpp"
#include "sim_manager.hpp"
#include "ui_manager.hpp"
#include "presentation_manager.hpp"
#include "entities/sim_components.hpp"

// Include the full definition needed for DebugMode enum and methods
#include "arch/native/renderer_fluid_dsf.hpp"

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
    if (!simManager) return; // Safety check

    if (pausePlayButton.rect.contains({x, y})) {
        simManager->togglePause(); return;
    }
    if (resetButton.rect.contains({x, y})) {
        simManager->resetSimulator(); return;
    }
    for (auto& btn : scenarioButtons) {
        if (btn.rect.contains({x, y})) {
            simManager->selectScenario(btn.scenario); return;
        }
    }
    for (auto& btn : speedButtons) {
        if (btn.rect.contains({x, y})) {
            simManager->setTimeScale(btn.speedMultiplier); return;
        }
    }
    for (auto& btn : colorSchemeButtons) {
        if (btn.rect.contains({x, y})) {
            if (btn.label == "Default") simManager->setColorScheme(ColorScheme::DEFAULT);
            else if (btn.label == "Sleep") simManager->setColorScheme(ColorScheme::SLEEP);
            else if (btn.label == "Temperature") simManager->setColorScheme(ColorScheme::TEMPERATURE);
            return;
        }
    }
    if (paused && nextFrameButton.rect.contains({x, y})) {
        simManager->stepOnce(); return;
    }
    if (debugButton.rect.contains({x, y})) {
        simManager->presentationManager.toggleDebugVisualization();
        return;
    }
}

void UIManager::renderUI(PresentationManager& presentationManager,
                         const entt::registry& registry,
                         bool paused,
                         SimulatorConstants::SimulationType currentScenario)
{
    scenarioButtons.clear();
    speedButtons.clear();
    colorSchemeButtons.clear();

    int const panelX = static_cast<int>(SimulatorConstants::ScreenLength) + 10;
    int panelY = 10;

    std::string pauseLabel = paused ? "Play" : "Pause";
    pausePlayButton.rect = sf::IntRect(panelX, panelY, 60, 20);
    pausePlayButton.label = pauseLabel;
    pausePlayButton.isSpecialButton = true;
    pausePlayButton.scenario = currentScenario;

    presentationManager.drawButton(pausePlayButton, highlightPausePlay ? sf::Color(200,200,0) : sf::Color(100,100,100));
    panelY += 25;

    nextFrameButton.rect = sf::IntRect(panelX, panelY, 80, 20);
    nextFrameButton.label = "Next Frame";
    nextFrameButton.isSpecialButton = true;
    nextFrameButton.scenario = currentScenario;

    presentationManager.drawButton(nextFrameButton,
                                 paused ? sf::Color(100,100,100) : sf::Color(50,50,50),
                                 paused ? sf::Color::White : sf::Color(150,150,150));
    panelY += 25;

    resetButton.rect = sf::IntRect(panelX, panelY, 60, 20);
    resetButton.label = "Reset";
    resetButton.isSpecialButton = true;
    resetButton.scenario = currentScenario;

    presentationManager.drawButton(resetButton, highlightReset ? sf::Color(200,200,0) : sf::Color(100,100,100));
    panelY += 25;

    presentationManager.renderText("Playback Speed:", panelX, panelY);
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
            UIButton btn;
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
            presentationManager.drawButton(btn, fillColor);
            speedButtons.push_back(btn);
            panelY += 25;
        }
    }
    panelY += 20;

    presentationManager.renderText("Color Scheme:", panelX, panelY);
    panelY += 25;
    {
        std::vector<std::pair<ColorScheme, std::string>> schemes = {
            {ColorScheme::DEFAULT, "Default"},
            {ColorScheme::SLEEP, "Sleep"},
            {ColorScheme::TEMPERATURE, "Temperature"}
        };

        for (auto& scheme : schemes)
        {
            UIButton btn;
            btn.rect = sf::IntRect(panelX, panelY, 100, 25);
            btn.label = scheme.second;
            btn.isSpecialButton = true;

            sf::Color fillColor = sf::Color(100,100,100);
            if (presentationManager.getColorScheme() == scheme.first)
            {
                fillColor = sf::Color(0,200,0);
            }
            presentationManager.drawButton(btn, fillColor);
            colorSchemeButtons.push_back(btn);
            panelY += 25;
        }
    }
    panelY += 20;

    presentationManager.renderText("Debug View:", panelX, panelY);
    panelY += 25;
    {
        debugButton.rect = sf::IntRect(panelX, panelY, 100, 25);
        bool debugActive = presentationManager.isDebugVisualization();

        std::string debugLabel = "Debug: OFF";
        if (debugActive) debugLabel = "Debug: ON";
        
        debugButton.label = debugLabel;
        debugButton.isSpecialButton = true;

        sf::Color fillColor = debugActive ? sf::Color(0,200,0) : sf::Color(100,100,100);
        presentationManager.drawButton(debugButton, fillColor);
        panelY += 25;
    }
    panelY += 20;

    presentationManager.renderText("Scenarios:", panelX, panelY);
    panelY += 25;
    for (auto& sc : scenarioList)
    {
        UIButton btn;
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

        presentationManager.drawButton(btn, fillColor);

        scenarioButtons.push_back(btn);
        panelY += 25;
    }
}

void UIManager::computeHighlights(int mouseX, int mouseY)
{
    highlightPausePlay = pausePlayButton.rect.contains({mouseX, mouseY});
    highlightReset = resetButton.rect.contains({mouseX, mouseY});

    highlightedScenario = static_cast<SimulatorConstants::SimulationType>(-1);
    for (auto& btn : scenarioButtons)
    {
        if (btn.rect.contains({mouseX, mouseY}))
        {
            highlightedScenario = btn.scenario;
            break;
        }
    }
}
