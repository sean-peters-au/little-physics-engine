/**
 * @fileoverview presentation_manager.cpp
 * @brief Implementation of PresentationManager singleton.
 */

#include "presentation_manager.hpp"
#include "renderers/solid_renderer.hpp"
#include "renderers/fluid_renderer.hpp"
#include "renderers/gas_renderer.hpp"
#include "renderers/ui_renderer.hpp"
#include "event_manager.hpp"
#include "sim.hpp"
#include "sim_manager.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <stdexcept> // For std::runtime_error
#include <SFML/Window/Mouse.hpp> // For sf::Mouse

// Required components/utils
#include "entities/entity_components.hpp"
#include "entities/sim_components.hpp"
#include "core/constants.hpp"
#include "math/constants.hpp"
#include "math/polygon.hpp"

// Static Color Mappers
sf::Color PresentationManager::defaultColorMapper(const PixelProperties& /* props */) {
    return sf::Color::White;
}
sf::Color PresentationManager::sleepColorMapper(const PixelProperties& props) {
    if (props.particle_count == 0) { return sf::Color::White; }
    return props.is_asleep ? sf::Color(200, 50, 50) : sf::Color(50, 200, 50);
}
sf::Color PresentationManager::temperatureColorMapper(const PixelProperties& props) {
    if (props.particle_count == 0 || !props.has_temperature) return {128, 128, 128};
    const double minTemp = 0.0, maxTemp = 100.0;
    double t = std::clamp((props.temperature - minTemp) / (maxTemp - minTemp), 0.0, 1.0);
    auto const r = static_cast<uint8_t>(255 * t);
    auto const b = static_cast<uint8_t>(255 * (1.0 - t));
    return {r, 0, b};
}

// Singleton instance getter
PresentationManager& PresentationManager::getInstance() {
    static PresentationManager instance(
        SimulatorConstants::ScreenLength + 200,
        SimulatorConstants::ScreenLength,
        SharedSystemConfig()
    );
    return instance;
}

// Private Constructor
PresentationManager::PresentationManager(int screenWidth, int screenHeight, const SharedSystemConfig& config)
    : initialized(false)
    , screenWidth(screenWidth)
    , screenHeight(screenHeight)
    , coordinates(config, SimulatorConstants::ScreenLength)
    , solidRenderer(std::make_unique<SolidRenderer>(coordinates))
    , fluidRenderer(std::make_unique<FluidRenderer>(coordinates, sf::Vector2u(SimulatorConstants::ScreenLength, SimulatorConstants::ScreenLength)))
    , gasRenderer(std::make_unique<GasRenderer>(coordinates))
{}

// Destructor
PresentationManager::~PresentationManager() = default;

// Initialize window, font, uiRenderer
bool PresentationManager::init() {
    window.create(sf::VideoMode(screenWidth, screenHeight), "N-Body Simulator");
    if (!font.loadFromFile("assets/fonts/arial.ttf")) {
        std::cerr << "Failed to load font assets/fonts/arial.ttf" << std::endl;
        return false;
    }
    uiRenderer = std::make_unique<UIRenderer>(font);
    initialized = true;
    return true;
}

// Clear window
void PresentationManager::clear() { window.clear(sf::Color::Black); }

// Display window contents
void PresentationManager::present() { window.display(); }

// Update coordinate system based on config
void PresentationManager::updateCoordinates(const SharedSystemConfig& config) { coordinates.updateConfig(config); }

// Handle window and UI events
void PresentationManager::handleEvents() {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window.close();
            return;
        }
        EventManager::getInstance().processEvent(event);
    }
    sf::Vector2i mousePos = sf::Mouse::getPosition(window);
    EventManager::getInstance().handleMouseMoved(mousePos.x, mousePos.y);
}

// Render a complete frame
void PresentationManager::renderFrame(float fps) {
    ECSSimulator& simulator = ECSSimulator::getInstance();
    const entt::registry& registry = simulator.getRegistry();
    clear();
    renderFluidParticlesInternal(registry);
    renderSolidParticlesInternal(registry);
    renderGasParticlesInternal(registry);
    renderFPSInternal(fps);
    renderUI();
    present();
}

// Delegate solid particle rendering
void PresentationManager::renderSolidParticlesInternal(const entt::registry &registry) {
    if (solidRenderer) {
        solidRenderer->setDebugRendering(solidDebugEnabled);
        solidRenderer->render(window, registry, currentColorScheme);
    }
}
// Delegate fluid particle rendering
void PresentationManager::renderFluidParticlesInternal(const entt::registry &registry) {
    if (fluidRenderer) { fluidRenderer->render(window, registry); }
}
// Delegate gas particle rendering
void PresentationManager::renderGasParticlesInternal(const entt::registry &registry) {
    if (gasRenderer) { gasRenderer->render(window, registry); }
}
// Render FPS using UIRenderer
void PresentationManager::renderFPSInternal(float fps) {
    if (!uiRenderer) return;
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1) << fps << " FPS";
    uiRenderer->renderText(window, ss.str(), 10, 10, sf::Color::White);
}

// Calculate UI layout and tell UIRenderer to draw it
void PresentationManager::renderUI() {
    if (!uiRenderer) return;
    SimManager& simManager = SimManager::getInstance();
    ECSSimulator& simulator = ECSSimulator::getInstance();
    bool paused = simManager.isPaused();
    SimulatorConstants::SimulationType currentScenario = simManager.getCurrentScenarioType();
    const auto& scenarioList = simManager.getScenarioList();
    const entt::registry& registry = simulator.getRegistry();
    highlightedButtonID = EventManager::getInstance().getHighlightedButtonID();
    currentButtonLayout.clear();
    int const panelX = static_cast<int>(SimulatorConstants::ScreenLength) + 10;
    int panelY = 10;

    // Define and draw Pause/Play Button
    {
        UIButton btn;
        btn.rect = sf::IntRect(panelX, panelY, 60, 20);
        btn.label = paused ? "Play" : "Pause";
        btn.isSpecialButton = true;
        btn.id = ButtonID::PAUSE_PLAY;
        sf::Color color = (highlightedButtonID == btn.id) ? sf::Color(200,200,0) : sf::Color(100,100,100);
        uiRenderer->drawButton(window, btn, color);
        currentButtonLayout.push_back(btn);
        panelY += 25;
    }
    // Define and draw Next Frame Button
    {
        UIButton btn;
        btn.rect = sf::IntRect(panelX, panelY, 80, 20);
        btn.label = "Next Frame";
        btn.isSpecialButton = true;
        btn.id = ButtonID::NEXT_FRAME;
        sf::Color bgColor = paused ? sf::Color(100,100,100) : sf::Color(50,50,50);
        sf::Color fgColor = paused ? sf::Color::White : sf::Color(150,150,150);
        uiRenderer->drawButton(window, btn, bgColor, fgColor);
        currentButtonLayout.push_back(btn);
        panelY += 25;
    }
    // Define and draw Reset Button
    {
        UIButton btn;
        btn.rect = sf::IntRect(panelX, panelY, 60, 20);
        btn.label = "Reset";
        btn.isSpecialButton = true;
        btn.id = ButtonID::RESET;
        sf::Color color = (highlightedButtonID == btn.id) ? sf::Color(200,200,0) : sf::Color(100,100,100);
        uiRenderer->drawButton(window, btn, color);
        currentButtonLayout.push_back(btn);
        panelY += 25;
    }
    // Define and draw Speed Controls
    uiRenderer->renderText(window, "Playback Speed:", panelX, panelY, sf::Color::White);
    panelY += 25;
    {
        std::vector<std::pair<double, std::string>> speeds = {{0.25, "0.25x"}, {0.5, "0.5x"}, {1.0, "1x"}};
        const Components::SimulatorState* simState = nullptr;
        auto stView = registry.view<Components::SimulatorState>();
        if (!stView.empty()) simState = &registry.get<Components::SimulatorState>(stView.front());
        for (const auto& sp : speeds) {
            UIButton btn;
            btn.rect = sf::IntRect(panelX, panelY, 50, 20);
            btn.label = sp.second;
            btn.speedMultiplier = sp.first;
            btn.isSpecialButton = true;
            if (sp.first == 0.25) btn.id = ButtonID::SPEED_0_25X;
            else if (sp.first == 0.5) btn.id = ButtonID::SPEED_0_5X;
            else btn.id = ButtonID::SPEED_1X;
            sf::Color color = (highlightedButtonID == btn.id) ? sf::Color(200,200,0) : sf::Color(100,100,100);
            if (simState && (std::fabs(simState->timeScale - sp.first) < 0.01)) color = sf::Color(0,200,0);
            uiRenderer->drawButton(window, btn, color);
            currentButtonLayout.push_back(btn);
            panelY += 25;
        }
    }
    panelY += 20;
    // Define and draw Color Scheme Buttons
    uiRenderer->renderText(window, "Color Scheme:", panelX, panelY, sf::Color::White);
    panelY += 25;
    {
        std::vector<std::pair<ColorScheme, std::string>> schemes = {
            {ColorScheme::DEFAULT, "Default"}, {ColorScheme::SLEEP, "Sleep"}, {ColorScheme::TEMPERATURE, "Temperature"}
        };
        ColorScheme activeScheme = getColorScheme();
        for (const auto& scheme : schemes) {
            UIButton btn;
            btn.rect = sf::IntRect(panelX, panelY, 100, 25);
            btn.label = scheme.second;
            btn.isSpecialButton = true;
            if (scheme.first == ColorScheme::DEFAULT) btn.id = ButtonID::COLOR_DEFAULT;
            else if (scheme.first == ColorScheme::SLEEP) btn.id = ButtonID::COLOR_SLEEP;
            else btn.id = ButtonID::COLOR_TEMP;
            sf::Color color = (highlightedButtonID == btn.id) ? sf::Color(200,200,0) : sf::Color(100,100,100);
            if (activeScheme == scheme.first) color = sf::Color(0,200,0);
            uiRenderer->drawButton(window, btn, color);
            currentButtonLayout.push_back(btn);
            panelY += 25;
        }
    }
    panelY += 20;
    // Define and draw Debug Toggle Button
    uiRenderer->renderText(window, "Debug View:", panelX, panelY, sf::Color::White);
    panelY += 25;
    {
        UIButton btn;
        btn.rect = sf::IntRect(panelX, panelY, 100, 25);
        btn.isSpecialButton = true;
        btn.id = ButtonID::DEBUG_TOGGLE;
        bool debugActive = isDebugVisualization();
        btn.label = debugActive ? "Debug: ON" : "Debug: OFF";
        sf::Color color = (highlightedButtonID == btn.id) ? sf::Color(200,200,0) : sf::Color(100,100,100);
        if (debugActive) color = sf::Color(0,200,0);
        uiRenderer->drawButton(window, btn, color);
        currentButtonLayout.push_back(btn);
        panelY += 25;
    }
    panelY += 20;
    // Define and draw Scenario Buttons
    uiRenderer->renderText(window, "Scenarios:", panelX, panelY, sf::Color::White);
    panelY += 25;
    for (const auto& sc : scenarioList) {
        UIButton btn;
        btn.rect = sf::IntRect(panelX, panelY, 120, 20);
        btn.label = sc.second;
        btn.isSpecialButton = false;
        btn.scenario = sc.first;
        btn.id = ButtonID::SCENARIO_BUTTON;
        bool isCurrent = (sc.first == currentScenario);
        sf::Color color = (highlightedButtonID == btn.id && !isCurrent) ? sf::Color(200,200,0) : sf::Color(100,100,100);
        if (isCurrent) color = sf::Color(0,200,0);
        uiRenderer->drawButton(window, btn, color);
        currentButtonLayout.push_back(btn);
        panelY += 25;
    }

    // Update EventManager layout for next frame
    EventManager::getInstance().updateLayout(currentButtonLayout);
}

// Set color scheme
void PresentationManager::setColorScheme(ColorScheme scheme) { currentColorScheme = scheme; }

// Get color scheme
ColorScheme PresentationManager::getColorScheme() const { return currentColorScheme; }

// Toggle debug modes
void PresentationManager::toggleDebugVisualization() {
    if (!solidDebugEnabled && fluidRenderer) { // If solid is OFF, try turning solid ON
         solidDebugEnabled = true;
         std::cout << "Solid Debug: ON" << std::endl;
         if (fluidRenderer->isDebugVisualization()) fluidRenderer->toggleDebugVisualization();
    } else if (solidDebugEnabled && fluidRenderer) { // If solid is ON, try turning fluid ON (and solid OFF)
        solidDebugEnabled = false;
        fluidRenderer->toggleDebugVisualization();
        std::cout << "Solid Debug: OFF, Fluid Debug toggled." << std::endl;
    } else if (fluidRenderer) { // If solid is OFF and fluid might be ON
        fluidRenderer->toggleDebugVisualization();
        std::cout << "Fluid Debug toggled." << std::endl;
    } else { // Fallback if no fluid renderer
        solidDebugEnabled = !solidDebugEnabled;
        std::cout << "Solid Debug: " << (solidDebugEnabled ? "ON" : "OFF") << std::endl;
    }
}

// Check if any debug mode is active
bool PresentationManager::isDebugVisualization() const {
    bool fluidDebugActive = fluidRenderer ? fluidRenderer->isDebugVisualization() : false;
    return solidDebugEnabled || fluidDebugActive;
}

// Get UIRenderer instance
UIRenderer& PresentationManager::getUIRenderer() {
    if (!uiRenderer) throw std::runtime_error("UIRenderer not initialized!");
    return *uiRenderer;
} 