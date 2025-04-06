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
#include "core/profile.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <stdexcept> // For std::runtime_error
#include <SFML/Window/Mouse.hpp> // For sf::Mouse
#include <SFML/Graphics/Texture.hpp>
#include <SFML/Graphics/Sprite.hpp>

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
    , metalFluidTexture()
    , metalFluidSprite()
    , fluidTextureBuffer()
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
void PresentationManager::renderFrame(float actualFPS, float actualTPS) {
    PROFILE_SCOPE("PresentationManager::renderFrame");
    ECSSimulator& simulator = ECSSimulator::getInstance();
    const entt::registry& registry = simulator.getRegistry();

    // Tell FluidRenderer to render to its offscreen texture
    if (fluidRenderer) {
        fluidRenderer->render(registry); 
    }

    // Clear SFML window
    clear(); 

    // Render particles
    renderMetalFluidInternal(registry);
    renderSolidParticlesInternal(registry);
    renderGasParticlesInternal(registry);

    // Render UI & Stats on top
    renderStatsInternal(actualFPS, actualTPS);
    renderUI();

    // Present final frame
    present(); 
}

// Delegate solid particle rendering
void PresentationManager::renderSolidParticlesInternal(const entt::registry &registry) {
    PROFILE_SCOPE("PresentationManager::renderSolidParticlesInternal");
    if (solidRenderer) {
        solidRenderer->setDebugRendering(solidDebugEnabled);
        solidRenderer->render(window, registry, currentColorScheme);
    }
}
// Delegate gas particle rendering
void PresentationManager::renderGasParticlesInternal(const entt::registry &registry) {
    PROFILE_SCOPE("PresentationManager::renderGasParticlesInternal");
    if (gasRenderer) { gasRenderer->render(window, registry); }
}

// Rename and update stats rendering
void PresentationManager::renderStatsInternal(float actualFPS, float actualTPS) {
    if (!uiRenderer) return;

    // Only render stats if debug mode is active
    if (isDebugVisualization()) {
        // Get desired time scale
        ECSSimulator& simulator = ECSSimulator::getInstance();
        const entt::registry& registry = simulator.getRegistry();
        double desiredTimeScale = 1.0; // Default
        auto stView = registry.view<Components::SimulatorState>();
        if (!stView.empty()) {
            desiredTimeScale = registry.get<Components::SimulatorState>(stView.front()).timeScale;
        }

        // Calculate achieved time scale (normalized by target TPS)
        const float targetTPS = static_cast<float>(SimulatorConstants::StepsPerSecond); // Use constant
        double achievedTimeScale = (targetTPS > 0) ? (actualTPS / targetTPS) * desiredTimeScale : 0.0;

        std::stringstream ssFPS, ssTPS, ssAcc;
        ssFPS << std::fixed << std::setprecision(1) << actualFPS << " FPS";
        ssTPS << std::fixed << std::setprecision(1) << actualTPS << " TPS";
        ssAcc << "Acc: " << std::fixed << std::setprecision(2) << achievedTimeScale
              << "x (Tgt: " << desiredTimeScale << "x)";

        // Position the stats
        int yPos = 10;
        uiRenderer->renderText(window, ssFPS.str(), 10, yPos, sf::Color::White); yPos += 15;
        uiRenderer->renderText(window, ssTPS.str(), 10, yPos, sf::Color::White); yPos += 15;
        uiRenderer->renderText(window, ssAcc.str(), 10, yPos, sf::Color::White);
    }
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
         if (fluidRenderer->isDebugVisualization()) fluidRenderer->toggleDebugVisualization();
    } else if (solidDebugEnabled && fluidRenderer) { // If solid is ON, try turning fluid ON (and solid OFF)
        solidDebugEnabled = false;
        fluidRenderer->toggleDebugVisualization();
    } else if (fluidRenderer) { // If solid is OFF and fluid might be ON
        fluidRenderer->toggleDebugVisualization();
    } else { // Fallback if no fluid renderer
        solidDebugEnabled = !solidDebugEnabled;
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

// --- Add new internal helper method --- 
/**
 * @brief Reads the fluid texture from FluidRenderer and draws it.
 */
void PresentationManager::renderMetalFluidInternal(const entt::registry& registry)
{
    PROFILE_SCOPE("PresentationManager::renderMetalFluidInternal");
    if (!fluidRenderer) return;

    sf::Vector2u textureSize;
    if (fluidRenderer->readFluidTexture(fluidTextureBuffer, textureSize)) {
        if (metalFluidTexture.getSize() != textureSize) {
            if (!metalFluidTexture.create(textureSize.x, textureSize.y)) {
                std::cerr << "Error: Failed to create sf::Texture for fluid." << std::endl;
                return; // Exit if texture fails
            }
            // No log needed here
        }
        if (metalFluidTexture.getSize() == textureSize && !fluidTextureBuffer.empty()) {
            metalFluidTexture.update(fluidTextureBuffer.data());
            metalFluidSprite.setTexture(metalFluidTexture, true); 
            sf::View defaultView = window.getDefaultView(); 
            window.setView(defaultView);
            // Remove sprite logging
            window.draw(metalFluidSprite);
            // Remove draw log
        } 
        // Remove skip log
    }
} 