/**
 * @file presentation_manager.cpp
 * @brief Implementation of PresentationManager.
 */

#include "presentation_manager.hpp"
#include "renderers/solid_renderer.hpp"  // Include sub-renderers
#include "renderers/fluid_renderer.hpp"
#include "renderers/gas_renderer.hpp"
#include "renderers/ui_renderer.hpp" // Include UIRenderer header
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

// Required components/utils
#include "entities/entity_components.hpp"
#include "entities/sim_components.hpp"
#include "core/constants.hpp"
#include "math/constants.hpp"
#include "math/polygon.hpp"
// Remove headers only needed by moved methods if not used elsewhere here
// #include "systems/rigid/contact_manager.hpp"
// #include "arch/native/renderer_fluid_dsf.hpp"

// EventManager and SimManager
#include "event_manager.hpp" // Include EventManager for calling processEvent
#include "sim.hpp" // Include for ECSSimulator access (registry)
#include "sim_manager.hpp" // Include for SimManager access (paused state, current scenario)

// --- PixelProperties Method Implementation (Remove - defined in renderer_types.hpp) ---
/*
void PresentationManager::PixelProperties::add(...) { ... }
*/

// --- Static Color Mappers (Keep here) ---
sf::Color PresentationManager::defaultColorMapper(const PixelProperties& /* props */) {
    return sf::Color::White;
}

sf::Color PresentationManager::sleepColorMapper(const PixelProperties& props) {
    if (props.particle_count == 0) { return sf::Color::White; }
    return props.is_asleep ? sf::Color(200, 50, 50) : sf::Color(50, 200, 50);
}

sf::Color PresentationManager::temperatureColorMapper(const PixelProperties& props) {
    if (props.particle_count == 0 || !props.has_temperature) {
        return {128, 128, 128};
    }

    const double minTemp = 0.0; // Adjust range as needed
    const double maxTemp = 100.0;
    double t = (props.temperature - minTemp) / (maxTemp - minTemp);
    t = std::max(0.0, std::min(1.0, t)); // Clamp to [0, 1]

    auto const r = static_cast<uint8_t>(255 * t);
    auto const b = static_cast<uint8_t>(255 * (1.0 - t));
    return {r, 0, b};
}

// Singleton instance getter implementation
PresentationManager& PresentationManager::getInstance() {
    // Initialize with default screen dimensions and config
    // NOTE: This assumes a fixed initial size. If size needs to be dynamic,
    // a separate initialization method for the singleton might be needed.
    static PresentationManager instance(
        SimulatorConstants::ScreenLength + 200, // width
        SimulatorConstants::ScreenLength,      // height
        SharedSystemConfig()                  // default config
    );
    return instance;
}

// Constructor implementation (now private)
PresentationManager::PresentationManager(int screenWidth, int screenHeight, const SharedSystemConfig& config)
    : initialized(false)
    , screenWidth(screenWidth)
    , screenHeight(screenHeight)
    , coordinates(config, SimulatorConstants::ScreenLength)
    // Initialize sub-renderers
    , solidRenderer(std::make_unique<SolidRenderer>(coordinates))
    , fluidRenderer(std::make_unique<FluidRenderer>(coordinates, sf::Vector2u(SimulatorConstants::ScreenLength, SimulatorConstants::ScreenLength)))
    , gasRenderer(std::make_unique<GasRenderer>(coordinates))
    // uiRenderer is initialized after font is loaded in init()
{
    // Font loading moved to init()
}

// Destructor implementation (can be default)
PresentationManager::~PresentationManager() = default;

// --- Core Methods Implementation ---
bool PresentationManager::init() {
    window.create(sf::VideoMode(screenWidth, screenHeight), "N-Body Simulator");
    if (!font.loadFromFile("assets/fonts/arial.ttf")) {
        std::cerr << "Failed to load font assets/fonts/arial.ttf" << std::endl;
        return false;
    }
    // Initialize UIRenderer *after* font is loaded
    uiRenderer = std::make_unique<UIRenderer>(font);

    initialized = true;
    // Initialize sub-renderers if they need it (e.g., load shaders)
    // if (solidRenderer) solidRenderer->init(); // If needed
    // if (fluidRenderer) fluidRenderer->init(); // If needed
    // if (gasRenderer) gasRenderer->init();   // If needed
    return true;
}

void PresentationManager::clear() {
    window.clear(sf::Color::Black);
}

void PresentationManager::present() {
    window.display();
}

void PresentationManager::updateCoordinates(const SharedSystemConfig& config) {
    coordinates.updateConfig(config);
    // Potentially notify sub-renderers if they cache coordinate info
    // if (solidRenderer) solidRenderer->onCoordinateUpdate();
    // if (fluidRenderer) fluidRenderer->onCoordinateUpdate();
    // if (gasRenderer) gasRenderer->onCoordinateUpdate();
}

// --- Rendering Orchestration Implementation ---
void PresentationManager::renderFrame(float fps) {
    // Get necessary state from other singletons
    ECSSimulator& simulator = ECSSimulator::getInstance();
    // SimManager& simManager = SimManager::getInstance(); // Remove unused variable
    const entt::registry& registry = simulator.getRegistry();

    clear();

    // Call internal render helpers
    renderSolidParticlesInternal(registry);
    renderFluidParticlesInternal(registry);
    renderGasParticlesInternal(registry);
    renderFPSInternal(fps);

    renderUI();

    present();
}

// Implement internal rendering methods (just delegate)
void PresentationManager::renderSolidParticlesInternal(const entt::registry &registry) {
    if (solidRenderer) {
        solidRenderer->setDebugRendering(solidDebugEnabled);
        solidRenderer->render(window, registry, currentColorScheme);
    }
}
void PresentationManager::renderFluidParticlesInternal(const entt::registry &registry) {
    if (fluidRenderer) {
        fluidRenderer->render(window, registry);
    }
}
void PresentationManager::renderGasParticlesInternal(const entt::registry &registry) {
    if (gasRenderer) {
        gasRenderer->render(window, registry);
    }
}
void PresentationManager::renderFPSInternal(float fps) {
    if (!uiRenderer) return;
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1) << fps << " FPS";
    // Fix: Call renderText on uiRenderer instance
    uiRenderer->renderText(window, ss.str(), 10, 10, sf::Color::White);
}

// --- UI Layout Calculation & Drawing ---
void PresentationManager::renderUI() {
    if (!uiRenderer) return; // Need UIRenderer to draw

    SimManager& simManager = SimManager::getInstance(); // Needed for state
    ECSSimulator& simulator = ECSSimulator::getInstance();

    // Get state needed for UI
    bool paused = simManager.isPaused(); // Add isPaused() getter to SimManager
    SimulatorConstants::SimulationType currentScenario = simManager.getCurrentScenarioType(); // Add getter to SimManager
    const auto& scenarioList = simManager.getScenarioList(); // Add getter to SimManager
    const entt::registry& registry = simulator.getRegistry();

    // Get current highlight state from EventManager
    highlightedButtonID = EventManager::getInstance().getHighlightedButtonID(); // Add getter to EventManager

    currentButtonLayout.clear(); // Clear previous layout

    int const panelX = static_cast<int>(SimulatorConstants::ScreenLength) + 10;
    int panelY = 10;

    // --- Calculate and Draw Buttons (Logic moved from UIManager::renderUI) ---

    // 1) Pause/Play
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

    // 2) Next Frame
    {
        UIButton btn;
        btn.rect = sf::IntRect(panelX, panelY, 80, 20);
        btn.label = "Next Frame";
        btn.isSpecialButton = true;
        btn.id = ButtonID::NEXT_FRAME;
        sf::Color bgColor = paused ? sf::Color(100,100,100) : sf::Color(50,50,50);
        sf::Color fgColor = paused ? sf::Color::White : sf::Color(150,150,150);
        // No highlight state for disabled button?
        uiRenderer->drawButton(window, btn, bgColor, fgColor);
        currentButtonLayout.push_back(btn);
        panelY += 25;
    }

    // 3) Reset
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

    // 4) Speed Controls
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
            if (simState && (std::fabs(simState->timeScale - sp.first) < 0.01)) {
                color = sf::Color(0,200,0); // Active color overrides highlight
            }
            uiRenderer->drawButton(window, btn, color);
            currentButtonLayout.push_back(btn);
            panelY += 25;
        }
    }
    panelY += 20;

    // 5) Color Scheme
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
            if (activeScheme == scheme.first) {
                color = sf::Color(0,200,0); // Active color overrides highlight
            }
            uiRenderer->drawButton(window, btn, color);
            currentButtonLayout.push_back(btn);
            panelY += 25;
        }
    }
    panelY += 20;

    // 6) Debug Toggle
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

    // 7) Scenarios
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
        if (isCurrent) {
            color = sf::Color(0,200,0);
        }
        uiRenderer->drawButton(window, btn, color);
        currentButtonLayout.push_back(btn);
        panelY += 25;
    }

    // Pass the calculated layout to EventManager for next frame's hit detection
    EventManager::getInstance().updateLayout(currentButtonLayout);
}

// --- State Management Implementation ---
void PresentationManager::setColorScheme(ColorScheme scheme) {
    currentColorScheme = scheme;
}

ColorScheme PresentationManager::getColorScheme() const {
    return currentColorScheme;
}

void PresentationManager::toggleDebugVisualization() {
    // Cycle through different debug aspects
    // Example: Toggle solid debug, then fluid debug
    if (!solidDebugEnabled && fluidRenderer) { // If solid is OFF, try turning solid ON
         solidDebugEnabled = true;
         std::cout << "Solid Debug: ON" << std::endl;
         // Ensure fluid debug is off if cycling
         if (fluidRenderer->isDebugVisualization()) fluidRenderer->toggleDebugVisualization();
    } else if (solidDebugEnabled && fluidRenderer) { // If solid is ON, try turning fluid ON (and solid OFF)
        solidDebugEnabled = false;
        fluidRenderer->toggleDebugVisualization(); // This will toggle it ON if it was OFF
        std::cout << "Solid Debug: OFF, Fluid Debug toggled." << std::endl;
    } else if (fluidRenderer) { // If solid is OFF and fluid might be ON
        fluidRenderer->toggleDebugVisualization(); // Toggle fluid (might turn it OFF)
        std::cout << "Fluid Debug toggled." << std::endl;
    } else { // Fallback if no fluid renderer
        solidDebugEnabled = !solidDebugEnabled;
        std::cout << "Solid Debug: " << (solidDebugEnabled ? "ON" : "OFF") << std::endl;
    }
}

bool PresentationManager::isDebugVisualization() const {
    // Return true if any debug mode is active
    bool fluidDebugActive = fluidRenderer ? fluidRenderer->isDebugVisualization() : false;
    return solidDebugEnabled || fluidDebugActive;
}

// --- Event Handling Implementation ---
void PresentationManager::handleEvents() {
    sf::Event event;
    while (window.pollEvent(event)) {
        // Handle window close directly
        if (event.type == sf::Event::Closed) {
            window.close();
            return; // Stop processing events if window closed
        }

        // Pass other events to EventManager
        EventManager::getInstance().processEvent(event);

        // EventManager now handles mouse move highlights internally,
        // but we might need its result for rendering. Let's assume EventManager
        // updates highlightedButtonID internally, and renderUI reads it.
    }

    // Update highlight state AFTER processing all events for the frame
    // This replaces UIManager::updateHighlights
    sf::Vector2i mousePos = sf::Mouse::getPosition(window);
    EventManager::getInstance().handleMouseMoved(mousePos.x, mousePos.y);
}

// --- Add Getter Implementation ---
UIRenderer& PresentationManager::getUIRenderer() {
    if (!uiRenderer) {
        throw std::runtime_error("UIRenderer not initialized!");
    }
    return *uiRenderer;
} 