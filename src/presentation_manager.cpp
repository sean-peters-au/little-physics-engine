/**
 * @file presentation_manager.cpp
 * @brief Implementation of PresentationManager.
 */

#include "presentation_manager.hpp"
#include "renderers/solid_renderer.hpp"  // Include sub-renderers
#include "renderers/fluid_renderer.hpp"
#include "renderers/gas_renderer.hpp"
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

// --- Constructor / Destructor ---
PresentationManager::PresentationManager(int screenWidth, int screenHeight, const SharedSystemConfig& config)
    : initialized(false)
    , screenWidth(screenWidth)
    , screenHeight(screenHeight)
    , coordinates(config, SimulatorConstants::ScreenLength)
    // Initialize sub-renderers
    , solidRenderer(std::make_unique<SolidRenderer>(coordinates))
    , fluidRenderer(std::make_unique<FluidRenderer>(coordinates, sf::Vector2u(SimulatorConstants::ScreenLength, SimulatorConstants::ScreenLength)))
    , gasRenderer(std::make_unique<GasRenderer>(coordinates))
{
    // Font loading remains here as PresentationManager provides it
}

PresentationManager::~PresentationManager() = default;

// --- Core Methods Implementation ---
bool PresentationManager::init() {
    window.create(sf::VideoMode(screenWidth, screenHeight), "N-Body Simulator"); // Simplified title
    if (!font.loadFromFile("assets/fonts/arial.ttf")) {
        std::cerr << "Failed to load font assets/fonts/arial.ttf" << std::endl;
        return false;
    }
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

// --- Rendering Orchestration ---
void PresentationManager::renderSolidParticles(const entt::registry &registry) {
    if (solidRenderer) {
        solidRenderer->setDebugRendering(solidDebugEnabled); // Pass debug state
        solidRenderer->render(window, registry, currentColorScheme);
    }
}

void PresentationManager::renderFluidParticles(const entt::registry &registry) {
    if (fluidRenderer) {
        // Fluid renderer manages its own debug state via toggleDebugVisualization
        fluidRenderer->render(window, registry);
    }
}

void PresentationManager::renderGasParticles(const entt::registry &registry) {
    if (gasRenderer) {
        gasRenderer->render(window, registry);
    }
}

void PresentationManager::renderFPS(float fps) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1) << fps << " FPS";
    renderText(ss.str(), 10, 10, sf::Color::White);
}

// --- UI Drawing Primitives Implementation (Keep here for now) ---
void PresentationManager::renderText(const std::string &text, int x, int y, sf::Color color) {
    sf::Text sfText;
    sfText.setFont(font); // Use member font
    sfText.setString(text);
    sfText.setCharacterSize(12);
    sfText.setFillColor(color);
    sfText.setPosition(static_cast<float>(x), static_cast<float>(y));
    window.draw(sfText);
}

void PresentationManager::drawButton(const UIButton& button, sf::Color fillColor, sf::Color textColor) {
    sf::RectangleShape shape(sf::Vector2f(static_cast<float>(button.rect.width),
                                          static_cast<float>(button.rect.height)));
    shape.setPosition(static_cast<float>(button.rect.left),
                      static_cast<float>(button.rect.top));
    shape.setFillColor(fillColor);
    shape.setOutlineColor(sf::Color::White);
    shape.setOutlineThickness(1.f);
    window.draw(shape);

    if (!button.label.empty()) {
        renderText(button.label, button.rect.left + 5, button.rect.top + 3, textColor);
    }
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

// --- Helper Methods (Remove - Moved to sub-renderers) ---
// Remove the definitions of:
// - renderParticles
// - renderContactDebug
// - renderVelocityDebug
// - renderAngularDebug
// - renderPolygonDebug
// - aggregateParticlesByPixel

// Remove the definition for renderParticles
/*
void PresentationManager::renderParticles(const entt::registry &registry) { ... }
*/

// Remove the definition for renderContactDebug
/*
void PresentationManager::renderContactDebug(const entt::registry &registry) { ... }
*/

// Remove the definition for renderVelocityDebug
/*
void PresentationManager::renderVelocityDebug(const entt::registry &registry) { ... }
*/

// Remove the definition for renderAngularDebug
/*
void PresentationManager::renderAngularDebug(const entt::registry &registry) { ... }
*/

// Remove the definition for renderPolygonDebug
/*
void PresentationManager::renderPolygonDebug(const entt::registry &registry) { ... }
*/

// Remove the definition for aggregateParticlesByPixel
/*
std::unordered_map<std::pair<int,int>, PresentationManager::PixelProperties, PixelCoordHash>
PresentationManager::aggregateParticlesByPixel(const entt::registry &registry) { ... }
*/ 