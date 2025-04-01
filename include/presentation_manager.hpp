/**
 * @file presentation_manager.hpp
 * @brief Manages rendering and UI display using SFML.
 *
 * Takes over responsibilities from the old Renderer class, focusing on:
 * - Window management
 * - Rendering orchestration (delegating to sub-renderers in the future)
 * - UI drawing primitives (buttons, text)
 * - Debug visualizations
 */

#pragma once

#include <entt/entt.hpp>
#include <string>
#include <functional>
#include <unordered_map>
#include <vector>
#include <memory>
#include <SFML/Graphics.hpp>

// Required component and system headers
#include "entities/entity_components.hpp"
#include "entities/sim_components.hpp"
#include "core/constants.hpp"
#include "core/coordinates.hpp"
#include "systems/i_system.hpp" // For SharedSystemConfig

// Forward declarations instead
class SolidRenderer;
class FluidRenderer;
class GasRenderer;

// Include the new types header
#include "renderer_types.hpp"

class PresentationManager {
public:
    // --- Remove moved type definitions ---
    // struct PixelProperties { ... };
    // using ColorMapper = std::function<sf::Color(const PixelProperties&)>;
    // struct UIButton { ... };
    // enum class ColorScheme { ... };

    // --- Constructor / Destructor ---
    PresentationManager(int screenWidth, int screenHeight,
                        const SharedSystemConfig& config = SharedSystemConfig());
    ~PresentationManager();

    // Prevent copying/moving
    PresentationManager(const PresentationManager&) = delete;
    PresentationManager& operator=(const PresentationManager&) = delete;
    PresentationManager(PresentationManager&&) = delete;
    PresentationManager& operator=(PresentationManager&&) = delete;

    // --- Core Methods ---
    bool init();
    void clear();
    void present();
    sf::RenderWindow& getWindow() { return window; }
    const sf::Font& getFont() const { return font; }
    bool isInitialized() const { return initialized; }
    bool isWindowOpen() const { return window.isOpen(); }
    void updateCoordinates(const SharedSystemConfig& config);

    // --- Rendering Orchestration ---
    void renderSolidParticles(const entt::registry& registry);
    void renderFluidParticles(const entt::registry& registry);
    void renderGasParticles(const entt::registry& registry);
    void renderFPS(float fps);

    // --- UI Drawing Primitives ---
    // Use types defined in renderer_types.hpp
    void renderText(const std::string& text, int x, int y, sf::Color color = sf::Color::White);
    void drawButton(const UIButton& button, sf::Color fillColor, sf::Color textColor = sf::Color::White);

    // --- State Management ---
    // Use ColorScheme from renderer_types.hpp
    void setColorScheme(ColorScheme scheme);
    ColorScheme getColorScheme() const;
    void toggleDebugVisualization();
    bool isDebugVisualization() const;

    // --- Static Color Mappers --- (Keep these accessible, use types from renderer_types.hpp)
    static sf::Color defaultColorMapper(const PixelProperties& props);
    static sf::Color sleepColorMapper(const PixelProperties& props);
    static sf::Color temperatureColorMapper(const PixelProperties& props);

private:
    // --- Member Variables ---
    sf::RenderWindow window;
    sf::Font font;
    bool initialized;
    int screenWidth;
    int screenHeight;
    Simulation::Coordinates coordinates; // Coordinate converter
    ColorScheme currentColorScheme = ColorScheme::DEFAULT; // Use enum from renderer_types.hpp
    // Debug state - maybe split later
    bool solidDebugEnabled = false;
    // bool fluidDebugEnabled = false; // FluidRenderer manages its own internal state
    // bool gasDebugEnabled = false;

    // Sub-Renderers (using forward-declared types is fine for unique_ptr)
    std::unique_ptr<SolidRenderer> solidRenderer;
    std::unique_ptr<FluidRenderer> fluidRenderer;
    std::unique_ptr<GasRenderer> gasRenderer;

    // --- Helper Methods (Removed from here) ---
    // void renderContactDebug(const entt::registry &registry); // Moved
    // void renderVelocityDebug(const entt::registry &registry); // Moved
    // void renderAngularDebug(const entt::registry &registry); // Moved
    // void renderPolygonDebug(const entt::registry &registry); // Moved
    // std::unordered_map<std::pair<int,int>, PixelProperties, PixelCoordHash>
    // aggregateParticlesByPixel(const entt::registry& registry); // Moved/Replaced
}; 