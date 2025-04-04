/**
 * @fileoverview presentation_manager.hpp
 * @brief Manages the application window, event polling, and rendering orchestration.
 *
 * Owns the SFML window and delegates rendering tasks to specific sub-renderers
 * (Solid, Fluid, Gas, UI). Handles the main event loop and passes relevant UI
 * events to the EventManager.
 */

#pragma once

#include <entt/entt.hpp>
#include <string>
#include <functional>
#include <unordered_map>
#include <vector>
#include <memory>
#include <SFML/Graphics.hpp>
#include <SFML/Window/Event.hpp>

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
class UIRenderer;
class EventManager;

// Include the new types header
#include "renderer_types.hpp"

/**
 * @class PresentationManager
 * @brief Singleton responsible for window, events, and rendering pipeline.
 */
class PresentationManager {
public:
    // --- Remove moved type definitions ---
    // struct PixelProperties { ... };
    // using ColorMapper = std::function<sf::Color(const PixelProperties&)>;
    // struct UIButton { ... };
    // enum class ColorScheme { ... };

    // Delete copy/move constructors and assignment operators
    PresentationManager(const PresentationManager&) = delete;
    PresentationManager& operator=(const PresentationManager&) = delete;
    PresentationManager(PresentationManager&&) = delete;
    PresentationManager& operator=(PresentationManager&&) = delete;

    /** @brief Get the singleton instance. */
    static PresentationManager& getInstance();

    // --- Core Methods ---
    /** @brief Initializes the SFML window, loads fonts, creates sub-renderers. */
    bool init();
    /** @brief Polls SFML events and dispatches them (window close, EventManager). */
    void handleEvents();
    /** @brief Clears the render target (window). */
    void clear();
    /** @brief Displays the rendered frame on the window. */
    void present();
    /** @brief Provides access to the SFML window object. */
    sf::RenderWindow& getWindow() { return window; }
    /** @brief Provides access to the loaded font object. */
    const sf::Font& getFont() const { return font; }
    /** @brief Checks if the manager has been successfully initialized. */
    bool isInitialized() const { return initialized; }
    /** @brief Checks if the main application window is still open. */
    bool isWindowOpen() const { return window.isOpen(); }
    /** @brief Updates coordinate conversion parameters for itself and sub-renderers. */
    void updateCoordinates(const SharedSystemConfig& config);

    // --- Rendering Orchestration ---
    /** @brief Orchestrates the rendering of a single frame (particles, UI, FPS). */
    void renderFrame(float fps);

    /** @brief Calculates UI layout based on current state and draws it via UIRenderer. */
    void renderUI();

    // --- UI Drawing Primitives (REMOVED from public interface) ---
    // void renderText(const std::string& text, int x, int y, sf::Color color = sf::Color::White);
    // void drawButton(const UIButton& button, sf::Color fillColor, sf::Color textColor = sf::Color::White);

    // --- Add Getter for UIRenderer ---
    /** @brief Provides access to the UIRenderer instance. */
    UIRenderer& getUIRenderer();

    // --- State Management ---
    /** @brief Sets the current color scheme for rendering. */
    void setColorScheme(ColorScheme scheme);
    /** @brief Gets the current rendering color scheme. */
    ColorScheme getColorScheme() const;
    /** @brief Toggles debug visualization modes (solid, fluid). */
    void toggleDebugVisualization();
    /** @brief Checks if any debug visualization mode is currently active. */
    bool isDebugVisualization() const;

    // --- Static Color Mappers --- (Keep these accessible, use types from renderer_types.hpp)
    /** @brief Static Color Mapper for default color mapping. */
    static sf::Color defaultColorMapper(const PixelProperties& props);
    /** @brief Static Color Mapper for sleep color mapping. */
    static sf::Color sleepColorMapper(const PixelProperties& props);
    /** @brief Static Color Mapper for temperature color mapping. */
    static sf::Color temperatureColorMapper(const PixelProperties& props);

private:
    // --- Make constructor private ---
    PresentationManager(int screenWidth, int screenHeight, const SharedSystemConfig& config = SharedSystemConfig());
    ~PresentationManager(); // Default destructor ok

    // --- Static instance for singleton ---
    // (Initialization details handled in .cpp)

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

    // Sub-Renderers
    std::unique_ptr<SolidRenderer> solidRenderer;
    std::unique_ptr<FluidRenderer> fluidRenderer;
    std::unique_ptr<GasRenderer> gasRenderer;
    std::unique_ptr<UIRenderer> uiRenderer;

    // --- Members for Metal Fluid Compositing ---
    sf::Texture metalFluidTexture;
    sf::Sprite metalFluidSprite;
    std::vector<uint8_t> fluidTextureBuffer;

    // --- Helper Methods (Removed from here) ---
    // void renderContactDebug(const entt::registry &registry); // Moved
    // void renderVelocityDebug(const entt::registry &registry); // Moved
    // void renderAngularDebug(const entt::registry &registry); // Moved
    // void renderPolygonDebug(const entt::registry &registry); // Moved
    // std::unordered_map<std::pair<int,int>, PixelProperties, PixelCoordHash>
    // aggregateParticlesByPixel(const entt::registry& registry); // Moved/Replaced

    // 6. Add internal rendering helpers
    void renderSolidParticlesInternal(const entt::registry& registry);
    void renderFluidParticlesInternal(const entt::registry& registry);
    void renderGasParticlesInternal(const entt::registry& registry);
    void renderFPSInternal(float fps);

    // 7. Store UI Layout data
    std::vector<UIButton> currentButtonLayout;
    ButtonID highlightedButtonID = ButtonID::UNKNOWN;
}; 