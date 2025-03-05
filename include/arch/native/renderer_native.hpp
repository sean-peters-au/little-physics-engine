/**
 * @file renderer_native.hpp
 * @brief Graphics rendering system using SFML
 *
 * This system handles:
 * - Particle rendering with density/temperature visualization
 * - Shape rendering (circles, squares, polygons)
 * - UI elements (buttons, text, FPS counter)
 * - Pixel-space aggregation for particle effects
 * 
 * Uses SFML for window management and drawing operations.
 */

#pragma once

#include <entt/entt.hpp>
#include <string>
#include <functional>
#include <unordered_map>
#include <vector>
#include <SFML/Graphics.hpp>
#include <memory>

#include "entities/entity_components.hpp"
#include "entities/sim_components.hpp"
#include "core/constants.hpp"
#include "core/coordinates.hpp"

/**
 * @brief Hash function for pixel coordinates
 * 
 * Used for efficient storage and lookup of particle properties in pixel-space.
 * Combines x,y coordinates into a single hash using bit operations.
 */
struct PixelCoordHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        std::size_t h1 = std::hash<int>()(p.first);
        std::size_t h2 = std::hash<int>()(p.second);
        // A typical combine approach:
        return h1 ^ (h2 + 0x9e3779b97f4a7c16ULL + (h1 << 6) + (h1 >> 2));
    }
};

/**
 * @class Renderer
 * @brief Manages rendering of simulation entities, UI, and aggregated visuals
 */
class Renderer {
public:
    /**
     * @brief Stores aggregated physical properties of all particles in a given screen pixel
     */
    struct PixelProperties {
        double density = 0.0;      ///< Total density at this pixel
        double temperature = 0.0;  ///< Mass-weighted average temperature
        double total_mass = 0.0;   ///< Sum of particle masses
        int particle_count = 0;    ///< Number of particles in pixel
        bool is_asleep = false;
        bool has_temperature = false;

        /**
         * @brief Adds a single particle's contribution to this pixel
         */
        void add(const Components::Density* dens,
                 const Components::Temperature* temp,
                 const Components::Mass* mass,
                 const Components::Sleep* sleep)
        {
            if (dens) {
                density += dens->value;
            }
            if (temp) {
                // If we have mass, weight the temperature by mass
                if (mass) {
                    double w = mass->value;
                    temperature = (temperature * total_mass + temp->value * w) / (total_mass + w);
                } else {
                    // fallback, average by count
                    temperature = (temperature * particle_count + temp->value) / (particle_count + 1);
                }
            }
            if (mass) {
                total_mass += mass->value;
            }
            if (sleep) {
                is_asleep = sleep->asleep;
            }
            if (temp) {
                has_temperature = true;
            }
            particle_count++;
        }
    };

    /** Function type for mapping pixel properties to an sf::Color. */
    using ColorMapper = std::function<sf::Color(const PixelProperties&)>;

    /** Default color mapper that always returns white. */
    static sf::Color whiteColor(const PixelProperties&) { return sf::Color::White; }

    /**
     * @brief Constructs renderer with given screen dimensions
     * @param screenWidth Width of the window
     * @param screenHeight Height of the window
     * @param config System configuration for coordinate conversion
     */
    Renderer(int screenWidth, int screenHeight, 
             const SystemConfig& config = SystemConfig());
    ~Renderer();

    /**
     * @brief Initializes SFML window and loads font
     * @return true if success, false otherwise
     */
    bool init();

    /** Clears the screen to black */
    void clear();

    /** Presents the rendered frame to display */
    void present();

    /**
     * @brief Renders all particles in the simulation
     * @param registry ECS registry containing entities/components
     * @param colorMapper Optional function to color them by density or temperature
     */
    void renderParticles(const entt::registry& registry);

    void renderSolidParticles(const entt::registry& registry);
    void renderFluidParticles(const entt::registry& registry);
    void renderGasParticles(const entt::registry& registry);

    /**
     * @brief Renders the current FPS in top-left corner
     * @param fps The frames-per-second value
     */
    void renderFPS(float fps);

    /**
     * @brief Renders the UI (buttons, scenario list, speed controls, etc.)
     */
    void renderUI(const entt::registry& registry,
                  bool paused,
                  SimulatorConstants::SimulationType currentScenario,
                  const std::vector<std::pair<SimulatorConstants::SimulationType,std::string>>& scenarios,
                  bool showPausePlayHighlight,
                  bool showResetHighlight,
                  SimulatorConstants::SimulationType highlightedScenario);

    /**
     * @brief Renders text at an (x,y) in the window
     */
    void renderText(const std::string& text, int x, int y, sf::Color color = sf::Color::White);

    /** Returns the SFML window so external code can process events, etc. */
    sf::RenderWindow& getWindow() { return window; }

    /** @brief True if we successfully called init() */
    bool isInitialized() const { return initialized; }

    /**
     * @brief UI button definition for scenario selection or speed toggles
     */
    struct UIButton {
        sf::IntRect rect;   ///< The bounding rect in screen coords
        std::string label;  ///< The button text
        bool isSpecialButton;   ///< Distinguish scenario vs special (play/pause, reset, speed)
        SimulatorConstants::SimulationType scenario; 
        double speedMultiplier;

        UIButton()
            : rect()
            , label()
            , isSpecialButton(false)
            , scenario(SimulatorConstants::SimulationType::KEPLERIAN_DISK)
            , speedMultiplier(1.0)
        {}
    };

    // For external code to read the current UI buttons
    std::vector<UIButton> scenarioButtons;
    UIButton pausePlayButton;
    UIButton resetButton;
    UIButton nextFrameButton;
    std::vector<UIButton> speedButtons;

    enum class ColorScheme {
        DEFAULT,    // Use entity's color component
        SLEEP,      // Green for awake, red for sleeping
        TEMPERATURE // Blue -> Red temperature gradient
    };

    ColorScheme currentColorScheme = ColorScheme::DEFAULT;

    void setColorScheme(ColorScheme scheme) { currentColorScheme = scheme; }
    ColorScheme getColorScheme() const { return currentColorScheme; }

    // Add to store color scheme buttons like we do for speed buttons
    std::vector<UIButton> colorSchemeButtons;

    // Add these static color mappers:
    static sf::Color defaultColorMapper(const PixelProperties& props);
    static sf::Color sleepColorMapper(const PixelProperties& props);
    static sf::Color temperatureColorMapper(const PixelProperties& props);

    /**
     * @brief Draws a rectangular button using SFML shapes. 
     * @param button The button rect/label info.
     * @param fillColor The fill color for the rectangle.
     * @param textColor The text color. Defaults to white.
     */
    void drawButton(const UIButton& button, sf::Color fillColor, sf::Color textColor = sf::Color::White);

    /**
     * @brief Toggles the debug visualization on/off.
     */
    void toggleDebugVisualization();

    /**
     * @brief Returns whether debug visualization is currently enabled.
     */
    bool isDebugVisualization() const;

    /**
     * @brief Updates coordinate conversion parameters based on new config
     * @param config The new system configuration
     */
    void updateCoordinates(const SystemConfig& config);

private:
    /** The SFML render window */
    sf::RenderWindow window;
    /** The font for text rendering */
    sf::Font font;
    /** Whether we've successfully inited SFML */
    bool initialized;
    /** Dimensions of the window */
    int screenWidth;
    int screenHeight;

    // Added member: fluidShader for fluid metaballs rendering.
    std::unique_ptr<sf::Shader> fluidShader;

    bool debugVisualization = false;
    UIButton debugButton;
    
    void renderContactDebug(const entt::registry &registry);
    void renderVelocityDebug(const entt::registry &registry);
    void renderAngularDebug(const entt::registry &registry);
    void renderPolygonDebug(const entt::registry &registry);

    // Add coordinates converter
    Simulation::Coordinates coordinates;

    /**
     * @brief Builds a map from pixel -> aggregated properties
     */
    std::unordered_map<std::pair<int,int>, PixelProperties, PixelCoordHash>
    aggregateParticlesByPixel(const entt::registry& registry);
};