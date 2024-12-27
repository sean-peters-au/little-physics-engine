/**
 * @file renderer.hpp
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

#ifndef RENDERER_H
#define RENDERER_H

#include <entt/entt.hpp>
#include <string>
#include <functional>
#include <unordered_map>
#include <vector>
#include <SFML/Graphics.hpp>

#include "nbody/components/basic.hpp"
#include "nbody/components/sim.hpp"
#include "nbody/core/constants.hpp"

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
        return h1 ^ (h2 + 0x9e3779b97f4a7c16ULL + (h1<<6) + (h1>>2));
    }
};

class Renderer {
public:
    /**
     * @brief Properties for aggregated particles in a pixel
     * 
     * Stores accumulated physical properties of all particles that map
     * to the same screen pixel. Used for density visualization and
     * temperature-based coloring.
     */
    struct PixelProperties {
        double density = 0.0;      ///< Total density at this pixel
        double temperature = 0.0;  ///< Mass-weighted average temperature
        double total_mass = 0.0;   ///< Sum of particle masses
        int particle_count = 0;    ///< Number of particles in pixel

        /**
         * @brief Adds a particle's properties to the pixel
         * @param density Particle density component (optional)
         * @param temp Particle temperature component (optional)
         * @param mass Particle mass component (optional)
         */
        void add(const Components::Density* density,
                 const Components::Temperature* temp,
                 const Components::Mass* mass) {
            if (density) this->density += density->value;
            if (temp) {
                if (mass) {
                    double weight = mass->value;
                    this->temperature = (this->temperature * total_mass + temp->value * weight) / (total_mass + weight);
                } else {
                    this->temperature = (this->temperature * particle_count + temp->value) / (particle_count + 1);
                }
            }
            if (mass) this->total_mass += mass->value;
            particle_count++;
        }
    };

    /** @brief Function type for mapping pixel properties to colors */
    using ColorMapper = std::function<sf::Color(const PixelProperties&)>;
    
    /** @brief Default color mapper that returns white */
    static sf::Color whiteColor(const PixelProperties&) { return sf::Color::White; }

    /**
     * @brief Constructs renderer with specified screen dimensions
     * @param screenWidth Width of render window in pixels
     * @param screenHeight Height of render window in pixels
     */
    Renderer(int screenWidth, int screenHeight);
    ~Renderer();

    /**
     * @brief Initializes SFML window and loads resources
     * @return true if initialization successful, false otherwise
     */
    bool init();

    /** @brief Clears screen to black */
    void clear();

    /** @brief Presents rendered frame to screen */
    void present();

    /**
     * @brief Renders all particles in the simulation
     * @param registry EnTT registry containing entities
     * @param colorMapper Function to determine particle colors (defaults to white)
     */
    void renderParticles(const entt::registry& registry, ColorMapper colorMapper = whiteColor);

    /**
     * @brief Renders FPS counter in top-left corner
     * @param fps Current frames per second
     */
    void renderFPS(float fps);

    /**
     * @brief Renders UI elements including buttons and scenario selection
     * 
     * Draws:
     * - Pause/Play button
     * - Reset button
     * - Speed control buttons (0.25x, 0.5x, 1x)
     * - Scenario selection buttons
     * 
     * @param registry EnTT registry for simulator state
     * @param paused Current pause state
     * @param currentScenario Active scenario type
     * @param scenarios Available scenario options
     * @param showPausePlayHighlight Highlight state for pause/play
     * @param showResetHighlight Highlight state for reset
     * @param highlightedScenario Currently highlighted scenario
     */
    void renderUI(const entt::registry& registry, 
                  bool paused, 
                  SimulatorConstants::SimulationType currentScenario,
                  const std::vector<std::pair<SimulatorConstants::SimulationType,std::string>>& scenarios,
                  bool showPausePlayHighlight,
                  bool showResetHighlight,
                  SimulatorConstants::SimulationType highlightedScenario);

    /**
     * @brief Renders text at specified position
     * @param text String to render
     * @param x X coordinate in pixels
     * @param y Y coordinate in pixels
     * @param color Text color (defaults to white)
     */
    void renderText(const std::string& text, int x, int y, sf::Color color = sf::Color::White);

    /** @brief Returns initialization state */
    bool isInitialized() const { return initialized; }

    /**
     * @brief Button representation for UI elements
     * 
     * Used for both scenario selection and control buttons (play/pause, reset).
     * Stores position, label, and type information for each button.
     */
    struct UIButton {
        sf::IntRect rect;          ///< Button bounds in pixels
        std::string label;         ///< Button text
        bool isSpecialButton;      ///< True for control buttons, false for scenarios
        SimulatorConstants::SimulationType scenario;  ///< Associated scenario type
        double speedMultiplier;    ///< Time scale multiplier (for speed buttons)

        UIButton() 
            : rect()
            , label()
            , isSpecialButton(false)
            , scenario(SimulatorConstants::SimulationType::CELESTIAL_GAS)
            , speedMultiplier(1.0)
        {}
    };

    std::vector<UIButton> scenarioButtons;  ///< List of scenario selection buttons
    UIButton pausePlayButton;               ///< Play/Pause toggle button
    UIButton resetButton;                   ///< Reset simulation button
    std::vector<UIButton> speedButtons;     ///< Time scale control buttons

    /** @brief Access to underlying SFML window */
    sf::RenderWindow& getWindow() { return window; }

private:
    sf::RenderWindow window;
    sf::Font font;
    bool initialized;
    int screenWidth;
    int screenHeight;

    /**
     * @brief Groups particles by screen pixel for density visualization
     * @param registry EnTT registry containing particle entities
     * @return Map of pixel coordinates to aggregated particle properties
     */
    std::unordered_map<std::pair<int,int>, PixelProperties, PixelCoordHash> 
    aggregateParticlesByPixel(const entt::registry& registry);
};

#endif