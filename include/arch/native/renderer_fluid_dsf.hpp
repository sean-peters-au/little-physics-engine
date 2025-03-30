/**
 * @file renderer_fluid_dsf.hpp
 * @brief Screen-space fluid renderer for 2D simulation
 * 
 * Implements fluid rendering using a blurred density field processed
 * by a screen-space shader for high-quality visuals.
 */

#pragma once

#include <entt/entt.hpp>
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>

#include "core/coordinates.hpp"

// Forward declare for pointer member
namespace sf { class Shader; }

/**
 * @class FluidSurfaceRenderer
 * @brief Handles density grid generation, blurring, and screen-space shader rendering
 */
class FluidSurfaceRenderer {
public:
    // --- Debug Visualization --- 
    enum class DebugMode {
        NONE,                // Normal rendering
        DENSITY_PRE_BLUR,    // Show density grid before blurring
        DENSITY_POST_BLUR,   // Show density grid after blurring
    };

    /**
     * @brief Sets the debug visualization mode.
     * @param mode The debug mode to activate.
     */
    void setDebugMode(DebugMode mode) { currentDebugMode = mode; }

    /**
     * @brief Gets the current debug visualization mode.
     */
    DebugMode getCurrentDebugMode() const { return currentDebugMode; }

    /**
     * @brief Constructor
     * @param resolution Render resolution
     * @param coordinates Coordinate transformer
     */
    FluidSurfaceRenderer(sf::Vector2u resolution, const Simulation::Coordinates& coordinates);
    
    /**
     * @brief Destructor
     */
    ~FluidSurfaceRenderer();
    
    /** 
     * @brief Update the internal density grid based on particle positions 
     *        and update the density texture for the shader.
     */
    void updateGridAndTexture(const entt::registry& registry, 
                             int gridSize = 60,
                             float smoothingRadius = 6.0f);
    
    /** 
     * @brief Render the fluid using screen-space shader 
     * @param target SFML render target
     * @param surfaceColor Base color (tint)
     * @param densityThreshold Shader uniform for surface threshold
     * @param edgeSmoothness Shader uniform for edge softness
     */
    void render(sf::RenderTarget& target, 
               const sf::Color& surfaceColor = sf::Color(40, 130, 240, 220),
               float densityThreshold = 0.15f, 
               float edgeSmoothness = 0.02f); // Use a small smoothness for sharp edges
    
    /** @brief Check if the renderer is properly initialized */
    bool isInitialized() const {
        return !grid.empty() && densityTexture.getSize().x > 0;
    }

private:
    // --- Debug Visualization Data ---
    DebugMode currentDebugMode = DebugMode::NONE;
    std::vector<std::vector<float>> debugDensityGrid; // Keep for density viz
    // --- End Debug Visualization Data ---

    /** @brief Grid cell stores only density now */
    struct GridCell {
        float density = 0.0f;
    };
    
    // Internal rendering resources
    sf::Vector2u resolution; // Simulation area resolution
    const Simulation::Coordinates& coordinates;
    std::vector<std::vector<GridCell>> grid; 
    
    std::unique_ptr<sf::Shader> fluidShader; // Screen-space shader
    sf::RenderTexture densityTexture; // Holds final blurred density (RGBA, density in Alpha)
    
    // Configuration parameters (simplified)
    int lastGridSize = 0;
    float lastRadius = 0.0f;
    
    /** @brief Generate the density grid from fluid particles */
    void generateDensityGrid(const entt::registry& registry, int gridSize, float smoothingRadius);
    /** @brief Apply a simple Box Blur to the density grid */
    void blurDensityGrid();
    /** @brief Update the density texture from the grid data */
    void updateDensityTexture();
    /** @brief Initialize shader */
    void initializeShader();
    /** @brief SPH Poly6 Kernel */
    float kernelPoly6(float r, float h);
}; 