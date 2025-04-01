/**
 * @file fluid_renderer.hpp
 * @brief Handles rendering of fluid particles, potentially wrapping FluidSurfaceRenderer.
 */
#pragma once

#include <entt/entt.hpp>
#include <SFML/Graphics.hpp>
#include <memory>

#include "core/coordinates.hpp"
#include "arch/native/renderer_fluid_dsf.hpp" // Include the actual surface renderer

class FluidRenderer {
public:
    /**
     * @brief Constructor.
     * @param coordinates Reference to the coordinate conversion utility.
     * @param screenDims Dimensions of the simulation screen area.
     */
    FluidRenderer(Simulation::Coordinates& coordinates, sf::Vector2u screenDims);
    ~FluidRenderer();

    /**
     * @brief Renders fluid particles onto the given render target.
     * @param target The SFML render target.
     * @param registry The ECS registry.
     */
    void render(sf::RenderTarget& target, const entt::registry& registry);

    /** @brief Toggles the internal fluid debug visualization modes. */
    void toggleDebugVisualization();

    /** @brief Checks if any fluid debug mode is active. */
    bool isDebugVisualization() const;

private:
    Simulation::Coordinates& coordinates; // Reference, not owned
    std::unique_ptr<FluidSurfaceRenderer> surfaceRenderer; // Owns the surface renderer
    // Add fallback shader if needed for metaballs?
    // std::unique_ptr<sf::Shader> metaballShader;
}; 