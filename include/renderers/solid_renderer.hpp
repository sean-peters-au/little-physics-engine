/**
 * @file solid_renderer.hpp
 * @brief Handles rendering of solid particles (polygons, circles, squares).
 */
#pragma once

#include <entt/entt.hpp>
#include <SFML/Graphics.hpp>
#include <string>
#include <functional>
#include <unordered_map>

#include "core/coordinates.hpp"
#include "entities/entity_components.hpp"
#include "renderer_types.hpp"

class SolidRenderer {
public:
    /**
     * @brief Constructor.
     * @param coordinates Reference to the coordinate conversion utility.
     */
    explicit SolidRenderer(Simulation::Coordinates& coordinates);

    /**
     * @brief Renders all solid particles onto the given render target.
     * @param target The SFML render target (usually the window).
     * @param registry The ECS registry containing simulation entities.
     * @param colorScheme The currently selected color scheme to use.
     */
    void render(sf::RenderTarget& target,
                const entt::registry& registry,
                ColorScheme colorScheme);

    /** @brief Sets whether debug visualizations should be rendered. */
    void setDebugRendering(bool enabled) { debugEnabled = enabled; }

private:
    Simulation::Coordinates& coordinates; // Reference, not owned
    bool debugEnabled = false;

    // --- Helper Methods (Implementations in .cpp) ---

    /** @brief Helper to aggregate particle properties per pixel for coloring. */
    std::unordered_map<std::pair<int,int>, PixelProperties, PixelCoordHash>
    aggregateSolidParticlesByPixel(const entt::registry& registry);

    // Debug rendering helpers (to be moved from PresentationManager)
    void renderContactDebug(sf::RenderTarget& target, const entt::registry& registry);
    void renderVelocityDebug(sf::RenderTarget& target, const entt::registry& registry);
    void renderAngularDebug(sf::RenderTarget& target, const entt::registry& registry);
    void renderPolygonDebug(sf::RenderTarget& target, const entt::registry& registry);

     // Helper to render text (needed for debug)
    void renderText(sf::RenderTarget& target, const sf::Font& font, const std::string& text, int x, int y, sf::Color color, unsigned int size = 12);
}; 