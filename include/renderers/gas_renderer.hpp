/**
 * @file gas_renderer.hpp
 * @brief Handles rendering of gas particles.
 */
#pragma once

#include <entt/entt.hpp>
#include <SFML/Graphics.hpp>

#include "core/coordinates.hpp"
#include "entities/entity_components.hpp"

class GasRenderer {
public:
    /**
     * @brief Constructor.
     * @param coordinates Reference to the coordinate conversion utility.
     */
    explicit GasRenderer(Simulation::Coordinates& coordinates);

    /**
     * @brief Renders all gas particles onto the given render target.
     * @param target The SFML render target.
     * @param registry The ECS registry.
     */
    void render(sf::RenderTarget& target, const entt::registry& registry);

private:
    Simulation::Coordinates& coordinates; // Reference, not owned
}; 