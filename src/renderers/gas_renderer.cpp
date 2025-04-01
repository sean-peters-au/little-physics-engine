/**
 * @file gas_renderer.cpp
 * @brief Implementation of GasRenderer.
 */

#include "renderers/gas_renderer.hpp"
#include <cmath>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderTarget.hpp>
#include <iostream>

GasRenderer::GasRenderer(Simulation::Coordinates& coords)
    : coordinates(coords) {}

void GasRenderer::render(sf::RenderTarget& target, const entt::registry& registry)
{
    std::cout << "GasRenderer::render called." << std::endl;
    int gasDrawnCount = 0;

    // Correct View: Doesn't require sf::CircleShape component.
    // Requires Position, Shape, Color, and ParticlePhase.
    auto view = registry.view<Components::Position, Components::Shape, Components::Color, Components::ParticlePhase>();

    for (auto entity : view) {
        const auto &phase = view.get<Components::ParticlePhase>(entity);
        // Skip if not a gas particle
        if (phase.phase != Components::Phase::Gas) continue;

        // Get needed components (Shape type must be Circle)
        const auto &shape = view.get<Components::Shape>(entity);
        if (shape.type != Components::ShapeType::Circle) continue;

        const auto &position = view.get<Components::Position>(entity);
        const auto &color    = view.get<Components::Color>(entity);

        // Get radius from the Components::Shape size field (assuming this holds radius in meters)
        float pixelRadius = static_cast<float>(coordinates.metersToPixels(shape.size));
        // Ensure minimum visible size
        pixelRadius = std::max(1.0f, pixelRadius);

        float pixelX = static_cast<float>(coordinates.metersToPixels(position.x));
        float pixelY = static_cast<float>(coordinates.metersToPixels(position.y));

        std::cout << "  Drawing Gas Particle at (" << pixelX << ", " << pixelY << ") Radius: " << pixelRadius << " Color: (" << (int)color.r << "," << (int)color.g << "," << (int)color.b << ")" << std::endl;

        // Create the SFML drawable locally
        sf::CircleShape gasCircle(pixelRadius);
        // Use the particle's color but make it semi-transparent
        gasCircle.setFillColor(sf::Color(color.r, color.g, color.b, 180));
        gasCircle.setOrigin(pixelRadius, pixelRadius); // Center the origin
        gasCircle.setPosition(pixelX, pixelY);

        // Draw to the provided target
        target.draw(gasCircle);
        gasDrawnCount++;
    }
    if (gasDrawnCount > 0) {
        std::cout << "GasRenderer: Drew " << gasDrawnCount << " gas particles." << std::endl;
    } else {
         std::cout << "GasRenderer: No gas particles found/drawn in the correct view." << std::endl;
    }
} 