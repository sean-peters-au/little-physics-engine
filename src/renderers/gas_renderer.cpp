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

    // View for gas particles with relevant components
    auto view = registry.view<Components::Position, Components::Shape, sf::CircleShape, Components::Color, Components::ParticlePhase>();

    for (auto entity : view) {
        const auto &phase = view.get<Components::ParticlePhase>(entity);
        // Skip if not a gas particle
        if (phase.phase != Components::Phase::Gas) continue;

        // Ensure it has the required components for rendering
        // (Position, Shape, CircleShape, Color are guaranteed by the view)
        const auto &position = view.get<Components::Position>(entity);
        const auto &shape    = view.get<Components::Shape>(entity);
        const auto &circle   = view.get<sf::CircleShape>(entity);
        const auto &color    = view.get<Components::Color>(entity);

        // Currently only supports Circle shapes for gas
        if (shape.type != Components::ShapeType::Circle) continue;

        // Convert radius and position to pixels
        float pixelRadius = static_cast<float>(coordinates.metersToPixels(circle.getRadius()));
        // Ensure minimum visible size
        pixelRadius = std::max(1.0f, pixelRadius);

        float pixelX = static_cast<float>(coordinates.metersToPixels(position.x));
        float pixelY = static_cast<float>(coordinates.metersToPixels(position.y));

        std::cout << "  Drawing Gas Particle at (" << pixelX << ", " << pixelY << ") Radius: " << pixelRadius << " Color: (" << (int)color.r << "," << (int)color.g << "," << (int)color.b << ")" << std::endl;

        // Create and configure the circle shape
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
         std::cout << "GasRenderer: No gas particles found/drawn." << std::endl;
    }
} 