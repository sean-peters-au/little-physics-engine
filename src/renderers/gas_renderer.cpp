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
    int gasDrawnCount = 0;

    auto view = registry.view<Components::Position, Components::Shape, Components::Color, Components::ParticlePhase>();

    for (auto entity : view) {
        const auto &phase = view.get<Components::ParticlePhase>(entity);
        if (phase.phase != Components::Phase::Gas) continue;

        const auto &shape = view.get<Components::Shape>(entity);
        if (shape.type != Components::ShapeType::Circle) continue;

        const auto &position = view.get<Components::Position>(entity);
        const auto &color    = view.get<Components::Color>(entity);

        float pixelRadius = static_cast<float>(coordinates.metersToPixels(shape.size));
        pixelRadius = std::max(1.0f, pixelRadius);

        float pixelX = static_cast<float>(coordinates.metersToPixels(position.x));
        float pixelY = static_cast<float>(coordinates.metersToPixels(position.y));

        sf::CircleShape gasCircle(pixelRadius);
        gasCircle.setFillColor(sf::Color(color.r, color.g, color.b, 180));
        gasCircle.setOrigin(pixelRadius, pixelRadius);
        gasCircle.setPosition(pixelX, pixelY);

        target.draw(gasCircle);
        gasDrawnCount++;
    }
} 