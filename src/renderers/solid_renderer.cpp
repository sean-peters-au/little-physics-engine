/**
 * @file solid_renderer.cpp
 * @brief Implementation of SolidRenderer.
 */

#include "renderers/solid_renderer.hpp"
#include "renderer_types.hpp" // Include for shared types
#include "presentation_manager.hpp" // Include for static color mappers

#include <cmath>
#include <algorithm> // For std::max
#include <vector>

// Headers required for component access and drawing
#include "core/constants.hpp"
#include "math/constants.hpp"
#include "math/polygon.hpp"
#include "systems/rigid/contact_manager.hpp" // For ContactRef

SolidRenderer::SolidRenderer(Simulation::Coordinates& coords)
    : coordinates(coords) {}

void SolidRenderer::render(sf::RenderTarget& target,
                           const entt::registry& registry,
                           ColorScheme colorScheme)
{
    ColorMapper mapper;
    switch (colorScheme) {
        case ColorScheme::SLEEP:        mapper = PresentationManager::sleepColorMapper; break;
        case ColorScheme::TEMPERATURE:  mapper = PresentationManager::temperatureColorMapper; break;
        case ColorScheme::DEFAULT:
        default:                      mapper = PresentationManager::defaultColorMapper; break;
    }

    std::unordered_map<std::pair<int,int>, PixelProperties, PixelCoordHash> pixelMap;
    if (colorScheme != ColorScheme::DEFAULT) {
        pixelMap = aggregateSolidParticlesByPixel(registry);
    }

    auto view = registry.view<Components::Position, Components::Shape>();
    for (auto entity : view) {
        if (registry.any_of<Components::ParticlePhase>(entity)) {
            const auto &phase = registry.get<Components::ParticlePhase>(entity);
            if (phase.phase != Components::Phase::Solid) continue;
        }

        const auto &pos = view.get<Components::Position>(entity);
        const auto &shape = view.get<Components::Shape>(entity);
        auto const px = static_cast<float>(coordinates.metersToPixels(pos.x));
        auto const py = static_cast<float>(coordinates.metersToPixels(pos.y));

        sf::Color fillColor;
        if (colorScheme == ColorScheme::DEFAULT && registry.any_of<Components::Color>(entity)) {
            const auto& colComp = registry.get<Components::Color>(entity);
            fillColor = sf::Color(colComp.r, colComp.g, colComp.b);
        } else {
            std::pair<int,int> const coords(static_cast<int>(px), static_cast<int>(py));
            auto it = pixelMap.find(coords);
            fillColor = (it != pixelMap.end()) ? mapper(it->second) : sf::Color::White;
        }

        double angleDeg = 0.0;
        if (registry.any_of<Components::AngularPosition>(entity)) {
            const auto &angPos = registry.get<Components::AngularPosition>(entity);
            angleDeg = (angPos.angle * 180.0 / MathConstants::PI);
        }

        if (const auto *poly = registry.try_get<PolygonShape>(entity)) {
            sf::ConvexShape convex;
            convex.setPointCount(poly->vertices.size());
            double const rad = angleDeg * (MathConstants::PI / 180.0);
            double const ca = std::cos(rad);
            double const sa = std::sin(rad);
            
            // Calculate center point in pixels
            float const centerPx = static_cast<float>(coordinates.metersToPixels(pos.x));
            float const centerPy = static_cast<float>(coordinates.metersToPixels(pos.y));
            // sf::Vector2f firstVertexPx; // REMOVE - No longer needed for line

            // Temporary storage for vertex positions before setting color
            std::vector<sf::Vector2f> vertexPositions(poly->vertices.size());
            float minY = centerPy, maxY = centerPy; // Find min/max Y for gradient

            for (size_t i = 0; i < poly->vertices.size(); ++i) {
                const auto &v = poly->vertices[i];
                double const rx = v.x * ca - v.y * sa;
                double const ry = v.x * sa + v.y * ca;
                auto const vxPx = static_cast<float>(coordinates.metersToPixels(pos.x + rx));
                auto const vyPx = static_cast<float>(coordinates.metersToPixels(pos.y + ry));
                vertexPositions[i] = sf::Vector2f(vxPx, vyPx);
                // Find bounds for gradient
                if (i == 0) { minY = maxY = vyPx; }
                else { minY = std::min(minY, vyPx); maxY = std::max(maxY, vyPx); }
            }

            // Set outline properties
            sf::Color outlineColor = sf::Color(std::max(0, fillColor.r - 50), 
                                                 std::max(0, fillColor.g - 50), 
                                                 std::max(0, fillColor.b - 50));
            convex.setOutlineThickness(0.0f); 
            convex.setOutlineColor(outlineColor);
            // convex.setFillColor(fillColor); // REMOVE - Will set vertex colors instead

            // Apply vertex colors for gradient
            float height = std::max(1.0f, maxY - minY); // Avoid division by zero
            for (size_t i = 0; i < poly->vertices.size(); ++i) {
                float t = (vertexPositions[i].y - minY) / height; // Normalize y position (0=top, 1=bottom)
                // Simple vertical gradient: lighter at top (t=0), darker at bottom (t=1)
                float factor = 1.1f - 0.3f * t; // e.g., from 1.1 down to 0.8
                sf::Color vertexColor(
                    static_cast<sf::Uint8>(std::min(255.f, std::max(0.f, fillColor.r * factor))),
                    static_cast<sf::Uint8>(std::min(255.f, std::max(0.f, fillColor.g * factor))),
                    static_cast<sf::Uint8>(std::min(255.f, std::max(0.f, fillColor.b * factor)))
                );
                convex.setPoint(i, vertexPositions[i]); // Set position and color
                convex.setFillColor(vertexColor); // NOTE: For sf::ConvexShape, you set FillColor *before* setting the point for vertex colors!
                                                  // Correction: This is wrong for sf::Convex. Need to set vertex colors differently. 
                                                  // Reverting to a simpler approach for now: Keep fill color, add outline.
            }
            // --- REVERTING VERTEX COLOR ATTEMPT --- Keep it simple for now.
            convex.setFillColor(fillColor); // Set the base fill color back.
            for (size_t i = 0; i < poly->vertices.size(); ++i) {
                 convex.setPoint(i, vertexPositions[i]); // Just set positions
            }
            // --- END REVERT --- 
            
            target.draw(convex);

            // REMOVE indicator line drawing
            // if (poly->vertices.size() > 0) { ... }
        }
        else {
             if (shape.type == Components::ShapeType::Circle) {
                float const radiusPixels = static_cast<float>(std::max(1.0, coordinates.metersToPixels(shape.size)));
                sf::CircleShape circle(radiusPixels);
                circle.setOrigin(radiusPixels, radiusPixels);
                circle.setPosition(px, py);
                circle.setFillColor(fillColor);
                
                // Add outline to circle
                circle.setOutlineThickness(1.0f); // Adjust thickness as needed
                sf::Color outlineColor = sf::Color(std::max(0, fillColor.r - 50), 
                                                     std::max(0, fillColor.g - 50), 
                                                     std::max(0, fillColor.b - 50));
                circle.setOutlineColor(outlineColor);
                
                target.draw(circle);

                // REMOVE indicator dot drawing
                // float const indicatorRadius = std::max(1.0F, radiusPixels * 0.2F);
                // sf::CircleShape indicator(indicatorRadius);
                // indicator.setFillColor(sf::Color(std::max(0, fillColor.r - 50), std::max(0, fillColor.g - 50), std::max(0, fillColor.b - 50)));
                // double angleRad = 0.0;
                //  if (registry.any_of<Components::AngularPosition>(entity)) {
                //      angleRad = registry.get<Components::AngularPosition>(entity).angle;
                //  }
                // float const indicatorX = px + (radiusPixels - indicatorRadius * 1.5f) * std::cos(angleRad);
                // float const indicatorY = py + (radiusPixels - indicatorRadius * 1.5f) * std::sin(angleRad);
                // indicator.setOrigin(indicatorRadius, indicatorRadius);
                // indicator.setPosition(indicatorX, indicatorY);
                // target.draw(indicator);
            } else {
                auto const halfSide = static_cast<float>(coordinates.metersToPixels(shape.size));
                float const side = halfSide * 2.0F;
                sf::RectangleShape rect(sf::Vector2f(side, side));
                rect.setOrigin(halfSide, halfSide);
                rect.setPosition(px, py);
                rect.setRotation(static_cast<float>(angleDeg));
                rect.setFillColor(fillColor);
                target.draw(rect);
            }
        }
    }

    if (debugEnabled) {
        renderContactDebug(target, registry);
        renderVelocityDebug(target, registry);
        renderAngularDebug(target, registry);
    }
}

std::unordered_map<std::pair<int,int>, PixelProperties, PixelCoordHash>
SolidRenderer::aggregateSolidParticlesByPixel(const entt::registry& registry)
{
    std::unordered_map<std::pair<int,int>, PixelProperties, PixelCoordHash> pixelMap;
    auto view = registry.view<Components::Position, Components::ParticlePhase>();
    for (auto entity : view) {
        const auto &phase = view.get<Components::ParticlePhase>(entity);
        if (phase.phase != Components::Phase::Solid) continue;

        const auto &pos = registry.get<Components::Position>(entity);
        int const px = static_cast<int>(coordinates.metersToPixels(pos.x));
        int const py = static_cast<int>(coordinates.metersToPixels(pos.y));

        if (px >= 0 && px < static_cast<int>(SimulatorConstants::ScreenLength) &&
            py >= 0 && py < static_cast<int>(SimulatorConstants::ScreenLength))
        {
            const auto *dens = registry.try_get<Components::Density>(entity);
            const auto *temp = registry.try_get<Components::Temperature>(entity);
            const auto *mass = registry.try_get<Components::Mass>(entity);
            const auto *sleep = registry.try_get<Components::Sleep>(entity);
            pixelMap[{px, py}].add(dens, temp, mass, sleep);
        }
    }
    return pixelMap;
}

void SolidRenderer::renderContactDebug(sf::RenderTarget& target, const entt::registry &registry) {
    auto view = registry.view<RigidBodyCollision::ContactRef>();
    const float lineThickness = 3.0F;
    const float contactPointSize = 4.0F;
    const float normalLength = 30.0F;

    for (auto entity : view) {
        const auto& contact = view.get<RigidBodyCollision::ContactRef>(entity);
        auto const px = static_cast<float>(coordinates.metersToPixels(contact.contactPoint.x));
        auto const py = static_cast<float>(coordinates.metersToPixels(contact.contactPoint.y));

        sf::CircleShape contactPoint(contactPointSize);
        contactPoint.setFillColor(sf::Color::Yellow);
        contactPoint.setPosition(px - contactPointSize, py - contactPointSize);
        target.draw(contactPoint);

        sf::Color colorA = sf::Color::White, colorB = sf::Color::White;
        if (registry.valid(contact.a) && registry.any_of<Components::Color>(contact.a)) {
            const auto& color = registry.get<Components::Color>(contact.a); colorA = sf::Color(color.r, color.g, color.b);
        }
        if (registry.valid(contact.b) && registry.any_of<Components::Color>(contact.b)) {
            const auto& color = registry.get<Components::Color>(contact.b); colorB = sf::Color(color.r, color.g, color.b);
        }
        sf::Color const darkColorA(static_cast<uint8_t>(colorA.r*0.9f), static_cast<uint8_t>(colorA.g*0.9f), static_cast<uint8_t>(colorA.b*0.9f));
        sf::Color const darkColorB(static_cast<uint8_t>(colorB.r*0.9f), static_cast<uint8_t>(colorB.g*0.9f), static_cast<uint8_t>(colorB.b*0.9f));

        float const angle = std::atan2(contact.normal.y, contact.normal.x) * 180.0F / static_cast<float>(MathConstants::PI);

        sf::RectangleShape normal(sf::Vector2f(normalLength, lineThickness));
        normal.setFillColor(darkColorA);
        normal.setPosition(px, py);
        normal.setRotation(angle);
        target.draw(normal);

        if (std::abs(contact.normalImpulseAccum) > 0.001f) {
             float const impulseLength = std::min(static_cast<float>(std::abs(contact.normalImpulseAccum)) * 5.0f, 50.f);
             sf::RectangleShape normalImpulse(sf::Vector2f(impulseLength, lineThickness));
             normalImpulse.setFillColor(darkColorB);
             normalImpulse.setPosition(px, py);
             normalImpulse.setRotation(angle);
             target.draw(normalImpulse);
        }
        if (std::abs(contact.tangentImpulseAccum) > 0.001f) {
             float const impulseLength = std::min(static_cast<float>(std::abs(contact.tangentImpulseAccum)) * 5.0f, 50.f);
             sf::RectangleShape tangentImpulse(sf::Vector2f(impulseLength, lineThickness));
             tangentImpulse.setFillColor(sf::Color::Blue);
             tangentImpulse.setPosition(px, py);
             float tangentAngle = angle + 90.0f;
             if(contact.tangentImpulseAccum < 0) tangentAngle += 180.0f;
             tangentImpulse.setRotation(tangentAngle);
             target.draw(tangentImpulse);
        }
    }
}

void SolidRenderer::renderVelocityDebug(sf::RenderTarget& target, const entt::registry &registry) {
    const float lineThickness = 2.0F;
    auto view = registry.view<Components::Position, Components::Velocity, Components::ParticlePhase>();
    for (auto entity : view) {
        const auto& phase = view.get<Components::ParticlePhase>(entity);
        if (phase.phase != Components::Phase::Solid) continue;

        const auto& pos = registry.get<Components::Position>(entity);
        const auto& vel = view.get<Components::Velocity>(entity);
        auto const px = static_cast<float>(coordinates.metersToPixels(pos.x));
        auto const py = static_cast<float>(coordinates.metersToPixels(pos.y));
        float const velMag = std::sqrt(vel.x * vel.x + vel.y * vel.y);
        float const velLength = velMag * 20.0F;
        if (velLength > 1.0F) {
            sf::RectangleShape velLine(sf::Vector2f(velLength, lineThickness));
            velLine.setFillColor(sf::Color::Cyan);
            velLine.setPosition(px, py);
            float const angle = std::atan2(vel.y, vel.x) * 180.0F / static_cast<float>(MathConstants::PI);
            velLine.setRotation(angle);
            target.draw(velLine);
        }
    }
}

void SolidRenderer::renderAngularDebug(sf::RenderTarget& target, const entt::registry &registry) {
    const float lineThickness = 2.0F;
    auto view = registry.view<Components::Position, Components::AngularVelocity, Components::ParticlePhase>();
    for (auto entity : view) {
        const auto& phase = view.get<Components::ParticlePhase>(entity);
        if (phase.phase != Components::Phase::Solid) continue;

        const auto& pos = registry.get<Components::Position>(entity);
        const auto& angVel = view.get<Components::AngularVelocity>(entity);

        if (std::abs(angVel.omega) > 0.05F) {
            auto const px = static_cast<float>(coordinates.metersToPixels(pos.x));
            auto const py = static_cast<float>(coordinates.metersToPixels(pos.y));

            float const radius = 15.0F;
            int const segments = 12;
            float const maxArc = static_cast<float>(MathConstants::PI) / 2.f;
            float const arcLength = std::min(static_cast<float>(std::abs(angVel.omega)) * 0.5f, maxArc);

            sf::VertexArray arc(sf::LineStrip, segments + 1);
            for (int i = 0; i <= segments; ++i) {
                float angle = static_cast<float>(i) / segments * arcLength;
                float currentAngle = -static_cast<float>(MathConstants::PI) / 2.f;
                 if (angVel.omega < 0) { angle = -angle; }
                 currentAngle += angle;
                float const x = px + radius * std::cos(currentAngle);
                float const y = py + radius * std::sin(currentAngle);
                arc[i] = sf::Vertex(sf::Vector2f(x, y), sf::Color::Magenta);
            }
            target.draw(arc);

            float arrowAngle = (angVel.omega > 0) ? arc[segments].position.y - arc[segments-1].position.y : arc[segments-1].position.y - arc[segments].position.y;
            arrowAngle = std::atan2(arrowAngle, (angVel.omega > 0) ? arc[segments].position.x - arc[segments-1].position.x : arc[segments-1].position.x - arc[segments].position.x);
            arrowAngle = arrowAngle * 180.f / static_cast<float>(MathConstants::PI);
            sf::ConvexShape arrowHead;
            arrowHead.setPointCount(3);
            arrowHead.setPoint(0, sf::Vector2f(0,0));
            arrowHead.setPoint(1, sf::Vector2f(-8,-4));
            arrowHead.setPoint(2, sf::Vector2f(-8, 4));
            arrowHead.setFillColor(sf::Color::Magenta);
            arrowHead.setPosition(arc[segments].position);
            arrowHead.setRotation(arrowAngle);
            target.draw(arrowHead);
        }
    }
}

void SolidRenderer::renderPolygonDebug(sf::RenderTarget& target, const entt::registry &registry/*, const sf::Font& font*/) {
    auto view = registry.view<Components::Position, PolygonShape, Components::AngularPosition>();
    for (auto entity : view) {
        const auto &pos = view.get<Components::Position>(entity);
        const auto &poly = view.get<PolygonShape>(entity);
        const auto &angPos = view.get<Components::AngularPosition>(entity);

        double const rad = angPos.angle;
        double const ca = std::cos(rad);
        double const sa = std::sin(rad);

        for (size_t i = 0; i < poly.vertices.size(); i++) {
            const auto &v = poly.vertices[i];
            double const rx = v.x * ca - v.y * sa;
            double const ry = v.x * sa + v.y * ca;
            auto const vxPx = static_cast<float>(coordinates.metersToPixels(pos.x + rx));
            auto const vyPx = static_cast<float>(coordinates.metersToPixels(pos.y + ry));

            sf::CircleShape dot(2.0F);
            dot.setFillColor(sf::Color::Yellow);
            dot.setOrigin(2.0f, 2.0f);
            dot.setPosition(vxPx, vyPx);
            target.draw(dot);
        }
    }
} 