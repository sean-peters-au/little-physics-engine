#define USE_MATH_DEFINES
#include "nbody/arch/native/renderer_native.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sim.hpp"
#include "nbody/math/polygon.hpp"
#include "nbody/systems/rigid_body_collision/contact_manager.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

static sf::Color densityGrayscale(const Renderer::PixelProperties &props) {
    // Example color mapper if you want a density-based grayscale
    const double maxDensity = 100.0;
    double factor = (props.density / maxDensity);
    if (factor > 1.0) { factor = 1.0;
}
    auto const intensity = static_cast<uint8_t>(std::round(factor * 255.0));
    return {intensity, intensity, intensity};
}

Renderer::Renderer(int screenWidth, int screenHeight)
    : 
     initialized(false)
    , screenWidth(screenWidth)
    , screenHeight(screenHeight)
{
}

Renderer::~Renderer() = default;

bool Renderer::init() {
    // Create a window of the specified size
    window.create(sf::VideoMode(screenWidth, screenHeight), "N-Body Simulator");

    // Load a font for text (FPS, UI, etc.)
    if (!font.loadFromFile("assets/fonts/arial.ttf")) {
        std::cerr << "Failed to load font assets/fonts/arial.ttf\n";
        return false;
    }
    initialized = true;
    return true;
}

void Renderer::clear() {
    window.clear(sf::Color::Black);
}

void Renderer::present() {
    window.display();
}

std::unordered_map<std::pair<int,int>, Renderer::PixelProperties, PixelCoordHash>
Renderer::aggregateParticlesByPixel(const entt::registry &registry)
{
    std::unordered_map<std::pair<int,int>, PixelProperties, PixelCoordHash> pixelMap;

    // We'll just gather basic info from Position, (Density, Temperature, Mass).
    // Each entity is "binned" to the closest pixel coordinate:
    auto view = registry.view<Components::Position>();
    for (auto entity : view) {
        const auto &pos = view.get<Components::Position>(entity);

        int const px = static_cast<int>(SimulatorConstants::metersToPixels(pos.x));
        int const py = static_cast<int>(SimulatorConstants::metersToPixels(pos.y));

        // Only accumulate if within the simulation screen
        if (px >= 0 && px < static_cast<int>(SimulatorConstants::ScreenLength) &&
            py >= 0 && py < static_cast<int>(SimulatorConstants::ScreenLength))
        {
            const auto *dens = registry.try_get<Components::Density>(entity);
            const auto *temp = registry.try_get<Components::Temperature>(entity);
            const auto *mass = registry.try_get<Components::Mass>(entity);
            const auto *sleep = registry.try_get<Components::Sleep>(entity);

            auto &pixProps = pixelMap[{px, py}];
            pixProps.add(dens, temp, mass, sleep);
        }
    }

    return pixelMap;
}

void Renderer::renderParticles(const entt::registry &registry) {
    ColorMapper mapper;
    switch (currentColorScheme) {
        case ColorScheme::SLEEP:
            mapper = sleepColorMapper;
            break;
        case ColorScheme::TEMPERATURE:
            mapper = temperatureColorMapper;
            break;
        case ColorScheme::DEFAULT:
        default:
            mapper = defaultColorMapper;
            break;
    }
    
    // Build the pixel aggregator for fallback coloring if entity lacks a Components::Color
    auto pixelMap = aggregateParticlesByPixel(registry);

    // Now draw each entity
    auto view = registry.view<Components::Position, Components::Shape>();
    for (auto entity : view) {
        const auto &pos = view.get<Components::Position>(entity);
        const auto &shape = view.get<Components::Shape>(entity);

        auto const px = static_cast<float>(SimulatorConstants::metersToPixels(pos.x));
        auto const py = static_cast<float>(SimulatorConstants::metersToPixels(pos.y));

        // Determine fill color
        sf::Color fillColor;
        if (currentColorScheme == ColorScheme::DEFAULT && registry.any_of<Components::Color>(entity)) {
            // In DEFAULT scheme, use entity's color component if it exists
            const auto& colComp = registry.get<Components::Color>(entity);
            fillColor = sf::Color(colComp.r, colComp.g, colComp.b);
        } else {
            // Otherwise use the color mapper
            std::pair<int,int> const coords(static_cast<int>(px), static_cast<int>(py));
            auto it = pixelMap.find(coords);
            if (it != pixelMap.end()) {
                fillColor = mapper(it->second);
            } else {
                fillColor = sf::Color::White; // default
            }
        }

        // Angle from AngularPosition if present
        double angleDeg = 0.0;
        if (registry.any_of<Components::AngularPosition>(entity)) {
            const auto &angPos = registry.get<Components::AngularPosition>(entity);
            angleDeg = (angPos.angle * 180.0 / SimulatorConstants::Pi);
        }

        // If we have a PolygonShape, we handle it with sf::ConvexShape
        if (const auto *poly = registry.try_get<PolygonShape>(entity)) {
            sf::ConvexShape convex;
            convex.setPointCount(poly->vertices.size());

            // Convert angle to radians for rotation
            double const rad = angleDeg * (SimulatorConstants::Pi / 180.0);
            double const ca = std::cos(rad);
            double const sa = std::sin(rad);

            // Each vertex is local to the entity, so rotate it, then offset by (px,py)
            for (size_t i = 0; i < poly->vertices.size(); ++i) {
                const auto &v = poly->vertices[i];
                // rotation
                double const rx = v.x * ca - v.y * sa;
                double const ry = v.x * sa + v.y * ca;

                auto const vxPx = static_cast<float>(SimulatorConstants::metersToPixels(pos.x + rx));
                auto const vyPx = static_cast<float>(SimulatorConstants::metersToPixels(pos.y + ry));
                convex.setPoint(i, sf::Vector2f(vxPx, vyPx));
            }

            convex.setFillColor(fillColor);
            window.draw(convex);
        }
        else {
            // Possibly circle or "square" (legacy usage for shape.type)
            if (shape.type == Components::ShapeType::Circle) {
                float const radiusPixels = static_cast<float>(std::max(1.0, SimulatorConstants::metersToPixels(shape.size)));
                sf::CircleShape circle(radiusPixels);
                circle.setOrigin(radiusPixels, radiusPixels);
                circle.setPosition(px, py);
                circle.setFillColor(fillColor);
                window.draw(circle);

                // Add rotation indicator
                float const indicatorRadius = std::max(1.0F, radiusPixels * 0.2F);  // 20% of main circle
                sf::CircleShape indicator(indicatorRadius);
                // set the colour to a slightly darker shade than the main circle (don't go below zero)
                indicator.setFillColor(sf::Color(std::max(0, fillColor.r - 50), std::max(0, fillColor.g - 50), std::max(0, fillColor.b - 50)));  // Dark indicator
                
                // Position the indicator on the edge of the main circle
                double angle = 0.0;
                if (registry.any_of<Components::AngularPosition>(entity)) {
                    angle = registry.get<Components::AngularPosition>(entity).angle;
                }
                
                // Calculate indicator position: main circle center + rotated radius vector
                float const indicatorX = px + (radiusPixels - indicatorRadius * 2) * std::cos(angle);
                float const indicatorY = py + (radiusPixels - indicatorRadius * 2) * std::sin(angle);
                
                indicator.setOrigin(indicatorRadius, indicatorRadius);
                indicator.setPosition(indicatorX, indicatorY);
                window.draw(indicator);
            } else {
                // Let's assume shape.type == Square or something similar
                auto const halfSide = static_cast<float>(SimulatorConstants::metersToPixels(shape.size));
                float const side = halfSide * 2.0F;
                sf::RectangleShape rect(sf::Vector2f(side, side));
                rect.setOrigin(halfSide, halfSide);
                rect.setPosition(px, py);
                rect.setRotation(static_cast<float>(angleDeg));
                rect.setFillColor(fillColor);
                window.draw(rect);
            }
        }
    }

    // Add debug visualization if enabled
    if (debugVisualization) {
        renderContactDebug(registry);
        renderVelocityDebug(registry);
        renderAngularDebug(registry);
        renderPolygonDebug(registry);
    }

    // Draw a divider line on the right edge of the simulation region
    sf::Vertex line[2] = {
        sf::Vertex(sf::Vector2f(static_cast<float>(SimulatorConstants::ScreenLength), 0.F), sf::Color::White),
        sf::Vertex(sf::Vector2f(static_cast<float>(SimulatorConstants::ScreenLength), static_cast<float>(screenHeight)), sf::Color::White)
    };
    window.draw(line, 2, sf::Lines);
}

void Renderer::renderFPS(float fps) {
    // Display fps with one decimal place
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1) << fps << " FPS";
    renderText(ss.str(), 10, 10, sf::Color::White);
}

void Renderer::renderText(const std::string &text, int x, int y, sf::Color color) {
    sf::Text sfText;
    sfText.setFont(font);
    sfText.setString(text);
    sfText.setCharacterSize(16);
    sfText.setFillColor(color);
    sfText.setPosition(static_cast<float>(x), static_cast<float>(y));
    window.draw(sfText);
}

void Renderer::renderUI(const entt::registry &registry,
                        bool paused,
                        SimulatorConstants::SimulationType currentScenario,
                        const std::vector<std::pair<SimulatorConstants::SimulationType, std::string>> &scenarios,
                        bool showPausePlayHighlight,
                        bool showResetHighlight,
                        SimulatorConstants::SimulationType highlightedScenario)
{
    scenarioButtons.clear();
    speedButtons.clear();

    // We'll place the UI to the right of the simulation area
    int const panelX = static_cast<int>(SimulatorConstants::ScreenLength) + 10;
    int panelY = 10;

    // Pause/Play button
    std::string const pauseLabel = paused ? "Play" : "Pause";
    pausePlayButton.rect = sf::IntRect(panelX, panelY, 80, 30);
    pausePlayButton.label = pauseLabel;
    pausePlayButton.isSpecialButton = true;
    pausePlayButton.scenario = currentScenario;

    {
        sf::RectangleShape shape(sf::Vector2f(static_cast<float>(pausePlayButton.rect.width),
                                              static_cast<float>(pausePlayButton.rect.height)));
        shape.setPosition(static_cast<float>(pausePlayButton.rect.left), static_cast<float>(pausePlayButton.rect.top));
        if (showPausePlayHighlight) {
            shape.setFillColor(sf::Color(200, 200, 0));
        } else {
            shape.setFillColor(sf::Color(100, 100, 100));
        }
        shape.setOutlineColor(sf::Color::White);
        shape.setOutlineThickness(1.F);
        window.draw(shape);

        renderText(pauseLabel, panelX + 5, panelY + 5);
    }
    panelY += 40;

    // Add Next Frame button (only enabled when paused)
    nextFrameButton.rect = sf::IntRect(panelX, panelY, 100, 30);
    nextFrameButton.label = "Next Frame";
    nextFrameButton.isSpecialButton = true;
    nextFrameButton.scenario = currentScenario;

    {
        sf::RectangleShape shape(sf::Vector2f(static_cast<float>(nextFrameButton.rect.width),
                                             static_cast<float>(nextFrameButton.rect.height)));
        shape.setPosition(static_cast<float>(nextFrameButton.rect.left), static_cast<float>(nextFrameButton.rect.top));
        
        // Gray out the button if not paused
        if (!paused) {
            shape.setFillColor(sf::Color(50, 50, 50));  // Darker gray when disabled
        } else {
            shape.setFillColor(sf::Color(100, 100, 100));
        }
        shape.setOutlineColor(sf::Color::White);
        shape.setOutlineThickness(1.F);
        window.draw(shape);

        renderText("Next Frame", panelX + 5, panelY + 5, paused ? sf::Color::White : sf::Color(150, 150, 150));
    }
    panelY += 50;

    // Reset button
    resetButton.rect = sf::IntRect(panelX, panelY, 80, 30);
    resetButton.label = "Reset";
    resetButton.isSpecialButton = true;
    resetButton.scenario = currentScenario;

    {
        sf::RectangleShape shape(sf::Vector2f(static_cast<float>(resetButton.rect.width),
                                              static_cast<float>(resetButton.rect.height)));
        shape.setPosition(static_cast<float>(resetButton.rect.left), static_cast<float>(resetButton.rect.top));
        if (showResetHighlight) {
            shape.setFillColor(sf::Color(200, 200, 0));
        } else {
            shape.setFillColor(sf::Color(100, 100, 100));
        }
        shape.setOutlineColor(sf::Color::White);
        shape.setOutlineThickness(1.F);
        window.draw(shape);

        renderText("Reset", panelX + 5, panelY + 5);
    }
    panelY += 50;

    // Speed section
    renderText("Playback Speed:", panelX, panelY, sf::Color::White);
    panelY += 25;

    // A few example speeds
    const std::vector<std::pair<double, std::string>> speeds = {
        {0.25, "0.25x"},
        {0.5,  "0.5x"},
        {1.0,  "1x"}
    };

    // If there's a simulator state, we can highlight the current timeScale
    const Components::SimulatorState* simState = nullptr;
    auto stView = registry.view<Components::SimulatorState>();
    if (!stView.empty()) {
        simState = &registry.get<Components::SimulatorState>(stView.front());
    }

    for (const auto& sp : speeds) {
        UIButton btn;
        btn.rect = sf::IntRect(panelX, panelY, 60, 25);
        btn.label = sp.second;
        btn.isSpecialButton = true;
        btn.scenario = currentScenario;
        btn.speedMultiplier = sp.first;

        sf::RectangleShape shape(sf::Vector2f(static_cast<float>(btn.rect.width), static_cast<float>(btn.rect.height)));
        shape.setPosition(static_cast<float>(btn.rect.left), static_cast<float>(btn.rect.top));

        // highlight if current timeScale ~ sp.first
        if ((simState != nullptr) && std::fabs(simState->timeScale - sp.first) < 0.01) {
            shape.setFillColor(sf::Color(0, 200, 0));
        } else {
            shape.setFillColor(sf::Color(100, 100, 100));
        }

        shape.setOutlineColor(sf::Color::White);
        shape.setOutlineThickness(1.F);
        window.draw(shape);

        renderText(btn.label, panelX + 5, panelY + 3);

        speedButtons.push_back(btn);
        panelY += 35;
    }

    panelY += 20;
    renderText("Color Scheme:", panelX, panelY, sf::Color::White);
    panelY += 25;

    const std::vector<std::pair<ColorScheme, std::string>> schemes = {
        {ColorScheme::DEFAULT, "Default"},
        {ColorScheme::SLEEP, "Sleep"},
        {ColorScheme::TEMPERATURE, "Temperature"}
    };

    colorSchemeButtons.clear();
    for (const auto& scheme : schemes) {
        UIButton btn;
        btn.rect = sf::IntRect(panelX, panelY, 100, 25);
        btn.label = scheme.second;
        btn.isSpecialButton = true;

        sf::RectangleShape shape(sf::Vector2f(btn.rect.width, btn.rect.height));
        shape.setPosition(btn.rect.left, btn.rect.top);

        if (currentColorScheme == scheme.first) {
            shape.setFillColor(sf::Color(0, 200, 0));
        } else {
            shape.setFillColor(sf::Color(100, 100, 100));
        }

        shape.setOutlineColor(sf::Color::White);
        shape.setOutlineThickness(1.F);
        window.draw(shape);

        renderText(btn.label, panelX + 5, panelY + 3);

        colorSchemeButtons.push_back(btn);
        panelY += 35;
    }

    panelY += 20;
    renderText("Debug View:", panelX, panelY, sf::Color::White);
    panelY += 25;
    
    debugButton.rect = sf::IntRect(panelX, panelY, 100, 25);
    debugButton.label = debugVisualization ? "Debug: ON" : "Debug: OFF";
    debugButton.isSpecialButton = true;
    
    sf::RectangleShape shape(sf::Vector2f(debugButton.rect.width, debugButton.rect.height));
    shape.setPosition(debugButton.rect.left, debugButton.rect.top);
    shape.setFillColor(debugVisualization ? sf::Color(0, 200, 0) : sf::Color(100, 100, 100));
    shape.setOutlineColor(sf::Color::White);
    shape.setOutlineThickness(1.F);
    window.draw(shape);
    
    renderText(debugButton.label, panelX + 5, panelY + 3);
    panelY += 35;

    panelY += 20;
    renderText("Scenarios:", panelX, panelY, sf::Color::White);
    panelY += 30;

    // Build scenario buttons
    for (const auto &sc : scenarios) {
        UIButton btn;
        btn.rect = sf::IntRect(panelX, panelY, 150, 30);
        btn.label = sc.second;
        btn.isSpecialButton = false;
        btn.scenario = sc.first;
        btn.speedMultiplier = 1.0;

        bool const isCurrent = (sc.first == currentScenario);
        bool const isHover   = (sc.first == highlightedScenario);

        sf::RectangleShape shape(sf::Vector2f(static_cast<float>(btn.rect.width), static_cast<float>(btn.rect.height)));
        shape.setPosition(static_cast<float>(btn.rect.left), static_cast<float>(btn.rect.top));

        if (isCurrent) {
            shape.setFillColor(sf::Color(0, 200, 0));
        }
        else if (isHover) {
            shape.setFillColor(sf::Color(200, 200, 0));
        }
        else {
            shape.setFillColor(sf::Color(100, 100, 100));
        }

        shape.setOutlineColor(sf::Color::White);
        shape.setOutlineThickness(1.F);
        window.draw(shape);

        renderText(btn.label, panelX + 5, panelY + 5);

        scenarioButtons.push_back(btn);
        panelY += 40;
    }
}

sf::Color Renderer::defaultColorMapper(const PixelProperties& /* props */) {
    return sf::Color::White;
}

sf::Color Renderer::sleepColorMapper(const PixelProperties& props) {
    if (props.particle_count == 0) { return sf::Color::White;
}
    return props.is_asleep ? sf::Color(200, 50, 50) : sf::Color(50, 200, 50);
}

sf::Color Renderer::temperatureColorMapper(const PixelProperties& props) {
    if (props.particle_count == 0 || !props.has_temperature) { 
        return {128, 128, 128}; // grey for no temperature
}
    
    // Temperature range from cold (blue) to hot (red)
    const double minTemp = 0.0;
    const double maxTemp = 100.0;
    double t = (props.temperature - minTemp) / (maxTemp - minTemp);
    t = std::max(0.0, std::min(1.0, t));
    
    // Blue (0,0,255) -> Red (255,0,0)
    auto const r = static_cast<uint8_t>(255 * t);
    auto const b = static_cast<uint8_t>(255 * (1.0 - t));
    return {r, 0, b};
}

void Renderer::renderContactDebug(const entt::registry &registry) {
    auto view = registry.view<RigidBodyCollision::ContactRef>();
    
    const float lineThickness = 3.0F;  
    const float contactPointSize = 4.0F;
    const float normalLength = 30.0F;  
    
    for (auto entity : view) {
        const auto& contact = view.get<RigidBodyCollision::ContactRef>(entity);
        
        auto const px = static_cast<float>(SimulatorConstants::metersToPixels(contact.contactPoint.x));
        auto const py = static_cast<float>(SimulatorConstants::metersToPixels(contact.contactPoint.y));
        
        // Draw contact point
        sf::CircleShape contactPoint(contactPointSize);
        contactPoint.setFillColor(sf::Color::Yellow);
        contactPoint.setPosition(px - contactPointSize, py - contactPointSize);
        window.draw(contactPoint);

        // Get colors of both shapes
        sf::Color colorA = sf::Color::White;
        sf::Color colorB = sf::Color::White;
        if (registry.valid(contact.a) && registry.any_of<Components::Color>(contact.a)) {
            const auto& color = registry.get<Components::Color>(contact.a);
            colorA = sf::Color(color.r, color.g, color.b);
        }
        if (registry.valid(contact.b) && registry.any_of<Components::Color>(contact.b)) {
            const auto& color = registry.get<Components::Color>(contact.b);
            colorB = sf::Color(color.r, color.g, color.b);
        }

        // Darken the colors for the normal lines (multiply by 0.7)
        sf::Color const darkColorA(
            static_cast<uint8_t>(colorA.r * 0.9F),
            static_cast<uint8_t>(colorA.g * 0.9F),
            static_cast<uint8_t>(colorA.b * 0.9F)
        );
        sf::Color const darkColorB(
            static_cast<uint8_t>(colorB.r * 0.9F),
            static_cast<uint8_t>(colorB.g * 0.9F),
            static_cast<uint8_t>(colorB.b * 0.9F)
        );

        // Draw normal (using shape A's color)
        sf::RectangleShape normal(sf::Vector2f(normalLength, lineThickness));
        normal.setFillColor(darkColorA);
        normal.setPosition(px, py);
        float const angle = std::atan2(contact.normal.y, contact.normal.x) * 180.0F / static_cast<float>(M_PI);
        normal.setRotation(angle);
        window.draw(normal);

        // Draw accumulated normal impulse (using shape B's color)
        if (std::abs(contact.normalImpulseAccum) > 0.001F) {
            float const impulseLength = std::abs(contact.normalImpulseAccum) * 5.0F;
            sf::RectangleShape normalImpulse(sf::Vector2f(impulseLength, lineThickness));
            normalImpulse.setFillColor(darkColorB);
            normalImpulse.setPosition(px, py);
            normalImpulse.setRotation(angle);
            window.draw(normalImpulse);
        }

        // Draw accumulated tangent impulse (blue line - keeping this distinct)
        if (std::abs(contact.tangentImpulseAccum) > 0.001F) {
            float const impulseLength = std::abs(contact.tangentImpulseAccum) * 5.0F;
            sf::RectangleShape tangentImpulse(sf::Vector2f(impulseLength, lineThickness));
            tangentImpulse.setFillColor(sf::Color::Blue);
            tangentImpulse.setPosition(px, py);
            tangentImpulse.setRotation(angle + 90.0F);
            window.draw(tangentImpulse);
        }
    }
}

void Renderer::renderVelocityDebug(const entt::registry &registry) {
    const float lineThickness = 2.0F;  // Increased line thickness
    
    auto view = registry.view<Components::Position, Components::Velocity>();
    for (auto entity : view) {
        const auto& pos = view.get<Components::Position>(entity);
        const auto& vel = view.get<Components::Velocity>(entity);

        auto const px = static_cast<float>(SimulatorConstants::metersToPixels(pos.x));
        auto const py = static_cast<float>(SimulatorConstants::metersToPixels(pos.y));
        
        // Scale velocity for visualization
        float const velLength = std::sqrt(vel.x * vel.x + vel.y * vel.y) * 20.0F;  // Scale factor of 20
        if (velLength > 0.1F) {  // Only draw if velocity is significant
            sf::RectangleShape velLine(sf::Vector2f(velLength, lineThickness));
            velLine.setFillColor(sf::Color::Cyan);
            velLine.setPosition(px, py);
            float const angle = std::atan2(vel.y, vel.x) * 180.0F / static_cast<float>(M_PI);
            velLine.setRotation(angle);
            window.draw(velLine);
        }
    }
}

void Renderer::renderAngularDebug(const entt::registry &registry) {
    const float lineThickness = 2.0F;  // Increased line thickness
    
    auto view = registry.view<Components::Position, Components::AngularVelocity>();
    for (auto entity : view) {
        const auto& pos = view.get<Components::Position>(entity);
        const auto& angVel = view.get<Components::AngularVelocity>(entity);

        if (std::abs(angVel.omega) > 0.1F) {  // Only draw if angular velocity is significant
            auto const px = static_cast<float>(SimulatorConstants::metersToPixels(pos.x));
            auto const py = static_cast<float>(SimulatorConstants::metersToPixels(pos.y));
            
            float const radius = 20.0F;  // Fixed radius for the arc
            int const segments = 16;  // Number of segments in the arc
            float const arcLength = std::min(static_cast<float>(std::abs(angVel.omega) * 0.5), static_cast<float>(M_PI));
            
            std::vector<sf::Vertex> arc;
            for (int i = 0; i <= segments; ++i) {
                float angle = static_cast<float>(i) / segments * arcLength;
                if (angVel.omega < 0) { angle = -angle;
}
                
                float const x = px + radius * std::cos(angle);
                float const y = py + radius * std::sin(angle);
                arc.emplace_back(sf::Vector2f(x, y), sf::Color::White);
            }
            
            // Draw thicker lines by drawing multiple offset lines
            for (float offset = -lineThickness/2; offset <= lineThickness/2; offset += 1.0F) {
                std::vector<sf::Vertex> thickArc = arc;
                for (auto& vertex : thickArc) {
                    vertex.position.y += offset;
                }
                window.draw(thickArc.data(), thickArc.size(), sf::LineStrip);
            }
        }
    }
}

void Renderer::renderPolygonDebug(const entt::registry &registry) {
    auto view = registry.view<Components::Position, PolygonShape, Components::AngularPosition>();
    for (auto entity : view) {
        const auto &pos = view.get<Components::Position>(entity);
        const auto &poly = view.get<PolygonShape>(entity);
        const auto &angPos = view.get<Components::AngularPosition>(entity);
        
        // Get rotation
        double const rad = angPos.angle;
        double const ca = std::cos(rad);
        double const sa = std::sin(rad);
        
        // Draw index number at each vertex
        for (size_t i = 0; i < poly.vertices.size(); i++) {
            const auto &v = poly.vertices[i];
            // Apply rotation and translation
            double const rx = v.x * ca - v.y * sa;
            double const ry = v.x * sa + v.y * ca;
            
            auto const vxPx = static_cast<float>(SimulatorConstants::metersToPixels(pos.x + rx));
            auto const vyPx = static_cast<float>(SimulatorConstants::metersToPixels(pos.y + ry));
            
            // Draw vertex index
            std::string const idx = std::to_string(i);
            renderText(idx, static_cast<int>(vxPx), static_cast<int>(vyPx), sf::Color::Yellow);
            
            // Optionally: Draw a small dot at vertex
            sf::CircleShape dot(2.0F);
            dot.setFillColor(sf::Color::Yellow);
            dot.setPosition(vxPx - 2.0F, vyPx - 2.0F);
            window.draw(dot);
        }
    }
}

void Renderer::handleUIClick(int x, int y) {
    // ... existing UI handling code ...

    // Check if debug button was clicked
    if (debugButton.rect.contains(x, y)) {
        debugVisualization = !debugVisualization;
        return;
    }
}