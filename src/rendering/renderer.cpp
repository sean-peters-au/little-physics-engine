#include "nbody/rendering/renderer.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sim.hpp"
#include "nbody/math/polygon.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

static sf::Color densityGrayscale(const Renderer::PixelProperties &props) {
    // Example color mapper if you want a density-based grayscale
    const double max_density = 100.0;
    double factor = (props.density / max_density);
    if (factor > 1.0) factor = 1.0;
    uint8_t intensity = static_cast<uint8_t>(std::round(factor * 255.0));
    return sf::Color(intensity, intensity, intensity);
}

Renderer::Renderer(int screenWidth, int screenHeight)
    : window()
    , font()
    , initialized(false)
    , screenWidth(screenWidth)
    , screenHeight(screenHeight)
{
}

Renderer::~Renderer() {}

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

        int px = static_cast<int>(SimulatorConstants::metersToPixels(pos.x));
        int py = static_cast<int>(SimulatorConstants::metersToPixels(pos.y));

        // Only accumulate if within the simulation screen
        if (px >= 0 && px < (int)SimulatorConstants::ScreenLength &&
            py >= 0 && py < (int)SimulatorConstants::ScreenLength)
        {
            auto *dens = registry.try_get<Components::Density>(entity);
            auto *temp = registry.try_get<Components::Temperature>(entity);
            auto *mass = registry.try_get<Components::Mass>(entity);

            auto &pixProps = pixelMap[{px, py}];
            pixProps.add(dens, temp, mass);
        }
    }

    return pixelMap;
}

void Renderer::renderParticles(const entt::registry &registry, ColorMapper colorMapper) {
    // Build the pixel aggregator for fallback coloring if entity lacks a Components::Color
    auto pixelMap = aggregateParticlesByPixel(registry);

    // Now draw each entity
    auto view = registry.view<Components::Position, Components::Shape>();
    for (auto entity : view) {
        // Optionally skip boundary entities or do something special
        if (registry.any_of<Components::Boundary>(entity)) {
            // continue;
        }

        const auto &pos = view.get<Components::Position>(entity);
        const auto &shape = view.get<Components::Shape>(entity);

        float px = (float)SimulatorConstants::metersToPixels(pos.x);
        float py = (float)SimulatorConstants::metersToPixels(pos.y);

        // Determine fill color
        sf::Color fillColor;
        if (auto *colComp = registry.try_get<Components::Color>(entity)) {
            fillColor = sf::Color(colComp->r, colComp->g, colComp->b);
        } else {
            // fallback color from the aggregated pixel data
            std::pair<int,int> coords((int)px, (int)py);
            auto it = pixelMap.find(coords);
            if (it != pixelMap.end()) {
                fillColor = colorMapper(it->second);
            } else {
                fillColor = sf::Color::White; // default
            }
        }

        // Angle from AngularPosition if present
        double angleDeg = 0.0;
        if (registry.any_of<Components::AngularPosition>(entity)) {
            auto &angPos = registry.get<Components::AngularPosition>(entity);
            angleDeg = (angPos.angle * 180.0 / SimulatorConstants::Pi);
        }

        // If we have a PolygonShape, we handle it with sf::ConvexShape
        if (auto *poly = registry.try_get<PolygonShape>(entity)) {
            sf::ConvexShape convex;
            convex.setPointCount(poly->vertices.size());

            // Convert angle to radians for rotation
            double rad = angleDeg * (SimulatorConstants::Pi / 180.0);
            double ca = std::cos(rad);
            double sa = std::sin(rad);

            // Each vertex is local to the entity, so rotate it, then offset by (px,py)
            for (size_t i = 0; i < poly->vertices.size(); ++i) {
                const auto &v = poly->vertices[i];
                // rotation
                double rx = v.x * ca - v.y * sa;
                double ry = v.x * sa + v.y * ca;

                float vx_px = (float)SimulatorConstants::metersToPixels(pos.x + rx);
                float vy_px = (float)SimulatorConstants::metersToPixels(pos.y + ry);
                convex.setPoint(i, sf::Vector2f(vx_px, vy_px));
            }

            convex.setFillColor(fillColor);
            window.draw(convex);
        }
        else {
            // Possibly circle or "square" (legacy usage for shape.type)
            if (shape.type == Components::ShapeType::Circle) {
                float radiusPixels = (float)std::max(1.0, SimulatorConstants::metersToPixels(shape.size));
                sf::CircleShape circle(radiusPixels);
                circle.setOrigin(radiusPixels, radiusPixels);
                circle.setPosition(px, py);
                circle.setFillColor(fillColor);
                window.draw(circle);
            } else {
                // Let's assume shape.type == Square or something similar
                float halfSide = (float)SimulatorConstants::metersToPixels(shape.size);
                float side = halfSide * 2.0f;
                sf::RectangleShape rect(sf::Vector2f(side, side));
                rect.setOrigin(halfSide, halfSide);
                rect.setPosition(px, py);
                rect.setRotation((float)angleDeg);
                rect.setFillColor(fillColor);
                window.draw(rect);
            }
        }
    }

    // Draw a divider line on the right edge of the simulation region
    sf::Vertex line[2] = {
        sf::Vertex(sf::Vector2f((float)SimulatorConstants::ScreenLength, 0.f), sf::Color::White),
        sf::Vertex(sf::Vector2f((float)SimulatorConstants::ScreenLength, (float)screenHeight), sf::Color::White)
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
    sfText.setPosition((float)x, (float)y);
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
    int panelX = (int)SimulatorConstants::ScreenLength + 10;
    int panelY = 10;

    // Pause/Play button
    std::string pauseLabel = paused ? "Play" : "Pause";
    pausePlayButton.rect = sf::IntRect(panelX, panelY, 80, 30);
    pausePlayButton.label = pauseLabel;
    pausePlayButton.isSpecialButton = true;
    pausePlayButton.scenario = currentScenario;

    {
        sf::RectangleShape shape(sf::Vector2f((float)pausePlayButton.rect.width,
                                              (float)pausePlayButton.rect.height));
        shape.setPosition((float)pausePlayButton.rect.left, (float)pausePlayButton.rect.top);
        if (showPausePlayHighlight) {
            shape.setFillColor(sf::Color(200, 200, 0));
        } else {
            shape.setFillColor(sf::Color(100, 100, 100));
        }
        shape.setOutlineColor(sf::Color::White);
        shape.setOutlineThickness(1.f);
        window.draw(shape);

        renderText(pauseLabel, panelX + 5, panelY + 5);
    }
    panelY += 40;

    // Reset button
    resetButton.rect = sf::IntRect(panelX, panelY, 80, 30);
    resetButton.label = "Reset";
    resetButton.isSpecialButton = true;
    resetButton.scenario = currentScenario;

    {
        sf::RectangleShape shape(sf::Vector2f((float)resetButton.rect.width,
                                              (float)resetButton.rect.height));
        shape.setPosition((float)resetButton.rect.left, (float)resetButton.rect.top);
        if (showResetHighlight) {
            shape.setFillColor(sf::Color(200, 200, 0));
        } else {
            shape.setFillColor(sf::Color(100, 100, 100));
        }
        shape.setOutlineColor(sf::Color::White);
        shape.setOutlineThickness(1.f);
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

    for (auto& sp : speeds) {
        UIButton btn;
        btn.rect = sf::IntRect(panelX, panelY, 60, 25);
        btn.label = sp.second;
        btn.isSpecialButton = true;
        btn.scenario = currentScenario;
        btn.speedMultiplier = sp.first;

        sf::RectangleShape shape(sf::Vector2f((float)btn.rect.width, (float)btn.rect.height));
        shape.setPosition((float)btn.rect.left, (float)btn.rect.top);

        // highlight if current timeScale ~ sp.first
        if (simState && std::fabs(simState->timeScale - sp.first) < 0.01) {
            shape.setFillColor(sf::Color(0, 200, 0));
        } else {
            shape.setFillColor(sf::Color(100, 100, 100));
        }

        shape.setOutlineColor(sf::Color::White);
        shape.setOutlineThickness(1.f);
        window.draw(shape);

        renderText(btn.label, panelX + 5, panelY + 3);

        speedButtons.push_back(btn);
        panelY += 35;
    }

    panelY += 20;
    renderText("Scenarios:", panelX, panelY, sf::Color::White);
    panelY += 30;

    // Build scenario buttons
    for (auto &sc : scenarios) {
        UIButton btn;
        btn.rect = sf::IntRect(panelX, panelY, 150, 30);
        btn.label = sc.second;
        btn.isSpecialButton = false;
        btn.scenario = sc.first;
        btn.speedMultiplier = 1.0;

        bool isCurrent = (sc.first == currentScenario);
        bool isHover   = (sc.first == highlightedScenario);

        sf::RectangleShape shape(sf::Vector2f((float)btn.rect.width, (float)btn.rect.height));
        shape.setPosition((float)btn.rect.left, (float)btn.rect.top);

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
        shape.setOutlineThickness(1.f);
        window.draw(shape);

        renderText(btn.label, panelX + 5, panelY + 5);

        scenarioButtons.push_back(btn);
        panelY += 40;
    }
}