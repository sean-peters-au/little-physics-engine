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
    const double max_density = 100.0;
    uint8_t intensity = static_cast<uint8_t>(
        std::min(255.0, (props.density / max_density) * 255.0));
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
    window.create(sf::VideoMode(screenWidth, screenHeight), "N-Body Simulator");

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

std::unordered_map<std::pair<int, int>, Renderer::PixelProperties, PixelCoordHash>
Renderer::aggregateParticlesByPixel(const entt::registry &registry)
{
    std::unordered_map<std::pair<int, int>, PixelProperties, PixelCoordHash> pixelMap;

    auto view = registry.view<Components::Position>();
    for (auto entity : view) {
        const auto &pos = view.get<Components::Position>(entity);
        int px = static_cast<int>(SimulatorConstants::metersToPixels(pos.x));
        int py = static_cast<int>(SimulatorConstants::metersToPixels(pos.y));

        // Check screen bounds
        if (px >= 0 && px < (int)SimulatorConstants::ScreenLength &&
            py >= 0 && py < (int)SimulatorConstants::ScreenLength)
        {
            auto *dens = registry.try_get<Components::Density>(entity);
            auto *temp = registry.try_get<Components::Temperature>(entity);
            auto *mass = registry.try_get<Components::Mass>(entity);

            auto &pixProp = pixelMap[{px, py}];
            pixProp.add(dens, temp, mass);
        }
    }

    return pixelMap;
}

void Renderer::renderParticles(const entt::registry &registry, ColorMapper colorMapper) {
    auto pixelMap = aggregateParticlesByPixel(registry);

    auto view = registry.view<Components::Position, Components::Shape>();
    for (auto entity : view) {
        // If entity has a boundary component, skip or do something else
        if (registry.any_of<Components::Boundary>(entity)) {
            // continue;
        }

        const auto &pos = view.get<Components::Position>(entity);
        const auto &shape = view.get<Components::Shape>(entity);

        float px = (float)SimulatorConstants::metersToPixels(pos.x);
        float py = (float)SimulatorConstants::metersToPixels(pos.y);

        // Determine color
        sf::Color c;
        if (auto* col = registry.try_get<Components::Color>(entity)) {
            c = sf::Color(col->r, col->g, col->b);
        } else {
            std::pair<int,int> coords((int)px, (int)py);
            auto it = pixelMap.find(coords);
            if (it != pixelMap.end()) {
                c = colorMapper(it->second);
            } else {
                c = sf::Color::White; // fallback
            }
        }

        double angleDeg = 0.0;
        if (registry.any_of<Components::AngularPosition>(entity)) {
            auto &angPos = registry.get<Components::AngularPosition>(entity);
            angleDeg = (float)(angPos.angle * 180.0 / 3.141592653589793);
        }

        // If it's a polygon
        if (auto *poly = registry.try_get<PolygonShape>(entity)) {
            sf::ConvexShape convex;
            convex.setPointCount(poly->vertices.size());

            double rad = angleDeg * 3.141592653589793 / 180.0;
            double ca = std::cos(rad);
            double sa = std::sin(rad);

            for (size_t i = 0; i < poly->vertices.size(); ++i) {
                const auto &v = poly->vertices[i];
                double rx = v.x * ca - v.y * sa;
                double ry = v.x * sa + v.y * ca;
                float fx = (float)SimulatorConstants::metersToPixels(pos.x + rx);
                float fy = (float)SimulatorConstants::metersToPixels(pos.y + ry);
                convex.setPoint(i, sf::Vector2f(fx, fy));
            }
            convex.setFillColor(c);
            window.draw(convex);
        }
        else {
            // Circle or square
            if (shape.type == Components::ShapeType::Circle) {
                float rPixels = (float)std::max(1.0, SimulatorConstants::metersToPixels(shape.size));
                sf::CircleShape circle(rPixels);
                circle.setOrigin(rPixels, rPixels);
                circle.setPosition(px, py);
                circle.setFillColor(c);
                window.draw(circle);
            } else {
                float halfSide = (float)SimulatorConstants::metersToPixels(shape.size);
                float side = halfSide * 2.0f;
                sf::RectangleShape rect(sf::Vector2f(side, side));
                rect.setOrigin(halfSide, halfSide);
                rect.setPosition(px, py);
                rect.setRotation((float)angleDeg);
                rect.setFillColor(c);
                window.draw(rect);
            }
        }
    }

    // Divider line at right edge of simulation area
    sf::Vertex line[] = {
        sf::Vertex(sf::Vector2f((float)SimulatorConstants::ScreenLength, 0.0f), sf::Color::White),
        sf::Vertex(sf::Vector2f((float)SimulatorConstants::ScreenLength, (float)screenHeight), sf::Color::White)
    };
    window.draw(line, 2, sf::Lines);
}

void Renderer::renderFPS(float fps) {
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

    int panelX = (int)SimulatorConstants::ScreenLength + 10;
    int panelY = 10;

    // Pause/Play
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

    // Reset
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

    // Speed buttons
    renderText("Playback Speed:", panelX, panelY, sf::Color::White);
    panelY += 25;

    const std::vector<std::pair<double, std::string>> speeds = {
        {0.25, "0.25x"},
        {0.5, "0.5x"},
        {1.0, "1x"}
    };

    // If there's a simulator state, we can highlight the current timeScale
    const Components::SimulatorState* stPtr = nullptr;
    auto sv = registry.view<Components::SimulatorState>();
    if (!sv.empty()) {
        stPtr = &registry.get<Components::SimulatorState>(sv.front());
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

        if (stPtr && std::fabs(stPtr->timeScale - sp.first) < 0.01) {
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
    for (auto& sc : scenarios) {
        UIButton btn;
        btn.rect = sf::IntRect(panelX, panelY, 150, 30);
        btn.label = sc.second;
        btn.scenario = sc.first;
        btn.isSpecialButton = false;
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