#include "nbody/rendering/renderer.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sim.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

static sf::Color densityGrayscale(const Renderer::PixelProperties& props) {
    const double max_density = 100.0;
    uint8_t intensity = static_cast<uint8_t>(
        std::min(255.0, (props.density / max_density) * 255.0)
    );
    return sf::Color(intensity, intensity, intensity);
}

Renderer::Renderer(int screenWidth, int screenHeight)
    : window(), font(), initialized(false), screenWidth(screenWidth), screenHeight(screenHeight)
{
}

Renderer::~Renderer() {}

bool Renderer::init() {
    window.create(sf::VideoMode(screenWidth, screenHeight), "N-Body Simulator");

    if (!font.loadFromFile("assets/fonts/arial.ttf")) {
        std::cout << "Failed to load font \"assets/fonts/arial.ttf\"" << std::endl;
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
Renderer::aggregateParticlesByPixel(const entt::registry& registry) {
    std::unordered_map<std::pair<int,int>, PixelProperties, PixelCoordHash> pixelMap;

    auto view = registry.view<Components::Position>();
    for (auto entity : view) {
        const auto &pos = view.get<Components::Position>(entity);
        int px = static_cast<int>(SimulatorConstants::metersToPixels(pos.x));
        int py = static_cast<int>(SimulatorConstants::metersToPixels(pos.y));

        if (px >=0 && px < (int)SimulatorConstants::ScreenLength &&
            py >=0 && py < (int)SimulatorConstants::ScreenLength) {

            auto* density = registry.try_get<Components::Density>(entity);
            auto* temp = registry.try_get<Components::Temperature>(entity);
            auto* mass = registry.try_get<Components::Mass>(entity);

            std::pair<int,int> coords(px,py);
            pixelMap[coords].add(density, temp, mass);
        }
    }

    return pixelMap;
}

void Renderer::renderParticles(const entt::registry& registry, ColorMapper colorMapper) {
    auto pixelMap = aggregateParticlesByPixel(registry);

    auto view = registry.view<Components::Position, Components::Shape>();
    for (auto entity : view) {
        const auto& pos = view.get<Components::Position>(entity);
        const auto& shape = view.get<Components::Shape>(entity);

        float px = static_cast<float>(SimulatorConstants::metersToPixels(pos.x));
        float py = static_cast<float>(SimulatorConstants::metersToPixels(pos.y));

        // Use particle color if available, otherwise use density/temperature mapping
        sf::Color c;
        if (auto* color = registry.try_get<Components::Color>(entity)) {
            c = sf::Color(color->r, color->g, color->b);
        } else {
            std::pair<int,int> coords(static_cast<int>(px), static_cast<int>(py));
            auto it = pixelMap.find(coords);
            if (it != pixelMap.end()) {
                c = colorMapper(it->second);
            } else {
                c = sf::Color::White;  // fallback
            }
        }

        double angleDeg = 0.0;
        if (registry.any_of<Components::AngularPosition>(entity)) {
            auto &angPos = registry.get<Components::AngularPosition>(entity);
            angleDeg = (float)(angPos.angle * 180.0 / 3.141592653589793);
        }

        if (shape.type == Components::ShapeType::Circle) {
            float radiusPixels = static_cast<float>(std::max(1.0, SimulatorConstants::metersToPixels(shape.size)));
            sf::CircleShape circle(radiusPixels);
            circle.setOrigin(radiusPixels, radiusPixels);
            circle.setPosition(px, py);
            // Rotation doesn't matter visually for a circle
            circle.setFillColor(c);
            window.draw(circle);
        } else {
            // Square
            float halfSidePx = static_cast<float>(SimulatorConstants::metersToPixels(shape.size));
            float sidePx = halfSidePx * 2.0f;
            sf::RectangleShape rect(sf::Vector2f(sidePx, sidePx));
            rect.setOrigin(halfSidePx, halfSidePx);
            rect.setPosition(px, py);
            rect.setRotation((float)angleDeg);
            rect.setFillColor(c);
            window.draw(rect);
        }
    }

    sf::Vertex line[] = {
        sf::Vertex(sf::Vector2f((float)SimulatorConstants::ScreenLength, 0.f), sf::Color::White),
        sf::Vertex(sf::Vector2f((float)SimulatorConstants::ScreenLength, (float)screenHeight), sf::Color::White)
    };
    window.draw(line, 2, sf::Lines);
}

void Renderer::renderFPS(float fps) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1) << fps << " FPS";
    renderText(ss.str(), 10, 10, sf::Color::White);
}

void Renderer::renderText(const std::string& text, int x, int y, sf::Color color) {
    sf::Text sfText;
    sfText.setFont(font);
    sfText.setString(text);
    sfText.setCharacterSize(16);
    sfText.setFillColor(color);
    sfText.setPosition((float)x, (float)y);
    window.draw(sfText);
}

void Renderer::renderUI(const entt::registry& registry,
                       bool paused,
                       SimulatorConstants::SimulationType currentScenario,
                       const std::vector<std::pair<SimulatorConstants::SimulationType,std::string>>& scenarios,
                       bool showPausePlayHighlight,
                       bool showResetHighlight,
                       SimulatorConstants::SimulationType highlightedScenario) {
    scenarioButtons.clear();
    speedButtons.clear();

    int panelX = (int)SimulatorConstants::ScreenLength + 10;
    int panelY = 10;

    // Pause/Play button
    std::string pauseLabel = paused ? "Play" : "Pause";
    pausePlayButton.rect = sf::IntRect(panelX, panelY, 80, 30);
    pausePlayButton.label = pauseLabel;
    pausePlayButton.isSpecialButton = true;
    pausePlayButton.scenario = currentScenario;

    {
        sf::RectangleShape btnShape(sf::Vector2f((float)pausePlayButton.rect.width, (float)pausePlayButton.rect.height));
        btnShape.setPosition((float)pausePlayButton.rect.left, (float)pausePlayButton.rect.top);
        if (showPausePlayHighlight) {
            btnShape.setFillColor(sf::Color(200,200,0));
        } else {
            btnShape.setFillColor(sf::Color(100,100,100));
        }
        btnShape.setOutlineColor(sf::Color::White);
        btnShape.setOutlineThickness(1.f);
        window.draw(btnShape);
        renderText(pauseLabel, panelX+5, panelY+5);
    }
    panelY += 40;

    // Reset button
    resetButton.rect = sf::IntRect(panelX, panelY, 80, 30);
    resetButton.label = "Reset";
    resetButton.isSpecialButton = true;
    resetButton.scenario = currentScenario;

    {
        sf::RectangleShape btnShape(sf::Vector2f((float)resetButton.rect.width, (float)resetButton.rect.height));
        btnShape.setPosition((float)resetButton.rect.left, (float)resetButton.rect.top);
        if (showResetHighlight) {
            btnShape.setFillColor(sf::Color(200,200,0));
        } else {
            btnShape.setFillColor(sf::Color(100,100,100));
        }
        btnShape.setOutlineColor(sf::Color::White);
        btnShape.setOutlineThickness(1.f);
        window.draw(btnShape);
        renderText("Reset", panelX+5, panelY+5);
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

    for (const auto& speed : speeds) {
        UIButton btn;
        btn.rect = sf::IntRect(panelX, panelY, 60, 25);
        btn.label = speed.second;
        btn.isSpecialButton = true;
        btn.speedMultiplier = speed.first;

        sf::RectangleShape btnShape(sf::Vector2f((float)btn.rect.width, (float)btn.rect.height));
        btnShape.setPosition((float)btn.rect.left, (float)btn.rect.top);

        // Get simulator state from registry
        const auto& state = registry.get<Components::SimulatorState>(
            registry.view<Components::SimulatorState>().front()
        );

        // Highlight current speed based on simulator state
        if (std::abs(state.timeScale - speed.first) < 0.01) {
            btnShape.setFillColor(sf::Color(0, 200, 0));
        } else {
            btnShape.setFillColor(sf::Color(100, 100, 100));
        }

        btnShape.setOutlineColor(sf::Color::White);
        btnShape.setOutlineThickness(1.f);
        window.draw(btnShape);
        renderText(btn.label, panelX + 5, panelY + 3);

        speedButtons.push_back(btn);
        panelY += 35;
    }

    panelY += 15;  // Extra spacing before scenarios

    renderText("Scenarios:", panelX, panelY, sf::Color::White);
    panelY += 20;

    for (auto& sc : scenarios) {
        UIButton btn;
        btn.rect = sf::IntRect(panelX, panelY, 150, 30);
        btn.label = sc.second;
        btn.scenario = sc.first;
        btn.isSpecialButton = false;

        bool isCurrent = (sc.first == currentScenario);
        bool isHover = (sc.first == highlightedScenario);

        sf::RectangleShape btnShape(sf::Vector2f((float)btn.rect.width, (float)btn.rect.height));
        btnShape.setPosition((float)btn.rect.left, (float)btn.rect.top);

        if (isCurrent) {
            btnShape.setFillColor(sf::Color(0,200,0));
        } else if (isHover) {
            btnShape.setFillColor(sf::Color(200,200,0));
        } else {
            btnShape.setFillColor(sf::Color(100,100,100));
        }

        btnShape.setOutlineColor(sf::Color::White);
        btnShape.setOutlineThickness(1.f);
        window.draw(btnShape);
        renderText(btn.label, panelX+5, panelY+5);

        panelY += 40;
        scenarioButtons.push_back(btn);
    }
}