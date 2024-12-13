#include "nbody/rendering/renderer.hpp"
#include "nbody/core/constants.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

// Example color mappers (if needed)
static sf::Color densityGrayscale(const Renderer::PixelProperties& props) {
    const double max_density = 100.0;
    uint8_t intensity = static_cast<uint8_t>(
        std::min(255.0, (props.density / max_density) * 255.0)
    );
    return sf::Color(intensity, intensity, intensity);
}

static sf::Color temperatureHeatmap(const Renderer::PixelProperties& props) {
    const double max_temp = 1000.0;
    double t = std::min(1.0, props.temperature / max_temp);

    uint8_t r = static_cast<uint8_t>(t * 255);
    uint8_t b = static_cast<uint8_t>((1.0 - t) * 255);
    uint8_t g = static_cast<uint8_t>(std::min(r, b) / 2);

    return sf::Color(r, g, b);
}

Renderer::Renderer(int screenWidth, int screenHeight)
    : window(), font(), initialized(false), screenWidth(screenWidth), screenHeight(screenHeight)
{
}

Renderer::~Renderer() {
    // SFML automatically cleans up resources in destructors
}

bool Renderer::init() {
    window.create(sf::VideoMode(screenWidth, screenHeight), "N-Body Simulator");
    
    // Update font path to look in assets/fonts directory
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

    // Draw each particle as a circle with correct radius
    auto view = registry.view<Components::Position, Components::Radius>();
    for (auto entity : view) {
        const auto& pos = view.get<Components::Position>(entity);
        const auto& radius = view.get<Components::Radius>(entity);

        // Convert position from meters to pixels
        float px = static_cast<float>(SimulatorConstants::metersToPixels(pos.x));
        float py = static_cast<float>(SimulatorConstants::metersToPixels(pos.y));
        
        // Convert radius from meters to pixels (minimum 1 pixel)
        float radiusPixels = static_cast<float>(
            std::max(1.0, SimulatorConstants::metersToPixels(radius.value))
        );

        // Create circle shape
        sf::CircleShape circle(radiusPixels);
        circle.setPosition(px - radiusPixels, py - radiusPixels); // Center the circle on the position
        
        // Get color based on density/temperature/etc at this position
        std::pair<int,int> coords(static_cast<int>(px), static_cast<int>(py));
        auto it = pixelMap.find(coords);
        if (it != pixelMap.end()) {
            circle.setFillColor(colorMapper(it->second));
        } else {
            circle.setFillColor(sf::Color::White);
        }

        window.draw(circle);
    }

    // Draw UI separator line
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

void Renderer::renderUI(bool paused,
                        SimulatorConstants::SimulationType currentScenario,
                        const std::vector<std::pair<SimulatorConstants::SimulationType,std::string>>& scenarios,
                        bool showPausePlayHighlight,
                        bool showResetHighlight,
                        SimulatorConstants::SimulationType highlightedScenario) {
    scenarioButtons.clear();

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