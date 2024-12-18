#ifndef RENDERER_H
#define RENDERER_H

#include <entt/entt.hpp>
#include <string>
#include <functional>
#include <unordered_map>
#include <vector>
#include <SFML/Graphics.hpp>

#include "nbody/components/basic.hpp"
#include "nbody/components/sim.hpp"
#include "nbody/core/constants.hpp"

// Hash for pixel coords
struct PixelCoordHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        std::size_t h1 = std::hash<int>()(p.first);
        std::size_t h2 = std::hash<int>()(p.second);
        return h1 ^ (h2 + 0x9e3779b97f4a7c16ULL + (h1<<6) + (h1>>2));
    }
};

class Renderer {
public:
    struct PixelProperties {
        double density = 0.0;
        double temperature = 0.0;
        double total_mass = 0.0;
        int particle_count = 0;
        void add(const Components::Density* density,
                 const Components::Temperature* temp,
                 const Components::Mass* mass) {
            if (density) this->density += density->value;
            if (temp) {
                if (mass) {
                    double weight = mass->value;
                    this->temperature = (this->temperature * total_mass + temp->value * weight) / (total_mass + weight);
                } else {
                    this->temperature = (this->temperature * particle_count + temp->value) / (particle_count + 1);
                }
            }
            if (mass) this->total_mass += mass->value;
            particle_count++;
        }
    };

    using ColorMapper = std::function<sf::Color(const PixelProperties&)>;
    static sf::Color whiteColor(const PixelProperties&) { return sf::Color::White; }

    Renderer(int screenWidth, int screenHeight);
    ~Renderer();

    bool init();
    void clear();
    void present();

    void renderParticles(const entt::registry& registry, ColorMapper colorMapper = whiteColor);
    void renderFPS(float fps);

    // UI rendering
    void renderUI(const entt::registry& registry, 
                  bool paused, 
                  SimulatorConstants::SimulationType currentScenario,
                  const std::vector<std::pair<SimulatorConstants::SimulationType,std::string>>& scenarios,
                  bool showPausePlayHighlight,
                  bool showResetHighlight,
                  SimulatorConstants::SimulationType highlightedScenario);

    void renderText(const std::string& text, int x, int y, sf::Color color = sf::Color::White);

    bool isInitialized() const { return initialized; }

    // Button representation
    struct UIButton {
        sf::IntRect rect;
        std::string label;
        bool isSpecialButton;
        SimulatorConstants::SimulationType scenario;
        double speedMultiplier;

        UIButton() 
            : rect()
            , label()
            , isSpecialButton(false)
            , scenario(SimulatorConstants::SimulationType::CELESTIAL_GAS)
            , speedMultiplier(1.0)
        {}
    };

    std::vector<UIButton> scenarioButtons;
    UIButton pausePlayButton;
    UIButton resetButton;
    std::vector<UIButton> speedButtons;

    sf::RenderWindow& getWindow() { return window; }

private:
    sf::RenderWindow window;
    sf::Font font;
    bool initialized;
    int screenWidth;
    int screenHeight;

    std::unordered_map<std::pair<int,int>, PixelProperties, PixelCoordHash> aggregateParticlesByPixel(const entt::registry& registry);
};

#endif // RENDERER_H