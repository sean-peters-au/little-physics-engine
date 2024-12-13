#ifndef RENDERER_H
#define RENDERER_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <entt/entt.hpp>
#include <string>
#include <functional>
#include <unordered_map>
#include "components.h"

// Forward declare simulation type
namespace SimulatorConstants {
    enum class SimulationType;
}

// Define a hash function for pair<int,int>
struct PixelCoordHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        // A simple combination hash
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

    using ColorMapper = std::function<SDL_Color(const PixelProperties&)>;

    static SDL_Color whiteColor(const PixelProperties&) {
        return {255, 255, 255, 255};
    }
    static SDL_Color densityGrayscale(const PixelProperties& props);
    static SDL_Color temperatureHeatmap(const PixelProperties& props);

public:
    Renderer(int screenWidth, int screenHeight);
    ~Renderer();

    bool init();
    void clear();
    void present();

    void renderParticles(const entt::registry& registry, ColorMapper colorMapper = whiteColor);
    void renderFPS(float fps);

    // New UI rendering method
    void renderUI(bool paused, SimulatorConstants::SimulationType scenario);

    bool isInitialized() const { return initialized; }

private:
    SDL_Window* window;
    SDL_Renderer* renderer;
    TTF_Font* font;
    bool initialized;
    int screenWidth;
    int screenHeight;

    std::unordered_map<std::pair<int,int>, PixelProperties, PixelCoordHash> aggregateParticlesByPixel(const entt::registry& registry);

    void renderText(const std::string& text, int x, int y, SDL_Color color = {255,255,255,255});
};

#endif // RENDERER_H