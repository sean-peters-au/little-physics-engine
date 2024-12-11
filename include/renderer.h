#ifndef RENDERER_H
#define RENDERER_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <entt/entt.hpp>
#include <string>
#include <functional>
#include <unordered_map>
#include "components.h"

// Hash function for pixel coordinates
struct PixelCoordHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

class Renderer {
public:
    // Struct to hold aggregated properties for a pixel
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
                // Weighted average for temperature
                if (mass) {
                    double weight = mass->value;
                    this->temperature = (this->temperature * total_mass + temp->value * weight) / (total_mass + weight);
                } else {
                    // Simple average if no mass available
                    this->temperature = (this->temperature * particle_count + temp->value) / (particle_count + 1);
                }
            }
            if (mass) this->total_mass += mass->value;
            particle_count++;
        }
    };

    // Color mapping function type - takes pixel properties and returns RGB values
    using ColorMapper = std::function<SDL_Color(const PixelProperties&)>;
    
    // Predefined color mappers
    static SDL_Color whiteColor(const PixelProperties&) {
        return {255, 255, 255, 255};
    }
    
    static SDL_Color densityGrayscale(const PixelProperties& props) {
        // Map density to grayscale (0.0 -> 0, max_density -> 255)
        const double max_density = 100.0; // Adjust based on your simulation
        uint8_t intensity = static_cast<uint8_t>(
            std::min(255.0, (props.density / max_density) * 255.0)
        );
        return {intensity, intensity, intensity, 255};
    }
    
    static SDL_Color temperatureHeatmap(const PixelProperties& props) {
        // Map temperature to a heat color (blue -> red)
        const double max_temp = 1000.0; // Adjust based on your simulation
        double t = std::min(1.0, props.temperature / max_temp);
        
        uint8_t r = static_cast<uint8_t>(t * 255);
        uint8_t b = static_cast<uint8_t>((1.0 - t) * 255);
        uint8_t g = static_cast<uint8_t>(std::min(r, b) / 2);
        
        return {r, g, b, 255};
    }

public:
    Renderer(int screenWidth, int screenHeight);
    ~Renderer();

    bool init();
    void clear();
    void present();
    
    // Enhanced rendering methods
    void renderParticles(const entt::registry& registry, ColorMapper colorMapper = whiteColor);
    void renderFPS(float fps);
    bool isInitialized() const { return initialized; }
    
private:
    SDL_Window* window;
    SDL_Renderer* renderer;
    TTF_Font* font;
    bool initialized;
    int screenWidth;
    int screenHeight;

    // Helper method to aggregate particles by pixel
    std::unordered_map<std::pair<int, int>, PixelProperties, PixelCoordHash>
    aggregateParticlesByPixel(const entt::registry& registry);
};

#endif // RENDERER_H 