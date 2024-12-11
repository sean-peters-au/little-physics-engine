#include "renderer.h"
#include <iostream>
#include <sstream>
#include <iomanip>

Renderer::Renderer(int screenWidth, int screenHeight)
    : window(nullptr)
    , renderer(nullptr)
    , font(nullptr)
    , initialized(false)
    , screenWidth(screenWidth)
    , screenHeight(screenHeight)
{
}

Renderer::~Renderer() {
    if (font) {
        TTF_CloseFont(font);
    }
    if (renderer) {
        SDL_DestroyRenderer(renderer);
    }
    if (window) {
        SDL_DestroyWindow(window);
    }
    TTF_Quit();
    SDL_Quit();
}

bool Renderer::init() {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return false;
    }
    
    if (TTF_Init() != 0) {
        std::cout << "TTF_Init Error: " << SDL_GetError() << std::endl;
        return false;
    }

    if (SDL_CreateWindowAndRenderer(screenWidth, screenHeight, 0, &window, &renderer) != 0) {
        std::cout << "Window/Renderer creation error: " << SDL_GetError() << std::endl;
        return false;
    }

    // Initialize font for FPS counter
    font = TTF_OpenFont("/System/Library/Fonts/Helvetica.ttc", 16);
    if (!font) {
        std::cout << "TTF_OpenFont Error: " << TTF_GetError() << std::endl;
        return false;
    }

    initialized = true;
    return true;
}

void Renderer::clear() {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
}

void Renderer::present() {
    SDL_RenderPresent(renderer);
}

std::unordered_map<std::pair<int, int>, Renderer::PixelProperties, PixelCoordHash>
Renderer::aggregateParticlesByPixel(const entt::registry& registry) {
    std::unordered_map<std::pair<int, int>, PixelProperties, PixelCoordHash> pixelMap;
    
    auto view = registry.view<Components::Position>();
    
    for (auto entity : view) {
        const auto& pos = view.get<Components::Position>(entity);
        
        // Get pixel coordinates
        int px = static_cast<int>(pos.x);
        int py = static_cast<int>(pos.y);
        
        // Get optional components
        auto* density = registry.try_get<Components::Density>(entity);
        auto* temp = registry.try_get<Components::Temperature>(entity);
        auto* mass = registry.try_get<Components::Mass>(entity);
        
        // Add particle properties to the pixel
        pixelMap[{px, py}].add(density, temp, mass);
    }
    
    return pixelMap;
}

void Renderer::renderParticles(const entt::registry& registry, ColorMapper colorMapper) {
    // First, aggregate particles by pixel
    auto pixelMap = aggregateParticlesByPixel(registry);
    
    // Then render each pixel with its aggregated properties
    for (const auto& [coords, props] : pixelMap) {
        SDL_Color color = colorMapper(props);
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        
        // Draw the pixel
        SDL_Rect point = {
            coords.first,    // x
            coords.second,   // y
            1,              // width
            1               // height
        };
        SDL_RenderFillRect(renderer, &point);
    }
}

void Renderer::renderFPS(float fps) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1) << fps << " FPS";
    
    SDL_Color textColor = {255, 255, 255, 255};
    SDL_Surface* surface = TTF_RenderText_Solid(font, ss.str().c_str(), textColor);
    if (surface) {
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
        if (texture) {
            SDL_Rect dstRect = {10, 10, surface->w, surface->h};
            SDL_RenderCopy(renderer, texture, NULL, &dstRect);
            SDL_DestroyTexture(texture);
        }
        SDL_FreeSurface(surface);
    }
} 