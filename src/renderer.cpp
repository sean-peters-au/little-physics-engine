#include "renderer.h"
#include "simulator_constants.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <functional>

SDL_Color Renderer::densityGrayscale(const PixelProperties& props) {
    const double max_density = 100.0;
    uint8_t intensity = static_cast<uint8_t>(
        std::min(255.0, (props.density / max_density) * 255.0)
    );
    return {intensity, intensity, intensity, 255};
}

SDL_Color Renderer::temperatureHeatmap(const PixelProperties& props) {
    const double max_temp = 1000.0;
    double t = std::min(1.0, props.temperature / max_temp);

    uint8_t r = static_cast<uint8_t>(t * 255);
    uint8_t b = static_cast<uint8_t>((1.0 - t) * 255);
    uint8_t g = static_cast<uint8_t>(std::min(r, b) / 2);

    return {r, g, b, 255};
}

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

std::unordered_map<std::pair<int,int>, Renderer::PixelProperties, PixelCoordHash>
Renderer::aggregateParticlesByPixel(const entt::registry& registry) {
    std::unordered_map<std::pair<int,int>, PixelProperties, PixelCoordHash> pixelMap;

    auto view = registry.view<Components::Position>();
    for (auto entity : view) {
        const auto &pos = view.get<Components::Position>(entity);
        int px = static_cast<int>(SimulatorConstants::metersToPixels(pos.x));
        int py = static_cast<int>(SimulatorConstants::metersToPixels(pos.y));

        auto* density = registry.try_get<Components::Density>(entity);
        auto* temp = registry.try_get<Components::Temperature>(entity);
        auto* mass = registry.try_get<Components::Mass>(entity);

        std::pair<int,int> coords(px,py);
        pixelMap[coords].add(density, temp, mass);
    }

    return pixelMap;
}

void Renderer::renderParticles(const entt::registry& registry, ColorMapper colorMapper) {
    auto pixelMap = aggregateParticlesByPixel(registry);
    for (const auto& [coords, props] : pixelMap) {
        SDL_Color color = colorMapper(props);
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        SDL_Rect point = { coords.first, coords.second, 1, 1 };
        SDL_RenderFillRect(renderer, &point);
    }
}

void Renderer::renderFPS(float fps) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1) << fps << " FPS";
    renderText(ss.str(), 10, 10, {255,255,255,255});
}

void Renderer::renderText(const std::string& text, int x, int y, SDL_Color color) {
    SDL_Surface* surface = TTF_RenderText_Solid(font, text.c_str(), color);
    if (surface) {
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
        if (texture) {
            SDL_Rect dstRect = {x, y, surface->w, surface->h};
            SDL_RenderCopy(renderer, texture, NULL, &dstRect);
            SDL_DestroyTexture(texture);
        }
        SDL_FreeSurface(surface);
    }
}

void Renderer::renderUI(bool paused, SimulatorConstants::SimulationType scenario) {
    std::string scenarioName;
    switch (scenario) {
        case SimulatorConstants::SimulationType::CELESTIAL_GAS:
            scenarioName = "CELESTIAL_GAS";
            break;
        case SimulatorConstants::SimulationType::ISOTHERMAL_BOX:
            scenarioName = "ISOTHERMAL_BOX";
            break;
        default:
            scenarioName = "UNKNOWN";
            break;
    }

    std::string stateText = "Scenario: " + scenarioName + (paused ? " [PAUSED]" : " [RUNNING]");
    renderText(stateText, 10, 30, {255,255,255,255});

    std::string instructions =
        "Controls:\n"
        "P: Pause/Play\n"
        "R: Reset\n"
        "1: CELESTIAL_GAS\n"
        "2: ISOTHERMAL_BOX";

    int line_y = 50;
    std::stringstream instr(instructions);
    std::string line;
    while (std::getline(instr, line)) {
        renderText(line, 10, line_y, {255,255,255,255});
        line_y += 20;
    }
}