/**
 * @file main_native.cpp
 * @brief Main entry point for the native platform.
 *
 * This file creates a SimManager instance, initializes it, and runs the main loop.
 */

#define NS_PRIVATE_IMPLEMENTATION
#define CA_PRIVATE_IMPLEMENTATION
#define MTL_PRIVATE_IMPLEMENTATION

#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>

#include <chrono>
#include <SFML/System.hpp>

#include "core/profile.hpp"
#include "sim_manager.hpp"

int main(int /*unused*/, char** /*unused*/)
{
    SimManager simManager;
    if (!simManager.init())
    {
        return 1;
    }

    // Desired FPS
    const int fps = 120;
    const int frameDelay = 1000 / fps; // in milliseconds

    sf::Clock frameClock;
    float avgFPS = 0.0F;
    float fpsTimer = 0.0F;
    int frameCount = 0;

    bool running = true;

    // For periodic profiler stats
    auto lastStats = std::chrono::steady_clock::now();

    while (running)
    {
        running = simManager.handleEvents();

        simManager.tick();

        // Measure frame time for FPS
        float dt = frameClock.restart().asMilliseconds();
        fpsTimer += dt;
        frameCount++;
        if (fpsTimer >= 1000.0F)
        {
            avgFPS = static_cast<float>(frameCount) / (fpsTimer / 1000.0F);
            frameCount = 0;
            fpsTimer = 0.0F;
        }

        simManager.render(avgFPS);

        // Simple frame limiting
        if (frameDelay > dt)
        {
            sf::sleep(sf::milliseconds(static_cast<int>(frameDelay - dt)));
        }

        // Print & reset profiler stats every 10s
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - lastStats).count();
        if (elapsed >= 10)
        {
            Profiling::Profiler::printStats();
            Profiling::Profiler::reset();
            lastStats = now;
        }
    }

    return 0;
}
