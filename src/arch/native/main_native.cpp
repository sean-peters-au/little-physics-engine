/**
 * @file main_native.cpp
 * @brief Main entry point for the native platform
 *
 * This file contains the main entry point for the native platform.
 * It initializes the simulator, builds the scenario list, and starts the main loop.
 */

// Required for metal
#define NS_PRIVATE_IMPLEMENTATION
#define CA_PRIVATE_IMPLEMENTATION
#define MTL_PRIVATE_IMPLEMENTATION

#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>

#include <iostream>
#include <SFML/Graphics.hpp>
#include <chrono>

#include "nbody/core/simulator.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/arch/native/renderer_native.hpp"
#include "nbody/components/sim.hpp"
#include "nbody/core/profile.hpp"

/**
 * @brief Helper: build a scenario list from getAllScenarios()
 * 
 * Returns a vector of (SimulationType, scenarioName)
 */
static std::vector<std::pair<SimulatorConstants::SimulationType, std::string>> buildScenarioList() {
    std::vector<std::pair<SimulatorConstants::SimulationType, std::string>> scenarios;
    for (auto s : SimulatorConstants::getAllScenarios()) {
        scenarios.emplace_back(s, SimulatorConstants::getScenarioName(s));
    }
    return scenarios;
}

/**
 * @brief Helper: apply new scenario and reset simulator
 * 
 * This calls initializeConstants(...), updates the baseTimeAcceleration, 
 * sets the scenario in the simulator, then calls simulator.reset().
 */
static void updateSimulatorState(ECSSimulator& simulator,
                                 SimulatorConstants::SimulationType scenario)
{
    // First re-init global constants to safe defaults
    SimulatorConstants::initializeConstants(scenario);

    // Pass the scenario to the simulator
    simulator.setScenario(scenario);

    // If we want to update the baseTimeAcceleration inside the simulator’s registry:
    auto& registry = simulator.getRegistry();
    auto stateView = registry.view<Components::SimulatorState>();
    if (!stateView.empty()) {
        auto& state = registry.get<Components::SimulatorState>(stateView.front());
        state.baseTimeAcceleration = SimulatorConstants::TimeAcceleration;
    }

    // Now fully reset the ECS with that scenario
    simulator.reset();
}

int main(int /*unused*/, char** /*unused*/) {
    // Choose an initial scenario from your enum
    auto initialScenario = SimulatorConstants::SimulationType::KEPLERIAN_DISK;

    // Create a larger window: the simulation area plus an extra 200px panel
    Renderer renderer(SimulatorConstants::ScreenLength + 200,
                      SimulatorConstants::ScreenLength);
    if (!renderer.init()) {
        return 1;
    }

    ECSSimulator simulator;

    // Build scenario list for the UI
    auto scenarioList = buildScenarioList();

    // Initialize with chosen scenario
    updateSimulatorState(simulator, initialScenario);

    simulator.init();

    // Desired FPS
    const int fps = 120;
    const int frameDelay = 1000 / fps; // in ms

    sf::Clock const fpsClock;      // tracks time for fps calc
    sf::Clock frameClock;    // tracks dt each frame
    float avgFPS = 0.0F;
    int frameCount = 0;
    float fpsTimer = 0.0F;   // accumulates time for 1-second check

    bool running = true;
    bool paused = false;
    bool stepFrame = false;

    // For UI highlights
    bool highlightPausePlay = false;
    bool highlightReset = false;
    auto highlightedScenario =
        static_cast<SimulatorConstants::SimulationType>(-1);

    while (running) {
        // Poll events
        sf::RenderWindow& window = renderer.getWindow();
        sf::Vector2i const mousePos = sf::Mouse::getPosition(window);
        int const mouseX = mousePos.x;
        int const mouseY = mousePos.y;

        highlightPausePlay = false;
        highlightReset = false;
        highlightedScenario = static_cast<SimulatorConstants::SimulationType>(-1);

        // For "hover" effect on pause/play
        if (mouseX >= renderer.pausePlayButton.rect.left &&
            mouseX <= renderer.pausePlayButton.rect.left + renderer.pausePlayButton.rect.width &&
            mouseY >= renderer.pausePlayButton.rect.top &&
            mouseY <= renderer.pausePlayButton.rect.top + renderer.pausePlayButton.rect.height)
        {
            highlightPausePlay = true;
        }

        // For "hover" effect on reset
        if (mouseX >= renderer.resetButton.rect.left &&
            mouseX <= renderer.resetButton.rect.left + renderer.resetButton.rect.width &&
            mouseY >= renderer.resetButton.rect.top &&
            mouseY <= renderer.resetButton.rect.top + renderer.resetButton.rect.height)
        {
            highlightReset = true;
        }

        // For scenario buttons
        for (auto& btn : renderer.scenarioButtons) {
            if (mouseX >= btn.rect.left && mouseX <= btn.rect.left + btn.rect.width &&
                mouseY >= btn.rect.top && mouseY <= btn.rect.top + btn.rect.height)
            {
                highlightedScenario = btn.scenario;
                break;
            }
        }

        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                running = false;
            }
            else if (event.type == sf::Event::MouseButtonPressed &&
                     event.mouseButton.button == sf::Mouse::Left)
            {
                // Click checks
                // Pause/Play
                if (mouseX >= renderer.pausePlayButton.rect.left &&
                    mouseX <= renderer.pausePlayButton.rect.left + renderer.pausePlayButton.rect.width &&
                    mouseY >= renderer.pausePlayButton.rect.top &&
                    mouseY <= renderer.pausePlayButton.rect.top + renderer.pausePlayButton.rect.height)
                {
                    paused = !paused;
                }
                // Reset
                if (mouseX >= renderer.resetButton.rect.left &&
                    mouseX <= renderer.resetButton.rect.left + renderer.resetButton.rect.width &&
                    mouseY >= renderer.resetButton.rect.top &&
                    mouseY <= renderer.resetButton.rect.top + renderer.resetButton.rect.height)
                {
                    simulator.reset();
                    paused = false;
                }

                // Scenario selection
                for (auto& btn : renderer.scenarioButtons) {
                    if (btn.rect.contains(sf::Vector2i(mouseX, mouseY))) {
                        updateSimulatorState(simulator, btn.scenario);
                        paused = false;
                        break;
                    }
                }

                // Speed buttons
                for (auto& btn : renderer.speedButtons) {
                    if (btn.rect.contains(sf::Vector2i(mouseX, mouseY))) {
                        auto view = simulator.getRegistry().view<Components::SimulatorState>();
                        if (!view.empty()) {
                            auto& st = simulator.getRegistry().get<Components::SimulatorState>(view.front());
                            st.timeScale = btn.speedMultiplier;
                        }
                        break;
                    }
                }

                // Color scheme buttons
                for (auto& btn : renderer.colorSchemeButtons) {
                    if (btn.rect.contains(sf::Vector2i(mouseX, mouseY))) {
                        if (btn.label == "Default") { 
                            renderer.setColorScheme(Renderer::ColorScheme::DEFAULT);
                        } else if (btn.label == "Sleep") { 
                            renderer.setColorScheme(Renderer::ColorScheme::SLEEP);
                        } else if (btn.label == "Temperature") { 
                            renderer.setColorScheme(Renderer::ColorScheme::TEMPERATURE);
}
                        break;
                    }
                }

                // Add debug button handler
                renderer.handleUIClick(mouseX, mouseY);  // This will handle the debug button toggle

                // Next Frame button
                if (paused && mouseX >= renderer.nextFrameButton.rect.left &&
                    mouseX <= renderer.nextFrameButton.rect.left + renderer.nextFrameButton.rect.width &&
                    mouseY >= renderer.nextFrameButton.rect.top &&
                    mouseY <= renderer.nextFrameButton.rect.top + renderer.nextFrameButton.rect.height)
                {
                    stepFrame = true;
                }
            }
            else if (event.type == sf::Event::KeyPressed) {
                // Keyboard shortcuts
                if (event.key.code == sf::Keyboard::Space) {  // Space for next frame
                    if (paused) { stepFrame = true;
}
                }
                else if (event.key.code == sf::Keyboard::P) {  // P for pause
                    paused = !paused;
                }
                else if (event.key.code == sf::Keyboard::R) {
                    simulator.reset();
                    paused = false;
                }
                else if (event.key.code == sf::Keyboard::Num1) {
                    updateSimulatorState(simulator, SimulatorConstants::SimulationType::KEPLERIAN_DISK);
                    paused = false;
                }
                else if (event.key.code == sf::Keyboard::Num2) {
                    updateSimulatorState(simulator, SimulatorConstants::SimulationType::ISOTHERMAL_BOX);
                    paused = false;
                }
                else if (event.key.code == sf::Keyboard::Num3) {
                    updateSimulatorState(simulator, SimulatorConstants::SimulationType::RANDOM_POLYGONS);
                    paused = false;
                }
                else if (event.key.code == sf::Keyboard::Escape) {
                    running = false;
                }
            }
        }

        // Step the simulation if not paused or stepping one frame
        if (!paused || stepFrame) {
            simulator.tick();
            stepFrame = false;  // Reset the step flag
        }

        // Start rendering
        renderer.clear();
        renderer.renderParticles(simulator.getRegistry());

        // Frame-based FPS measure
        float const dt = frameClock.restart().asMilliseconds();
        fpsTimer += dt;
        frameCount++;
        if (fpsTimer >= 1000.0F) {
            avgFPS = static_cast<float>(frameCount) / (fpsTimer / 1000.0F);
            frameCount = 0;
            fpsTimer = 0.0F;
        }

        renderer.renderFPS(avgFPS);

        // UI overlay
        renderer.renderUI(simulator.getRegistry(),
                          paused,
                          // show which scenario is currently "active"
                          // (In a bigger design you'd track it inside the simulator or scenarioPtr)
                          // we'll just pass the scenario set earlier, or store it in the simulator
                          SimulatorConstants::SimulationType::KEPLERIAN_DISK,
                          scenarioList,
                          highlightPausePlay,
                          highlightReset,
                          highlightedScenario);

        renderer.present();

        // Attempt frame limit
        if (frameDelay > dt) {
            sf::sleep(sf::milliseconds(static_cast<int>(frameDelay - dt)));
        }

        // Print & reset profiler stats every 10s
        auto now = std::chrono::steady_clock::now();
        static auto lastStats = now;
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - lastStats).count();
        if (elapsed >= 10) {
            Profiling::Profiler::printStats();
            Profiling::Profiler::reset();
            lastStats = now;
        }
    }

    return 0;
}