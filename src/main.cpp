#include <iostream>
#include <SFML/Graphics.hpp>

#include "ecs_simulator.h"
#include "simulator_constants.h"
#include "renderer.h"

// Helper function to get scenario list with names
static std::vector<std::pair<SimulatorConstants::SimulationType,std::string>> buildScenarioList() {
    std::vector<std::pair<SimulatorConstants::SimulationType,std::string>> scenarios;
    for (auto s : SimulatorConstants::getAllScenarios()) {
        scenarios.push_back({s, SimulatorConstants::getScenarioName(s)});
    }
    return scenarios;
}

int main(int, char**) {
    // Start with CELESTIAL_GAS scenario
    auto initialScenario = SimulatorConstants::SimulationType::CELESTIAL_GAS;
    SimulatorConstants::initializeConstants(initialScenario);

    Renderer renderer(SimulatorConstants::ScreenLength + 200, SimulatorConstants::ScreenLength);
    if (!renderer.init()) {
        return 1;
    }

    CoordinateSystem coordSystem;
    ECSSimulator simulator(&coordSystem);
    simulator.setScenario(initialScenario);
    simulator.init();

    const int FPS = 60;
    const int frameDelay = 1000 / FPS; // target milliseconds per frame

    sf::Clock fpsClock;
    sf::Clock frameClock;
    float avgFPS = 0;
    int frameCount = 0;
    float fpsTimer = 0.0f;

    bool running = true;
    bool paused = false;

    auto scenarioList = buildScenarioList();

    // To highlight buttons when hovered or clicked
    SimulatorConstants::SimulationType highlightedScenario = initialScenario;
    bool highlightPausePlay = false;
    bool highlightReset = false;

    while(running) {
        // Handle events
        sf::Event event;
        sf::RenderWindow& window = renderer.getWindow();

        // Mouse position for hovering
        sf::Vector2i mousePos = sf::Mouse::getPosition(window);
        int mouseX = mousePos.x;
        int mouseY = mousePos.y;

        // Reset highlights every frame
        highlightPausePlay = false;
        highlightReset = false;
        highlightedScenario = (SimulatorConstants::SimulationType)-1;

        // Update highlight states based on mouse hover
        // Pause/Play
        if (mouseX >= renderer.pausePlayButton.rect.left && mouseX <= renderer.pausePlayButton.rect.left + renderer.pausePlayButton.rect.width &&
            mouseY >= renderer.pausePlayButton.rect.top && mouseY <= renderer.pausePlayButton.rect.top + renderer.pausePlayButton.rect.height) {
            highlightPausePlay = true;
        }

        // Reset
        if (mouseX >= renderer.resetButton.rect.left && mouseX <= renderer.resetButton.rect.left + renderer.resetButton.rect.width &&
            mouseY >= renderer.resetButton.rect.top && mouseY <= renderer.resetButton.rect.top + renderer.resetButton.rect.height) {
            highlightReset = true;
        }

        // Scenario buttons
        for (auto& btn : renderer.scenarioButtons) {
            if (mouseX >= btn.rect.left && mouseX <= btn.rect.left + btn.rect.width &&
                mouseY >= btn.rect.top && mouseY <= btn.rect.top + btn.rect.height) {
                highlightedScenario = btn.scenario;
                break;
            }
        }

        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                running = false;
            } else if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
                // Check button clicks
                // Pause/Play
                if (mouseX >= renderer.pausePlayButton.rect.left && mouseX <= renderer.pausePlayButton.rect.left + renderer.pausePlayButton.rect.width &&
                    mouseY >= renderer.pausePlayButton.rect.top && mouseY <= renderer.pausePlayButton.rect.top + renderer.pausePlayButton.rect.height) {
                    paused = !paused;
                }

                // Reset
                if (mouseX >= renderer.resetButton.rect.left && mouseX <= renderer.resetButton.rect.left + renderer.resetButton.rect.width &&
                    mouseY >= renderer.resetButton.rect.top && mouseY <= renderer.resetButton.rect.top + renderer.resetButton.rect.height) {
                    simulator.reset();
                    paused = false;
                }

                // Scenario change
                for (auto& btn : renderer.scenarioButtons) {
                    if (mouseX >= btn.rect.left && mouseX <= btn.rect.left + btn.rect.width &&
                        mouseY >= btn.rect.top && mouseY <= btn.rect.top + btn.rect.height) {
                        simulator.setScenario(btn.scenario);
                        simulator.reset();
                        paused = false;
                        break;
                    }
                }
            } else if (event.type == sf::Event::KeyPressed) {
                // Keyboard shortcuts as fallback
                if (event.key.code == sf::Keyboard::P) {
                    paused = !paused;
                } else if (event.key.code == sf::Keyboard::R) {
                    simulator.reset();
                    paused = false;
                } else if (event.key.code == sf::Keyboard::Num1) {
                    simulator.setScenario(SimulatorConstants::SimulationType::CELESTIAL_GAS);
                    simulator.reset();
                    paused = false;
                } else if (event.key.code == sf::Keyboard::Num2) {
                    simulator.setScenario(SimulatorConstants::SimulationType::ISOTHERMAL_BOX);
                    simulator.reset();
                    paused = false;
                } else if (event.key.code == sf::Keyboard::Num3) {
                    simulator.setScenario(SimulatorConstants::SimulationType::BOUNCY_BALLS);
                    simulator.reset();
                    paused = false;
                }
            }
        }

        // Tick simulation if not paused
        if (!paused) {
            simulator.tick();
        }

        renderer.clear();
        renderer.renderParticles(simulator.getRegistry(), Renderer::whiteColor);

        // FPS calculation
        float dt = frameClock.restart().asMilliseconds();
        fpsTimer += dt;
        frameCount++;
        if (fpsTimer >= 1000.0f) {
            avgFPS = (float)frameCount / (fpsTimer / 1000.0f);
            frameCount = 0;
            fpsTimer = 0.0f;
        }

        renderer.renderFPS(avgFPS);

        // Render UI overlay
        renderer.renderUI(paused, simulator.getCurrentScenario(), scenarioList,
                          highlightPausePlay, highlightReset, highlightedScenario);

        renderer.present();

        // Framerate control (not always necessary with SFML)
        if (frameDelay > dt) {
            sf::sleep(sf::milliseconds((int)(frameDelay - dt)));
        }
    }

    return 0;
}