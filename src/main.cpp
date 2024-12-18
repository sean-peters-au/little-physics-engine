#include <iostream>
#include <SFML/Graphics.hpp>

#include "nbody/core/simulator.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/rendering/renderer.hpp"
#include "nbody/components/sim.hpp"

// Helper function to get scenario list with names
static std::vector<std::pair<SimulatorConstants::SimulationType,std::string>> buildScenarioList() {
    std::vector<std::pair<SimulatorConstants::SimulationType,std::string>> scenarios;
    for (auto s : SimulatorConstants::getAllScenarios()) {
        scenarios.push_back({s, SimulatorConstants::getScenarioName(s)});
    }
    return scenarios;
}

// Helper function to update simulator state when changing scenarios
void updateSimulatorState(ECSSimulator& simulator, SimulatorConstants::SimulationType scenario) {
    SimulatorConstants::initializeConstants(scenario);
    simulator.setScenario(scenario);
    
    // Update base time acceleration for new scenario
    auto& registry = simulator.getRegistry();
    auto stateView = registry.view<Components::SimulatorState>();
    if (!stateView.empty()) {
        auto& state = registry.get<Components::SimulatorState>(stateView.front());
        state.baseTimeAcceleration = SimulatorConstants::TimeAcceleration;
    }
    
    simulator.reset();
}

int main(int, char**) {
    // Start with CELESTIAL_GAS scenario
    auto initialScenario = SimulatorConstants::SimulationType::CELESTIAL_GAS;

    Renderer renderer(SimulatorConstants::ScreenLength + 200, SimulatorConstants::ScreenLength);
    if (!renderer.init()) {
        return 1;
    }

    CoordinateSystem coordSystem;
    ECSSimulator simulator(&coordSystem);
    
    // Create simulator state entity
    auto stateEntity = simulator.getRegistry().create();
    
    updateSimulatorState(simulator, initialScenario);  // Use helper instead of direct calls
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
                        updateSimulatorState(simulator, btn.scenario);  // Use helper here too
                        paused = false;
                        break;
                    }
                }

                // Check speed buttons
                for (const auto& btn : renderer.speedButtons) {
                    if (btn.rect.contains(mousePos)) {
                        auto stateView = simulator.getRegistry().view<Components::SimulatorState>();
                        if (!stateView.empty()) {
                            auto& state = simulator.getRegistry().get<Components::SimulatorState>(stateView.front());
                            state.timeScale = btn.speedMultiplier;
                        }
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
                    updateSimulatorState(simulator, SimulatorConstants::SimulationType::CELESTIAL_GAS);
                    paused = false;
                } else if (event.key.code == sf::Keyboard::Num2) {
                    updateSimulatorState(simulator, SimulatorConstants::SimulationType::ISOTHERMAL_BOX);
                    paused = false;
                } else if (event.key.code == sf::Keyboard::Num3) {
                    updateSimulatorState(simulator, SimulatorConstants::SimulationType::BOUNCY_BALLS);
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
        renderer.renderUI(simulator.getRegistry(), 
                        paused, 
                        simulator.getCurrentScenario(), 
                        scenarioList,
                        highlightPausePlay, 
                        highlightReset, 
                        highlightedScenario);

        renderer.present();

        // Framerate control (not always necessary with SFML)
        if (frameDelay > dt) {
            sf::sleep(sf::milliseconds((int)(frameDelay - dt)));
        }
    }

    return 0;
}