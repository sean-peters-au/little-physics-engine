/**
 * @file sim_manager.cpp
 * @brief Implementation of SimManager, which orchestrates simulation, UI, and scenarios.
 */

#include <iostream>

#include <SFML/Window/Event.hpp>

#include "nbody/core/constants.hpp"
#include "nbody/core/profile.hpp"
#include "nbody/core/sim_manager.hpp"

SimManager::SimManager()
    : renderer(SimulatorConstants::ScreenLength + 200,
               SimulatorConstants::ScreenLength)
    , simulator()
    , scenarioManager()
    , uiManager()
    , running(true)
    , paused(false)
    , stepFrame(false)
{
    // Link the UIManager so it can call back to this SimManager
    uiManager.setSimManager(this);
}

bool SimManager::init()
{
    // Initialize our renderer (creates the SFML window, loads font, etc.)
    if (!renderer.init())
    {
        std::cerr << "Renderer initialization failed." << std::endl;
        return false;
    }

    // Build scenario list and store it in the ScenarioManager
    scenarioManager.buildScenarioList();

    // Choose an initial scenario to start with
    scenarioManager.setInitialScenario(SimulatorConstants::SimulationType::KEPLERIAN_DISK);
    scenarioManager.updateSimulatorState(simulator, scenarioManager.getCurrentScenario());

    // Initialize the simulator
    simulator.init();

    // Let UIManager know about the scenario list for rendering scenario buttons, etc.
    uiManager.setScenarioList(scenarioManager.getScenarioList());

    return true;
}

bool SimManager::handleEvents()
{
    sf::RenderWindow& window = renderer.getWindow();

    sf::Event event;
    while (window.pollEvent(event))
    {
        // A request to close the window or press ESC -> set running=false
        if (event.type == sf::Event::Closed)
        {
            running = false;
        }
        else if (event.type == sf::Event::KeyPressed)
        {
            switch (event.key.code)
            {
                case sf::Keyboard::Escape:
                    running = false;
                    break;
                case sf::Keyboard::P: // Pause
                    togglePause();
                    break;
                case sf::Keyboard::Space: // Advance one frame if paused
                    if (paused)
                    {
                        stepFrame = true;
                    }
                    break;
                case sf::Keyboard::R: // Reset simulator
                    resetSimulator();
                    break;
                case sf::Keyboard::Num1:
                    selectScenario(SimulatorConstants::SimulationType::KEPLERIAN_DISK);
                    break;
                case sf::Keyboard::Num2:
                    selectScenario(SimulatorConstants::SimulationType::RANDOM_POLYGONS);
                    break;
                case sf::Keyboard::Num3:
                    selectScenario(SimulatorConstants::SimulationType::SIMPLE_FLUID);
                    break;
                case sf::Keyboard::Num4:
                    selectScenario(SimulatorConstants::SimulationType::FLUID_AND_POLYGONS);
                    break;
                default:
                    break;
            }
        }
        else if (event.type == sf::Event::MouseButtonPressed &&
                 event.mouseButton.button == sf::Mouse::Left)
        {
            // Delegate UI mouse clicks to UIManager
            uiManager.handleClick(event.mouseButton.x, event.mouseButton.y, paused);
        }
    }

    // Also do "hover"/highlight logic in UIManager
    uiManager.updateHighlights(renderer.getWindow());

    return running;
}

void SimManager::tick()
{
    // Step the simulation if not paused or stepping one frame
    if (!paused || stepFrame)
    {
        simulator.tick();
        stepFrame = false;
    }
}

void SimManager::render(float fps)
{
    renderer.clear();

    // Render the simulation’s particles
    renderer.renderParticles(simulator.getRegistry());

    // Render FPS
    renderer.renderFPS(fps);

    // Render UI
    uiManager.renderUI(renderer, simulator.getRegistry(), paused, scenarioManager.getCurrentScenario());

    // Present final frame
    renderer.present();
}

void SimManager::togglePause()
{
    paused = !paused;
}

void SimManager::resetSimulator()
{
    simulator.reset();
    paused = false;
}

void SimManager::stepOnce()
{
    stepFrame = true;
}

void SimManager::setTimeScale(double multiplier)
{
    auto& registry = simulator.getRegistry();
    auto view = registry.view<Components::SimulatorState>();
    if (!view.empty())
    {
        auto& st = registry.get<Components::SimulatorState>(view.front());
        st.timeScale = multiplier;
    }
}

void SimManager::setColorScheme(Renderer::ColorScheme scheme)
{
    renderer.setColorScheme(scheme);
}

void SimManager::selectScenario(SimulatorConstants::SimulationType scenario)
{
    scenarioManager.updateSimulatorState(simulator, scenario);
    paused = false;
}