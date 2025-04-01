/**
 * @fileoverview sim_manager.cpp
 * @brief Implementation of SimManager, which coordinates subsystems and user interaction.
 */


#include <iostream>

#include <SFML/Window/Event.hpp>

#include "sim_manager.hpp"
#include "presentation_manager.hpp"
#include "entities/sim_components.hpp"
#include "core/constants.hpp"
#include "core/profile.hpp"

SimManager::SimManager()
    : presentationManager(SimulatorConstants::ScreenLength + 200, SimulatorConstants::ScreenLength, SharedSystemConfig()),
      simulator(),
      scenarioManager(),
      uiManager(),
      running(true),
      paused(false),
      stepFrame(false) {
  uiManager.setSimManager(this);
}

bool SimManager::init() {
  if (!presentationManager.init()) {
    std::cerr << "PresentationManager initialization failed." << std::endl;
    return false;
  }

  scenarioManager.buildScenarioList();
  scenarioManager.setInitialScenario(SimulatorConstants::SimulationType::KEPLERIAN_DISK);

  selectScenario(scenarioManager.getCurrentScenario());

  uiManager.setScenarioList(scenarioManager.getScenarioList());
  return true;
}

bool SimManager::handleEvents() {
  sf::RenderWindow& window = presentationManager.getWindow();

  sf::Event event;
  while (window.pollEvent(event)) {
    if (event.type == sf::Event::Closed) {
      running = false;
    } else if (event.type == sf::Event::KeyPressed) {
      switch (event.key.code) {
        case sf::Keyboard::Escape:
          running = false;
          break;
        case sf::Keyboard::P:
          togglePause();
          break;
        case sf::Keyboard::Space:
          if (paused) {
            stepFrame = true;
          }
          break;
        case sf::Keyboard::R:
          resetSimulator();
          break;
        case sf::Keyboard::D:
          presentationManager.toggleDebugVisualization();
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
        case sf::Keyboard::Num5:
          selectScenario(SimulatorConstants::SimulationType::HOURGLASSES);
          break;
        case sf::Keyboard::Num6:
          selectScenario(SimulatorConstants::SimulationType::PLANETARY_OCEAN);
          break;
        case sf::Keyboard::Num7:
          selectScenario(SimulatorConstants::SimulationType::GALTON_BOARD);
          break;
        default:
          break;
      }
    } else if (event.type == sf::Event::MouseButtonPressed &&
               event.mouseButton.button == sf::Mouse::Left) {
      uiManager.handleClick(event.mouseButton.x, event.mouseButton.y, paused);
    }
  }

  uiManager.updateHighlights(window);
  return running;
}

void SimManager::tick() {
  if (!paused || stepFrame) {
    simulator.tick();
    stepFrame = false;
  }
}

void SimManager::render(float fps) {
  presentationManager.clear();

  presentationManager.renderSolidParticles(simulator.getRegistry());
  presentationManager.renderFluidParticles(simulator.getRegistry());
  presentationManager.renderGasParticles(simulator.getRegistry());

  presentationManager.renderFPS(fps);

  uiManager.renderUI(presentationManager, simulator.getRegistry(), paused, scenarioManager.getCurrentScenario());

  presentationManager.present();
}

void SimManager::togglePause() {
  paused = !paused;
}

void SimManager::resetSimulator() {
  simulator.reset();
  paused = false;
}

void SimManager::stepOnce() {
  stepFrame = true;
}

void SimManager::setTimeScale(double multiplier) {
  auto& registry = simulator.getRegistry();
  auto view = registry.view<Components::SimulatorState>();
  if (!view.empty()) {
    auto& st = registry.get<Components::SimulatorState>(view.front());
    st.timeScale = multiplier;
  }
}

void SimManager::setColorScheme(PresentationManager::ColorScheme scheme) {
  presentationManager.setColorScheme(scheme);
}

void SimManager::selectScenario(SimulatorConstants::SimulationType scenario) {
  scenarioManager.setInitialScenario(scenario);

  auto scenarioPtr = scenarioManager.createScenario(scenario);
  ScenarioSystemConfig config = scenarioPtr->getSystemsConfig();

  simulator.applyConfig(config);
  presentationManager.updateCoordinates(config.sharedConfig);
  simulator.loadScenario(std::move(scenarioPtr));
  simulator.reset();

  paused = false;
}