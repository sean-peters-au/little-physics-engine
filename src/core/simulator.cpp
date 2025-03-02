/**
 * @fileoverview simulator.cpp
 * @brief Implementation of ECSSimulator.
 */

#include "nbody/core/simulator.hpp"

#include <iostream>
#include <memory>

#include "nbody/components/basic.hpp"
#include "nbody/components/sim.hpp"
#include "nbody/core/profile.hpp"
#include "nbody/core/system_config.hpp"
#include "nbody/systems/barnes_hut.hpp"
#include "nbody/systems/boundary.hpp"
#include "nbody/systems/dampening.hpp"
#include "nbody/systems/fluid/fluid.hpp"
#include "nbody/systems/gravity.hpp"
#include "nbody/systems/movement.hpp"
#include "nbody/systems/rigid/collision_data.hpp"
#include "nbody/systems/rigid/rigid_body_collision.hpp"
#include "nbody/systems/rotation.hpp"
#include "nbody/systems/sleep.hpp"

ECSSimulator::ECSSimulator() = default;

ECSSimulator::~ECSSimulator() = default;

void ECSSimulator::loadScenario(std::unique_ptr<IScenario> scenario) {
  scenarioPtr = std::move(scenario);
}

void ECSSimulator::applyConfig(const SystemConfig& cfg) {
  currentConfig = cfg;
  
  // Update config for all existing systems
  for (auto& system : systems) {
    system->setSystemConfig(currentConfig);
  }
}

void ECSSimulator::reset() {
  Components::SimulatorState savedState;
  savedState.timeScale = 1.0;
  savedState.baseTimeAcceleration = 1.0;

  auto stateView = registry.view<Components::SimulatorState>();
  if (!stateView.empty()) {
    savedState = registry.get<Components::SimulatorState>(stateView.front());
  }

  registry.clear();

  auto stateEntity = registry.create();
  registry.emplace<Components::SimulatorState>(stateEntity, savedState);

  if (scenarioPtr) {
    scenarioPtr->createEntities(registry);
  }

  init();
}

void ECSSimulator::createSystems() {
  // Clear previous systems
  systems.clear();
  
  // Create new system instances based on active systems in the config
  for (auto systemType : currentConfig.activeSystems) {
    switch (systemType) {
      case Systems::SystemType::FLUID:
        systems.push_back(std::make_unique<Systems::FluidSystem>());
        break;
      case Systems::SystemType::COLLISION:
        systems.push_back(std::make_unique<Systems::RigidBodyCollisionSystem>());
        break;
      case Systems::SystemType::BASIC_GRAVITY:
        systems.push_back(std::make_unique<Systems::BasicGravitySystem>());
        break;
      case Systems::SystemType::ROTATION:
        systems.push_back(std::make_unique<Systems::RotationSystem>());
        break;
      case Systems::SystemType::BARNES_HUT:
        systems.push_back(std::make_unique<Systems::BarnesHutSystem>());
        break;
      case Systems::SystemType::MOVEMENT:
        systems.push_back(std::make_unique<Systems::MovementSystem>());
        break;
      case Systems::SystemType::SLEEP:
        systems.push_back(std::make_unique<Systems::SleepSystem>());
        break;
      case Systems::SystemType::DAMPENING:
        systems.push_back(std::make_unique<Systems::DampeningSystem>());
        break;
      case Systems::SystemType::BOUNDARY:
        systems.push_back(std::make_unique<Systems::BoundarySystem>());
        break;
    }
  }
  
  // Configure all systems
  for (auto& system : systems) {
    system->setSystemConfig(currentConfig);
  }
}

void ECSSimulator::init() {
  std::cout << "ECSSimulator::init()" << std::endl;
  
  // Create all systems according to the current configuration
  createSystems();
}

void ECSSimulator::tick() {
  PROFILE_SCOPE("ECSSimulator::tick");

  // Update all systems in order
  for (auto& system : systems) {
    system->update(registry);
  }
}

entt::registry& ECSSimulator::getRegistry() {
  return registry;
}

const entt::registry& ECSSimulator::getRegistry() const {
  return registry;
}

IScenario& ECSSimulator::getCurrentScenario() const {
  return *scenarioPtr;
}