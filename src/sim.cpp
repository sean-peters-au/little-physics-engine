/**
 * @fileoverview simulator.cpp
 * @brief Implementation of ECSSimulator.
 */

#include "sim.hpp"

#include <iostream>
#include <memory>

#include "entities/entity_components.hpp"
#include "entities/sim_components.hpp"
#include "core/profile.hpp"
#include "systems/system_config.hpp"
#include "systems/barnes_hut.hpp"
#include "systems/boundary.hpp"
#include "systems/dampening.hpp"
#include "systems/fluid/fluid.hpp"
#include "systems/gravity.hpp"
#include "systems/movement.hpp"
#include "systems/rigid/collision_data.hpp"
#include "systems/rigid/rigid_body_collision.hpp"
#include "systems/rotation.hpp"
#include "systems/sleep.hpp"

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
  
  systems.push_back(std::make_unique<Systems::FluidSystem>());
  systems.push_back(std::make_unique<Systems::RigidBodyCollisionSystem>());
  systems.push_back(std::make_unique<Systems::BasicGravitySystem>());
  systems.push_back(std::make_unique<Systems::BarnesHutSystem>());
  systems.push_back(std::make_unique<Systems::DampeningSystem>());
  systems.push_back(std::make_unique<Systems::RotationSystem>());
  systems.push_back(std::make_unique<Systems::MovementSystem>());
  systems.push_back(std::make_unique<Systems::SleepSystem>());
  systems.push_back(std::make_unique<Systems::BoundarySystem>());
  
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