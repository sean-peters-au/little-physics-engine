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
#include "scenarios/i_scenario.hpp"
#include "systems/shared_system_config.hpp"
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

void ECSSimulator::applyConfig(const ScenarioSystemConfig& cfg) {
  // Store the current config
  currentConfig = cfg;
  
  // Apply configs to all existing systems
  for (auto& system : systems) {
    // Apply shared config to all systems
    system->setSharedSystemConfig(currentConfig.sharedConfig);
    
    // Apply specific configs by type
    if (auto* dampeningSystem = dynamic_cast<Systems::DampeningSystem*>(system.get())) {
      dampeningSystem->setSpecificConfig(currentConfig.dampeningConfig);
    }
    else if (auto* boundarySystem = dynamic_cast<Systems::BoundarySystem*>(system.get())) {
      boundarySystem->setSpecificConfig(currentConfig.boundaryConfig);
    }
    else if (auto* rotationSystem = dynamic_cast<Systems::RotationSystem*>(system.get())) {
      rotationSystem->setSpecificConfig(currentConfig.rotationConfig);
    }
    else if (auto* movementSystem = dynamic_cast<Systems::MovementSystem*>(system.get())) {
      movementSystem->setSpecificConfig(currentConfig.movementConfig);
    }
    else if (auto* gravitySystem = dynamic_cast<Systems::BasicGravitySystem*>(system.get())) {
      gravitySystem->setSpecificConfig(currentConfig.gravityConfig);
    }
    else if (auto* barnesHutSystem = dynamic_cast<Systems::BarnesHutSystem*>(system.get())) {
      barnesHutSystem->setSpecificConfig(currentConfig.barnesHutConfig);
    }
    else if (auto* fluidSystem = dynamic_cast<Systems::FluidSystem*>(system.get())) {
      fluidSystem->setSpecificConfig(currentConfig.fluidConfig);
    }
    else if (auto* rigidBodySystem = dynamic_cast<Systems::RigidBodyCollisionSystem*>(system.get())) {
      rigidBodySystem->setSpecificConfig(currentConfig.rigidBodyConfig);
    }
    else if (auto* sleepSystem = dynamic_cast<Systems::SleepSystem*>(system.get())) {
      sleepSystem->setSpecificConfig(currentConfig.sleepConfig);
    }
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
  // systems.push_back(std::make_unique<Systems::DampeningSystem>());
  systems.push_back(std::make_unique<Systems::RotationSystem>());
  systems.push_back(std::make_unique<Systems::MovementSystem>());
  systems.push_back(std::make_unique<Systems::SleepSystem>());
  systems.push_back(std::make_unique<Systems::BoundarySystem>());
  
  // Configure all systems with both shared and specific configs
  for (auto& system : systems) {
    // Apply shared config
    system->setSharedSystemConfig(currentConfig.sharedConfig);
    
    // Apply specific configs by type
    if (auto* dampeningSystem = dynamic_cast<Systems::DampeningSystem*>(system.get())) {
      dampeningSystem->setSpecificConfig(currentConfig.dampeningConfig);
    }
    else if (auto* boundarySystem = dynamic_cast<Systems::BoundarySystem*>(system.get())) {
      boundarySystem->setSpecificConfig(currentConfig.boundaryConfig);
    }
    else if (auto* rotationSystem = dynamic_cast<Systems::RotationSystem*>(system.get())) {
      rotationSystem->setSpecificConfig(currentConfig.rotationConfig);
    }
    else if (auto* movementSystem = dynamic_cast<Systems::MovementSystem*>(system.get())) {
      movementSystem->setSpecificConfig(currentConfig.movementConfig);
    }
    else if (auto* gravitySystem = dynamic_cast<Systems::BasicGravitySystem*>(system.get())) {
      gravitySystem->setSpecificConfig(currentConfig.gravityConfig);
    }
    else if (auto* barnesHutSystem = dynamic_cast<Systems::BarnesHutSystem*>(system.get())) {
      barnesHutSystem->setSpecificConfig(currentConfig.barnesHutConfig);
    }
    else if (auto* fluidSystem = dynamic_cast<Systems::FluidSystem*>(system.get())) {
      fluidSystem->setSpecificConfig(currentConfig.fluidConfig);
    }
    else if (auto* rigidBodySystem = dynamic_cast<Systems::RigidBodyCollisionSystem*>(system.get())) {
      rigidBodySystem->setSpecificConfig(currentConfig.rigidBodyConfig);
    }
    else if (auto* sleepSystem = dynamic_cast<Systems::SleepSystem*>(system.get())) {
      sleepSystem->setSpecificConfig(currentConfig.sleepConfig);
    }
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