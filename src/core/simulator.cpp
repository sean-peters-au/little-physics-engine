#include "nbody/core/simulator.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/core/scenario_config.hpp"
#include "nbody/core/i_scenario.hpp"
#include "nbody/core/profile.hpp"

#include "nbody/systems/sleep.hpp"
#include "nbody/systems/rotation.hpp"
#include "nbody/systems/movement.hpp"
#include "nbody/systems/boundary.hpp"
#include "nbody/systems/dampening.hpp"
#include "nbody/systems/barnes_hut.hpp"
#include "nbody/systems/thermodynamics.hpp"
#include "nbody/systems/gravity.hpp"
#include "nbody/systems/rigid_body_collision/rigid_body_collision.hpp"
#include "nbody/systems/rigid_body_collision/collision_data.hpp"

#include "nbody/components/basic.hpp"
#include <nbody/math/polygon.hpp>
#include <random>
#include <cmath>
#include <iostream>
#include <memory>

// Our scenario classes
#include "nbody/scenarios/keplerian_disk.hpp"
#include "nbody/scenarios/random_polygons.hpp"
#include <nbody/scenarios/simple_fluid.hpp>
#include <nbody/systems/fluid/fluid.hpp>

ECSSimulator::ECSSimulator() {
    // e.g. set a default scenarioPtr if you like,
    // or just do nothing
}

void ECSSimulator::setScenario(SimulatorConstants::SimulationType scenario) {
    // Store the scenario type for use in reset()
    currentScenario = scenario;
    std::cerr << "Scenario set to: " << static_cast<int>(scenario) << std::endl;
}

void ECSSimulator::reset() {
    // Save existing simulator state
    Components::SimulatorState savedState;
    savedState.timeScale = 1.0;
    savedState.baseTimeAcceleration = SimulatorConstants::TimeAcceleration;

    auto stateView = registry.view<Components::SimulatorState>();
    if (!stateView.empty()) {
        savedState = registry.get<Components::SimulatorState>(stateView.front());
    }

    // Clear ECS
    registry.clear();

    // Use the stored scenario instead of hardcoding RANDOM_POLYGONS
    SimulatorConstants::SimulationType const chosen = currentScenario;

    // Reset global constants to default
    SimulatorConstants::initializeConstants(chosen);

    // Create scenario object
    switch (chosen) {
        case SimulatorConstants::SimulationType::KEPLERIAN_DISK:
            scenarioPtr = std::make_unique<KeplerianDiskScenario>();
            break;
        case SimulatorConstants::SimulationType::SIMPLE_FLUID:
            scenarioPtr = std::make_unique<SimpleFluidScenario>();
            break;
        case SimulatorConstants::SimulationType::RANDOM_POLYGONS:
        default:
            scenarioPtr = std::make_unique<RandomPolygonsScenario>();
            break;
    }

    // Grab config from scenario
    ScenarioConfig const cfg = scenarioPtr->getConfig();
    // Apply config to SimulatorConstants
    applyScenarioConfig(cfg);

    // Recreate a fresh SimulatorState entity
    auto stateEntity = registry.create();
    registry.emplace<Components::SimulatorState>(stateEntity, savedState);

    // Let the scenario create all ECS entities
    scenarioPtr->createEntities(registry);

    // Now do any post-scenario init you want
    init();
}

void ECSSimulator::init() {
    std::cout << "ECSSimulator::init" << std::endl;
    if (!fluidSystem) {  // Only construct if not already created.
        fluidSystem = std::make_unique<Systems::FluidSystem>();
    }
    
    // (Any other initialization code)
}

void ECSSimulator::tick() {
    PROFILE_SCOPE("Tick");
    // Run the scenario's chosen ECS systems in order
    for (auto system : SimulatorConstants::ActiveSystems) {
        switch (system) {
        case Systems::SystemType::FLUID: {
            fluidSystem->update(registry);
            break;
        }
        case Systems::SystemType::COLLISION: {
            Systems::RigidBodyCollisionSystem::update(registry);
            break;
        }
        case Systems::SystemType::BASIC_GRAVITY: {
            Systems::BasicGravitySystem::update(registry);
            break;
        }
        case Systems::SystemType::ROTATION: {
            Systems::RotationSystem::update(registry);
            break;
        }
        case Systems::SystemType::BARNES_HUT: {
            Systems::BarnesHutSystem::update(registry);
            break;
        }
        case Systems::SystemType::GRID_THERMODYNAMICS: {
            Systems::GridThermodynamicsSystem::update(registry);
            break;
        }
        case Systems::SystemType::MOVEMENT: {
            Systems::MovementSystem::update(registry);
            break;
        }
        case Systems::SystemType::SLEEP: {
            Systems::SleepSystem::update(registry);
            break;
        }
        case Systems::SystemType::DAMPENING: {
            Systems::DampeningSystem::update(registry);
            break;
        }
        case Systems::SystemType::BOUNDARY: {
            Systems::BoundarySystem::update(registry);
            break;
        }
        } // switch
    } // for
}