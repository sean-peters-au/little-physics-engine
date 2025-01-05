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
#include "nbody/systems/sph.hpp"
#include "nbody/systems/gravity.hpp"
#include "nbody/systems/rigid_body_collision.hpp"
#include "nbody/systems/collision/collision_data.hpp"

#include "nbody/components/basic.hpp"
#include <nbody/math/polygon.hpp>
#include <random>
#include <cmath>
#include <iostream>
#include <memory>

// Our scenario classes
#include "nbody/scenarios/keplerian_disk.hpp"
#include "nbody/scenarios/isothermal_box.hpp"
#include "nbody/scenarios/random_polygons.hpp"

ECSSimulator::ECSSimulator() {
    // e.g. set a default scenarioPtr if you like,
    // or just do nothing
}

ECSSimulator::ECSSimulator(CoordinateSystem* /*coordSystem*/) {
    // Possibly store the coordSystem if you need it
}

void ECSSimulator::setScenario(SimulatorConstants::SimulationType scenario) {
    // We'll store the scenario type in a unique_ptr. Let's create it in reset()
    // so this function might just remember it, or do nothing:
    std::cerr << "Scenario set to: " << (int)scenario << std::endl;

    // For now, do nothing. We actually do the real scenario switching in reset().
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

    // Suppose we have a local "scenario" variable in setScenario(), or we just pick one here:
    // For demonstration, let's say we always do RANDOM_POLYGONS:
    // In practice, store your scenario choice in a member variable or pass it in
    SimulatorConstants::SimulationType chosen = SimulatorConstants::SimulationType::RANDOM_POLYGONS;

    // Reset global constants to default
    SimulatorConstants::initializeConstants(chosen);

    // Create scenario object
    switch (chosen) {
        case SimulatorConstants::SimulationType::KEPLERIAN_DISK:
            scenarioPtr = std::make_unique<KeplerianDiskScenario>();
            break;
        case SimulatorConstants::SimulationType::ISOTHERMAL_BOX:
            scenarioPtr = std::make_unique<IsothermalBoxScenario>();
            break;
        case SimulatorConstants::SimulationType::RANDOM_POLYGONS:
        default:
            scenarioPtr = std::make_unique<RandomPolygonsScenario>();
            break;
    }

    // Grab config from scenario
    ScenarioConfig cfg = scenarioPtr->getConfig();
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
    // Extra steps if needed. Right now, do nothing.
}

void ECSSimulator::tick() {
    // Run the scenario's chosen ECS systems in order
    for (auto system : SimulatorConstants::ActiveSystems) {
        switch (system) {
        case Systems::SystemType::COLLISION: {
            PROFILE_SCOPE("Collision");
            Systems::RigidBodyCollisionSystem::update(registry);
            break;
        }
        case Systems::SystemType::ROTATION: {
            PROFILE_SCOPE("Rotation");
            Systems::RotationSystem::update(registry);
            break;
        }
        case Systems::SystemType::BASIC_GRAVITY: {
            PROFILE_SCOPE("Basic Gravity");
            Systems::BasicGravitySystem::update(registry);
            break;
        }
        case Systems::SystemType::BARNES_HUT: {
            PROFILE_SCOPE("Barnes Hut");
            Systems::BarnesHutSystem::update(registry);
            break;
        }
        case Systems::SystemType::SPH: {
            PROFILE_SCOPE("SPH");
            Systems::SPHSystem::update(registry);
            break;
        }
        case Systems::SystemType::GRID_THERMODYNAMICS: {
            PROFILE_SCOPE("Grid Thermodynamics");
            Systems::GridThermodynamicsSystem::update(registry);
            break;
        }
        case Systems::SystemType::MOVEMENT: {
            PROFILE_SCOPE("Movement");
            Systems::MovementSystem::update(registry);
            break;
        }
        case Systems::SystemType::SLEEP: {
            PROFILE_SCOPE("Sleep");
            Systems::SleepSystem::update(registry);
            break;
        }
        case Systems::SystemType::DAMPENING: {
            PROFILE_SCOPE("Dampening");
            Systems::DampeningSystem::update(registry);
            break;
        }
        case Systems::SystemType::BOUNDARY: {
            PROFILE_SCOPE("Boundary");
            Systems::BoundarySystem::update(registry);
            break;
        }
        } // switch
    } // for
}