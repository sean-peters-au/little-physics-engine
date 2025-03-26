/**
 * @file i_scenario.hpp
 * @brief Declaration of the IScenario interface
 */

#pragma once

#include <entt/entt.hpp>
#include "systems/shared_system_config.hpp"
#include "systems/dampening.hpp"
#include "systems/boundary.hpp"
#include "systems/rotation.hpp"
#include "systems/movement.hpp"
#include "systems/gravity.hpp"
#include "systems/barnes_hut.hpp"
#include "systems/fluid/fluid.hpp"
#include "systems/rigid/rigid_body_collision.hpp"
#include "systems/sleep.hpp"

/**
 * @struct ScenarioSystemConfig
 * @brief Complete configuration for a scenario, including shared and system-specific parameters
 */
struct ScenarioSystemConfig {
    // Shared parameters used by all systems
    SharedSystemConfig sharedConfig;
    
    // System-specific configurations with sensible defaults
    Systems::DampeningConfig dampeningConfig;
    Systems::BoundaryConfig boundaryConfig;
    Systems::RotationConfig rotationConfig;
    Systems::MovementConfig movementConfig;
    Systems::GravityConfig gravityConfig;
    Systems::BarnesHutConfig barnesHutConfig;
    Systems::FluidConfig fluidConfig;
    Systems::RigidBodyCollisionConfig rigidBodyConfig;
    Systems::SleepConfig sleepConfig;
    
    // Additional scenario-specific parameters could be added here
};

/**
 * @brief Abstract base class for any simulation scenario
 *
 * Each scenario must provide:
 *  - getConfig() returning ScenarioSystemConfig
 *  - createEntities() that spawns all ECS entities
 */
class IScenario {
public:
    virtual ~IScenario() = default;

    /**
     * @brief Returns scenario configuration (universe scale, particle count, etc.)
     */
    virtual ScenarioSystemConfig getSystemsConfig() const = 0;

    /**
     * @brief Creates scenario-specific entities in the registry
     */
    virtual void createEntities(entt::registry &registry) const = 0;
};
