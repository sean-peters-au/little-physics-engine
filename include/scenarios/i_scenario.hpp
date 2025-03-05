/**
 * @file i_scenario.hpp
 * @brief Declaration of the IScenario interface
 */

#pragma once

#include <entt/entt.hpp>
#include "systems/system_config.hpp"

/**
 * @brief Abstract base class for any simulation scenario
 *
 * Each scenario must provide:
 *  - getConfig() returning ScenarioConfig
 *  - createEntities() that spawns all ECS entities
 */
class IScenario {
public:
    virtual ~IScenario() = default;

    /**
     * @brief Returns scenario configuration (universe scale, particle count, etc.)
     */
    virtual SystemConfig getConfig() const = 0;

    /**
     * @brief Creates scenario-specific entities in the registry
     */
    virtual void createEntities(entt::registry &registry) const = 0;
};
