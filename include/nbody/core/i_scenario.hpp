#ifndef NBODY_I_SCENARIO_HPP
#define NBODY_I_SCENARIO_HPP

#include <entt/entt.hpp>
#include "nbody/core/scenario_config.hpp"

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
    virtual ScenarioConfig getConfig() const = 0;

    /**
     * @brief Creates scenario-specific entities in the registry
     */
    virtual void createEntities(entt::registry &registry) const = 0;
};

#endif // NBODY_I_SCENARIO_HPP