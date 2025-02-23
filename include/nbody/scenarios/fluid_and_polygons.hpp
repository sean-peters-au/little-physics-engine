#pragma once

#include "nbody/core/constants.hpp"
#include "nbody/scenarios/scenario_config.hpp"
#include "nbody/scenarios/i_scenario.hpp"
#include "entt/entt.hpp"

/**
 * @brief Test scenario that combines a tank of fluid particles with
 *        polygon obstacles.
 */
class FluidAndPolygonsScenario : public IScenario {
public:
    /**
     * @brief Retrieves the configuration for the test fluid-polygons scenario.
     *
     * @return ScenarioConfig with simulation parameters.
     */
    ScenarioConfig getConfig() const;

    /**
     * @brief Creates all entities for the scenario, including boundary walls,
     *        fluid particles, and polygon obstacles.
     *
     * @param registry The entt registry into which the entities will be inserted.
     */
    void createEntities(entt::registry &registry) const;
};
