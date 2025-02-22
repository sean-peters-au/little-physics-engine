/**
 * @file random_polygons.hpp
 * @brief Declaration of the RandomPolygonsScenario class
 */

#pragma once

#include <entt/entt.hpp>
#include "nbody/scenarios/i_scenario.hpp"

/**
 * @class RandomPolygonsScenario
 *
 * Implements a scenario with "bouncy balls" or random polygon shapes.
 */
class RandomPolygonsScenario : public IScenario {
public:
    RandomPolygonsScenario() = default;
    ~RandomPolygonsScenario() override = default;

    ScenarioConfig getConfig() const override;
    void createEntities(entt::registry &registry) const override;
};
