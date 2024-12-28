#ifndef NBODY_RANDOM_POLYGONS_SCENARIO_HPP
#define NBODY_RANDOM_POLYGONS_SCENARIO_HPP

#include "nbody/core/i_scenario.hpp"
#include <entt/entt.hpp>

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

#endif // NBODY_RANDOM_POLYGONS_SCENARIO_HPP