#ifndef NBODY_ISOTHERMAL_BOX_SCENARIO_HPP
#define NBODY_ISOTHERMAL_BOX_SCENARIO_HPP

#include "nbody/core/i_scenario.hpp"
#include <entt/entt.hpp>

/**
 * @class IsothermalBoxScenario
 *
 * Implements a uniform box of gas with no initial velocity.
 */
class IsothermalBoxScenario : public IScenario {
public:
    IsothermalBoxScenario() = default;
    ~IsothermalBoxScenario() override = default;

    ScenarioConfig getConfig() const override;
    void createEntities(entt::registry &registry) const override;
};

#endif // NBODY_ISOTHERMAL_BOX_SCENARIO_HPP