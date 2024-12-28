#ifndef NBODY_KEPLERIAN_DISK_SCENARIO_HPP
#define NBODY_KEPLERIAN_DISK_SCENARIO_HPP

#include "nbody/core/i_scenario.hpp"
#include <entt/entt.hpp>

/**
 * @class KeplerianDiskScenario
 * 
 * Creates a central body plus an orbiting disk of gas.
 */
class KeplerianDiskScenario : public IScenario {
public:
    KeplerianDiskScenario() = default;
    ~KeplerianDiskScenario() override = default;

    ScenarioConfig getConfig() const override;
    void createEntities(entt::registry &registry) const override;

private:
    void createCentralBody(entt::registry &registry) const;
    void createKeplerianDisk(entt::registry &registry) const;
};

#endif // NBODY_KEPLERIAN_DISK_SCENARIO_HPP