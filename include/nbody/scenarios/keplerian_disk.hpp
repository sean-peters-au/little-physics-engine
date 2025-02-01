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
    static void createCentralBody(entt::registry &registry) ;
    static void createKeplerianDisk(entt::registry &registry) ;
};

#endif // NBODY_KEPLERIAN_DISK_SCENARIO_HPP