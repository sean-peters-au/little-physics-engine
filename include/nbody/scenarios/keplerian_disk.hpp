/**
 * @file keplerian_disk.hpp
 * @brief Declaration of the KeplerianDiskScenario class
 */

#pragma once

#include <entt/entt.hpp>
#include "nbody/scenarios/i_scenario.hpp"

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
