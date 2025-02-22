/**
 * @file scenario_config.hpp
 * @brief Configuration for a simulation scenario.
 */

#pragma once

#include <vector>
#include "nbody/systems/systems.hpp"

/**
 * @brief Container for simulation constants and active ECS systems.
 *
 * This structure holds key simulation parameters including universe scaling, timing,
 * physical properties (gravity, collision, drag, density), spatial grid sizing, and particle properties.
 * It also specifies which ECS systems are active for the scenario.
 */
struct ScenarioConfig {
    double UniverseSizeMeters;
    double TimeAcceleration;
    double MetersPerPixel;
    double SecondsPerTick;
    double GravitationalSoftener;
    double CollisionCoeffRestitution;
    double DragCoeff;
    double ParticleDensity;

    int GridSize;
    double CellSizePixels;

    int ParticleCount;
    double ParticleMassMean;
    double ParticleMassStdDev;
    double InitialVelocityFactor;

    // Active ECS systems for this scenario.
    std::vector<Systems::SystemType> activeSystems;
};

/**
 * @brief Applies simulation configuration to global constants.
 *
 * Updates the global simulation constants based on the provided ScenarioConfig.
 * This function is typically called during the configuration phase of a scenario,
 * before any entities are created.
 *
 * @param cfg The ScenarioConfig containing simulation parameters and active ECS systems.
 */
void applyScenarioConfig(const ScenarioConfig &cfg);
