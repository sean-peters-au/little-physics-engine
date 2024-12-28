#ifndef NBODY_SCENARIO_CONFIG_HPP
#define NBODY_SCENARIO_CONFIG_HPP

#include <vector>
#include "nbody/systems/systems.hpp"

/**
 * @struct ScenarioConfig
 * @brief Holds scenario-defined constants & ECS system selection.
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

    // The ECS systems to run in this scenario
    std::vector<Systems::SystemType> activeSystems;
};

/**
 * @brief Applies a ScenarioConfig to the global SimulatorConstants variables.
 */
void applyScenarioConfig(const ScenarioConfig &cfg);

#endif // NBODY_SCENARIO_CONFIG_HPP