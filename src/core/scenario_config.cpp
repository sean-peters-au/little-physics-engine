#include "nbody/core/scenario_config.hpp"
#include "nbody/core/constants.hpp"

void applyScenarioConfig(const ScenarioConfig &cfg) {
    using namespace SimulatorConstants;

    UniverseSizeMeters        = cfg.UniverseSizeMeters;
    TimeAcceleration          = cfg.TimeAcceleration;
    MetersPerPixel            = cfg.MetersPerPixel;
    SecondsPerTick            = cfg.SecondsPerTick;
    GravitationalSoftener     = cfg.GravitationalSoftener;
    CollisionCoeffRestitution = cfg.CollisionCoeffRestitution;
    DragCoeff                 = cfg.DragCoeff;
    ParticleDensity           = cfg.ParticleDensity;

    GridSize                  = cfg.GridSize;
    CellSizePixels            = cfg.CellSizePixels;

    ParticleCount             = cfg.ParticleCount;
    ParticleMassMean          = cfg.ParticleMassMean;
    ParticleMassStdDev        = cfg.ParticleMassStdDev;
    InitialVelocityFactor     = cfg.InitialVelocityFactor;

    ActiveSystems = cfg.activeSystems;
}