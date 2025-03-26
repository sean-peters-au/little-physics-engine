#pragma once

#include <vector>
#include "systems/systems.hpp"

/**
 * @struct SharedSystemConfig
 * @brief Holds all system configuration parameters for the simulation.
 */
struct SharedSystemConfig {
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
};