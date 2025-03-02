#pragma once

#include <vector>
#include "nbody/systems/systems.hpp"

/**
 * @struct SystemConfig
 * @brief Holds all system configuration parameters for the simulation.
 */
struct SystemConfig {
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

    std::vector<Systems::SystemType> activeSystems;
};