#include "nbody/core/constants.hpp"
#include <cmath>
#include <iostream>
#include <vector>

namespace SimulatorConstants {

    // Global constants
    const double Pi    = 3.141592654;
    const double RealG = 6.674e-11;  // m³/kg/s²
    const double Epsilon = 1e-9;

    // Scenario variables
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

    // Display
    const unsigned int ScreenLength   = 600;
    const unsigned int StepsPerSecond = 120;
    const unsigned int Threads        = 1;

    // ECS systems for the scenario
    std::vector<Systems::SystemType> ActiveSystems;

    void initializeConstants(SimulationType /*type*/) {
        // Basic default init
        UniverseSizeMeters         = 1.0;
        TimeAcceleration           = 1.0;
        MetersPerPixel             = 1.0;
        SecondsPerTick             = 1.0;
        GravitationalSoftener      = 0.0;
        CollisionCoeffRestitution  = 0.0;
        DragCoeff                  = 0.0;
        ParticleDensity            = 0.0;

        GridSize                   = 50;
        CellSizePixels             = 0.0;

        ParticleCount              = 100;
        ParticleMassMean           = 1.0;
        ParticleMassStdDev         = 0.1;
        InitialVelocityFactor      = 1.0;

        ActiveSystems.clear();
    }

    // Utility
    double pixelsToMeters(double pixels) {
        return pixels * MetersPerPixel;
    }

    double metersToPixels(double meters) {
        return meters / MetersPerPixel;
    }

    double simulationToRealTime(double ticks) {
        return ticks * SecondsPerTick / TimeAcceleration;
    }

    double realToSimulationTime(double seconds) {
        return seconds * TimeAcceleration / SecondsPerTick;
    }

    // Scenario listing
    std::vector<SimulationType> getAllScenarios() {
        return {
            SimulationType::KEPLERIAN_DISK,
            SimulationType::ISOTHERMAL_BOX,
            SimulationType::RANDOM_POLYGONS,
            SimulationType::SIMPLE_FLUID,
            SimulationType::FLUID_AND_POLYGONS
        };
    }

    std::string getScenarioName(SimulationType scenario) {
        switch (scenario) {
            case SimulationType::KEPLERIAN_DISK:   return "KEPLERIAN_DISK";
            case SimulationType::ISOTHERMAL_BOX:   return "ISOTHERMAL_BOX";
            case SimulationType::RANDOM_POLYGONS:  return "RANDOM_POLYGONS";
            case SimulationType::SIMPLE_FLUID:     return "SIMPLE_FLUID";
            case SimulationType::FLUID_AND_POLYGONS: return "FLUID_AND_POLYGONS";
            default:                               return "UNKNOWN";
        }
    }

} // namespace SimulatorConstants