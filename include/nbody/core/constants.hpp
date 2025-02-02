#ifndef SIMULATOR_CONSTANTS_H
#define SIMULATOR_CONSTANTS_H

#include <vector>
#include <string>
#include "nbody/systems/systems.hpp"  // <-- includes Systems::SystemType

namespace SimulatorConstants {

    /**
     * @brief The high-level simulation scenario types. 
     * 
     * If you want more (e.g. SOLAR_SYSTEM, MOLECULAR, etc.), add them here.
     */
    enum class SimulationType {
        KEPLERIAN_DISK,
        ISOTHERMAL_BOX,
        RANDOM_POLYGONS,
        SIMPLE_FLUID
    };

    // Truly global constants
    extern const double Pi;
    extern const double RealG;  // Real gravitational constant 6.674e-11
    extern const double Epsilon;

    // Scenario-specific
    extern double UniverseSizeMeters;
    extern double TimeAcceleration;
    extern double MetersPerPixel;
    extern double SecondsPerTick;
    extern double GravitationalSoftener;
    extern double CollisionCoeffRestitution;
    extern double DragCoeff;
    extern double ParticleDensity;

    extern int GridSize;
    extern double CellSizePixels;

    extern int ParticleCount;
    extern double ParticleMassMean;
    extern double ParticleMassStdDev;
    extern double InitialVelocityFactor;

    // Display constants
    extern const unsigned int ScreenLength;
    extern const unsigned int StepsPerSecond;
    extern const unsigned int Threads;

    // Active ECS systems for the current scenario (populated by scenario config)
    extern std::vector<Systems::SystemType> ActiveSystems;

    /**
     * @brief Reset or initialize scenario-dependent variables to safe defaults.
     * @param type The scenario type (not used directly if you rely on scenario config).
     */
    void initializeConstants(SimulationType type);

    // Utility conversions
    double pixelsToMeters(double pixels);
    double metersToPixels(double meters);
    double simulationToRealTime(double ticks);
    double realToSimulationTime(double seconds);

    // If you want a function listing all possible scenarios:
    std::vector<SimulationType> getAllScenarios();
    std::string getScenarioName(SimulationType scenario);
}

#endif // SIMULATOR_CONSTANTS_H