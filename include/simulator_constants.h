#ifndef SIMULATOR_CONSTANTS_H
#define SIMULATOR_CONSTANTS_H

#include <vector>
#include <string>

namespace SimulatorConstants {
    enum class SimulationType {
        CELESTIAL_GAS,    // Gas-to-planet formation
        SOLAR_SYSTEM,     // Planetary orbits (not currently implemented)
        GALAXY,           // Galactic rotation (not currently implemented)
        MOLECULAR,        // Molecular dynamics (not currently implemented)
        ISOTHERMAL_BOX,   // A simple uniform box of gas
        BOUNCY_BALLS      // Bouncy balls in a box
    };

    enum class ECSSystem {
        BASIC_GRAVITY,
        BARNES_HUT,
        COLLISION,
        SPH,
        GRID_THERMODYNAMICS,
        MOVEMENT
    };

    extern const double Pi;
    extern const double RealG;  // Real gravitational constant 6.674e-11

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

    extern const unsigned int ScreenLength;
    extern const unsigned int StepsPerSecond;
    extern const unsigned int Threads;

    extern std::vector<ECSSystem> ActiveSystems;

    void initializeConstants(SimulationType type);

    double pixelsToMeters(double pixels);
    double metersToPixels(double meters);
    double simulationToRealTime(double ticks);
    double realToSimulationTime(double seconds);

    // Helper functions to manage scenarios
    std::vector<SimulationType> getAllScenarios();
    std::string getScenarioName(SimulationType scenario);
}

#endif