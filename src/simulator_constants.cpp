#include "simulator_constants.h"
#include <cmath>
#include <iostream>
#include <vector>

namespace SimulatorConstants {
    const double Pi = 3.141592654;
    const double RealG = 6.674e-11;  // m³/kg/s²

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

    const unsigned int ScreenLength = 600;
    const unsigned int StepsPerSecond = 60;
    const unsigned int Threads = 1;

    std::vector<ECSSystem> ActiveSystems;

    void initializeConstants(SimulationType type) {
        switch (type) {
            case SimulationType::CELESTIAL_GAS: {
                // Same as before
                MetersPerPixel = 1e7;
                UniverseSizeMeters = ScreenLength * MetersPerPixel;
                double central_mass = 1e25;
                double inner_orbit_radius = 100 * MetersPerPixel;  
                double orbital_period = 2 * Pi * std::sqrt(std::pow(inner_orbit_radius, 3) /
                                                  (RealG * central_mass));  
                SecondsPerTick = 1.0 / static_cast<double>(StepsPerSecond);
                TimeAcceleration = orbital_period / (5.0 * StepsPerSecond) * 20.0; 
                GridSize = 100;
                CellSizePixels = static_cast<double>(ScreenLength) / GridSize;
                ParticleCount = 500;
                ParticleMassMean = 1e22;  
                ParticleMassStdDev = 1e21;
                GravitationalSoftener = 2e7;  
                CollisionCoeffRestitution = 0.5;
                DragCoeff = 1e-11;  
                ParticleDensity = 0.1;
                InitialVelocityFactor = 1.0;

                // Configure which systems to use and their order
                ActiveSystems = {
                    ECSSystem::BARNES_HUT,
                    ECSSystem::SPH,
                    ECSSystem::GRID_THERMODYNAMICS,
                    ECSSystem::MOVEMENT
                };

                std::cout << "Simulation constants initialized for CELESTIAL_GAS:\n";
                break;
            }
            case SimulationType::ISOTHERMAL_BOX: {
                // A simple box of gas particles with no gravity or central body
                MetersPerPixel = 1e6; // Smaller scale
                UniverseSizeMeters = ScreenLength * MetersPerPixel;
                // Time scaling: no complex orbital timescales needed
                SecondsPerTick = 1.0 / StepsPerSecond;
                TimeAcceleration = 1.0; 
                GridSize = 50;
                CellSizePixels = static_cast<double>(ScreenLength) / GridSize;

                ParticleCount = 500;
                ParticleMassMean = 1e20;
                ParticleMassStdDev = 1e19;
                GravitationalSoftener = 1e6;
                CollisionCoeffRestitution = 0.5;
                DragCoeff = 0.0;
                ParticleDensity = 0.5;
                InitialVelocityFactor = 0.0; // Start with no initial velocity

                // Configure different systems for isothermal box
                ActiveSystems = {
                    ECSSystem::SPH,
                    ECSSystem::GRID_THERMODYNAMICS,
                    ECSSystem::MOVEMENT
                };

                std::cout << "Simulation constants initialized for ISOTHERMAL_BOX:\n";
                break;
            }
            default:
                // Default to CELESTIAL_GAS if not recognized
                initializeConstants(SimulationType::CELESTIAL_GAS);
                break;
        }
    }

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
}