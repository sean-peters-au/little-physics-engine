#include "simulator_constants.h"
#include <cmath>
#include <iostream>

namespace SimulatorConstants {
    // Universal constants
    const double Pi = 3.141592654;
    const double RealG = 6.674e-11;  // m³/kg/s²

    // These will be set by initializeConstants()
    double TimeAcceleration;
    double MetersPerPixel;
    double SecondsPerTick;
    double GravitationalSoftener;
    double CollisionCoeffRestitution;
    double DragCoeff;
    double ParticleDensity;
    
    // Grid parameters
    int GridSize;
    double CellSizePixels;
    
    // Particle parameters
    int ParticleCount;
    double ParticleMassMean;
    double ParticleMassStdDev;
    double InitialVelocityFactor;
    
    // Display settings (constant across simulations)
    const unsigned int ScreenLength = 600;
    const unsigned int StepsPerSecond = 60;
    const unsigned int Threads = 1;

    void initializeConstants(SimulationType type) {
        switch (type) {
            case SimulationType::CELESTIAL_GAS:
                // Space scale: 1 pixel = 500 km = 5e5 meters
                MetersPerPixel = 5e5;
                
                // Time scale: 1 second real time = 1 day simulation time
                TimeAcceleration = 24 * 3600;  // seconds in a day
                SecondsPerTick = 1.0;  // Each tick is 1 second of simulation time
                
                // Grid parameters - high resolution for accurate forces
                GridSize = 500;  // 1000x1000 grid for fine-grained gravity
                CellSizePixels = static_cast<double>(ScreenLength) / GridSize;
                
                // Large number of particles
                ParticleCount = 3000;  // 100k particles
                ParticleMassMean = 1e21;  // Reduced mass for more particles
                ParticleMassStdDev = 1e20;
                
                // Physical parameters
                GravitationalSoftener = 5e5;  // 500 km softening
                CollisionCoeffRestitution = 0.3;
                DragCoeff = 1e-7;
                ParticleDensity = 0.1;
                InitialVelocityFactor = 0.7;

                // Debug output
                std::cout << "Simulation constants initialized:\n"
                         << "  Meters per pixel: " << MetersPerPixel << "\n"
                         << "  Time acceleration: " << TimeAcceleration << " (1 real second = "
                         << TimeAcceleration << " simulation seconds)\n"
                         << "  Seconds per tick: " << SecondsPerTick << "\n"
                         << "  Effective time per tick: " << (SecondsPerTick * TimeAcceleration)
                         << " simulation seconds\n"
                         << "  Grid: " << GridSize << "x" << GridSize 
                         << " (cell size: " << CellSizePixels << " pixels)\n"
                         << "  Particles: " << ParticleCount << "\n"
                         << "  Particle mass: " << ParticleMassMean << " kg\n"
                         << "  Central mass: " << (ParticleMassMean * 100.0) << " kg\n"
                         << "  Total simulation mass: " << (ParticleMassMean * (ParticleCount + 100.0)) << " kg\n";
                break;

            // Add other simulation types here
            default:
                // Default to CELESTIAL_GAS for now
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
