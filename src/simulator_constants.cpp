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
        // Declare all variables at the start
        double seconds_per_frame;
        double inner_orbit_radius;
        double central_mass;
        double orbital_period;
        double days_per_second;
        
        switch (type) {
            case SimulationType::CELESTIAL_GAS:
                // Space scale: 1 pixel = 10,000 km = 1e7 meters
                MetersPerPixel = 1e7;
                
                // Mass scale: Central mass = 1e25 kg (about 1/100th of Jupiter)
                central_mass = 1e25;
                
                // Calculate orbital period at inner radius (100 pixels)
                inner_orbit_radius = 100 * MetersPerPixel;  // 1e9 meters
                orbital_period = 2 * Pi * std::sqrt(std::pow(inner_orbit_radius, 3) / 
                                                  (RealG * central_mass));  // ≈ 24,300 seconds
                
                // We want inner orbit (100 pixels) to move at 2.1 pixels/tick
                // Circumference = 2π * 100 ≈ 628 pixels
                // Need to cover this in 300 ticks (5 seconds)
                SecondsPerTick = 1.0 / static_cast<double>(StepsPerSecond);  // 1/60th second real time
                
                // Reduce time acceleration to 1/10th of previous value
                TimeAcceleration = orbital_period / (5.0 * StepsPerSecond) * 20.0;  // 20x instead of 200x
                
                // Grid parameters
                GridSize = 100;
                CellSizePixels = static_cast<double>(ScreenLength) / GridSize;
                
                // Particle parameters
                ParticleCount = 3000;
                ParticleMassMean = 1e22;  // About 1/1000th of central mass
                ParticleMassStdDev = 1e21;
                
                // Physical parameters
                // Increase softening length to 20,000 km (2 pixels)
                GravitationalSoftener = 2e7;  
                CollisionCoeffRestitution = 0.5;
                DragCoeff = 1e-7;  
                ParticleDensity = 0.1;
                InitialVelocityFactor = 1.0;
                
                // Calculate debug values
                seconds_per_frame = SecondsPerTick * TimeAcceleration;
                days_per_second = TimeAcceleration / (24.0 * 3600.0);
                
                // Debug output
                std::cout << "Simulation constants initialized:\n"
                         << "  Space scale: 1 pixel = " << (MetersPerPixel/1000.0) << " km\n"
                         << "  Screen size: " << (ScreenLength * MetersPerPixel / 1e9) << " million km\n"
                         << "  Central mass: " << central_mass << " kg (1/100th Jupiter)\n"
                         << "  Inner orbit (100 px) period: " << (orbital_period/3600.0) << " hours\n"
                         << "  Time scale: 1 real second = " << days_per_second << " days\n"
                         << "  Each frame advances: " << (seconds_per_frame / 3600.0) << " hours\n"
                         << "  Expected inner orbit speed: " << (2 * Pi * 100 / (5.0 * StepsPerSecond)) << " pixels/tick\n"
                         << "  Particle count: " << ParticleCount << "\n"
                         << "  Particle mass: " << ParticleMassMean << " kg\n"
                         << "  Orbital period at screen edge: " 
                         << (2 * Pi * std::sqrt(std::pow(ScreenLength * MetersPerPixel / 2, 3) / 
                            (RealG * central_mass)) / (24.0 * 3600.0)) 
                         << " days\n"
                         << "  Softening length: " << (GravitationalSoftener/1000.0) << " km\n";
                break;

            default:
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
