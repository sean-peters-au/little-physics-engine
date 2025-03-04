/**
 * @file keplerian_disk.hpp
 * @brief Declaration of the KeplerianDiskScenario class
 */

#pragma once

#include <entt/entt.hpp>
#include "nbody/scenarios/i_scenario.hpp"

/**
 * @struct KeplerianDiskConfig
 * @brief Configuration parameters specific to the Keplerian disk scenario
 */
struct KeplerianDiskConfig {
    // Central body parameters
    double centralMass = 1e36;  // Mass of the central body
    
    // Disk geometry parameters
    double innerRadiusPixels = 100.0;  // Inner radius of the disk in pixels
    double outerRadiusFactor = 2.5;    // Outer radius as fraction of screen size
    
    // Disk height profile parameters
    double heightScaleFactor = 20.0;   // Controls the disk thickness
    double heightPowerLaw = 1.25;      // Power law for disk height vs. radius
    
    // Disk density profile parameters
    double densityPowerLaw = 15.0/8.0; // Power law for density vs. radius
    
    // Particle mass distribution
    double particleMassMean = 1e22;    // Mean mass of particles
    double particleMassStdDev = 1e21;  // Standard deviation of particle mass
    double massRadialPowerLaw = 0.5;   // How particle mass scales with radius
    
    // Orbital parameters
    double orbitalPeriodFraction = 5.0; // Controls time acceleration relative to orbital period
    double velocityDispersionFactor = 0.01; // Random variation in orbital velocities
    double radialVelocityFactor = 0.001;    // Radial velocity component
    
    // Particle count
    int particleCount = 1000;          // Number of particles in the disk
};

/**
 * @class KeplerianDiskScenario
 * 
 * Creates a central body plus an orbiting disk of gas.
 */
class KeplerianDiskScenario : public IScenario {
public:
    KeplerianDiskScenario() = default;
    ~KeplerianDiskScenario() override = default;

    SystemConfig getConfig() const override;
    void createEntities(entt::registry &registry) const override;

private:
    static void createCentralBody(entt::registry &registry, const KeplerianDiskConfig& config);
    static void createKeplerianDisk(entt::registry &registry, const KeplerianDiskConfig& config);
    
    KeplerianDiskConfig diskConfig;
};
