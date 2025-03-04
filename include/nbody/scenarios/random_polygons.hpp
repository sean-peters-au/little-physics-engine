/**
 * @file random_polygons.hpp
 * @brief Declaration of the RandomPolygonsScenario class
 */

#pragma once

#include <entt/entt.hpp>
#include "nbody/scenarios/i_scenario.hpp"

/**
 * @struct RandomPolygonsConfig
 * @brief Configuration parameters specific to the random polygons scenario
 */
struct RandomPolygonsConfig {
    // Shape type distribution
    double circlesFraction = 0.1;       // Fraction of shapes that are circles
    double regularFraction = 0.6;       // Fraction of shapes that are regular polygons
    // Random polygon fraction is 1.0 - circlesFraction - regularFraction
    
    // Size parameters
    double smallShapeRatio = 0.90;      // Ratio of small to large shapes
    double smallShapeMin = 0.1;         // Minimum size for small shapes
    double smallShapeMax = 0.25;        // Maximum size for small shapes
    double largeShapeMin = 0.3;         // Minimum size for large shapes
    double largeShapeMax = 0.5;         // Maximum size for large shapes
    
    // Friction parameters
    double floorStaticFriction = 0.6;   // Static friction for floor
    double floorDynamicFriction = 0.4;  // Dynamic friction for floor
    double wallStaticFriction = 0.2;    // Static friction for walls
    double wallDynamicFriction = 0.1;   // Dynamic friction for walls
    double particleStaticFriction = 0.3; // Static friction for particles
    double particleDynamicFriction = 0.1; // Dynamic friction for particles
    
    // Particle parameters
    int particleCount = 100;            // Number of particles to create
    double particleMassMean = 1.0;      // Mean mass of particles
    double particleMassStdDev = 0.1;    // Standard deviation of particle mass
    double initialVelocityFactor = 1.0; // Initial velocity scaling
    
    // Wall parameters
    double wallThickness = 0.1;         // Thickness of boundary walls
};

/**
 * @class RandomPolygonsScenario
 *
 * Implements a scenario with "bouncy balls" or random polygon shapes.
 */
class RandomPolygonsScenario : public IScenario {
public:
    RandomPolygonsScenario() = default;
    ~RandomPolygonsScenario() override = default;

    SystemConfig getConfig() const override;
    void createEntities(entt::registry &registry) const override;
    
private:
    RandomPolygonsConfig scenarioConfig;
};

