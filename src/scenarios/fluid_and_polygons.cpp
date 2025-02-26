/**
 * @file fluid_and_polygons.cpp
 * @brief A scenario placing fluid particles near the top and random polygons below,
 *        all falling under gravity in a bounded box.
 */

#include <random>
#include <cmath>
#include <ctime>
#include <iostream>

#include "nbody/scenarios/fluid_and_polygons.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sph.hpp"
#include "nbody/math/polygon.hpp"

///////////////////////////////////////////////////////////////////////////////
// Some constants for this combined scenario
///////////////////////////////////////////////////////////////////////////////
static constexpr int    KFluidParticleCount      = 1000;   // number of fluid particles
static constexpr double KFluidParticleMass       = 0.005;  // mass per fluid particle
static constexpr double KFluidRestDensity        = 1000.0; // typical rest density for water

static constexpr int    KPolygonCount            = 5;     // how many random polygons to spawn
static constexpr double KPolygonMassMean         = 0.5;   // typical mass for polygons
static constexpr double KPolygonMassStdDev       = 0.2;   // variation in polygon mass

// Polygons: friction matches random_polygons.cpp
static constexpr double KFloorStaticFriction     = 0.6;
static constexpr double KFloorDynamicFriction    = 0.4;
static constexpr double KWallStaticFriction      = 0.2;
static constexpr double KWallDynamicFriction     = 0.1;
static constexpr double KPolyStaticFriction      = 0.3;
static constexpr double KPolyDynamicFriction     = 0.1;

// Wall thickness
static constexpr double KWallThickness           = 0.1;   // thickness of bounding walls
static constexpr double KWallMass                = 1e30;  // effectively infinite mass

// Add near the top where other constants are defined
static constexpr double KInitialVelocityScale = 0.5;

// Helper to create a static boundary wall with infinite mass
static void makeWall(entt::registry &registry,
                     double cx, double cy,
                     double halfW, double halfH,
                     double staticFriction, double dynamicFriction)
{
    auto wallEnt = registry.create();
    registry.emplace<Components::Position>(wallEnt, cx, cy);
    registry.emplace<Components::Velocity>(wallEnt, 0.0, 0.0);
    registry.emplace<Components::Mass>(wallEnt, KWallMass);

    // Mark as boundary + solid
    registry.emplace<Components::Boundary>(wallEnt);
    registry.emplace<Components::ParticlePhase>(wallEnt, Components::Phase::Solid);

    // Force asleep so it doesn't move
    auto &sleepC = registry.emplace<Components::Sleep>(wallEnt);
    sleepC.asleep = true;
    sleepC.sleepCounter = 9999999;

    // Material friction
    registry.emplace<Components::Material>(wallEnt, staticFriction, dynamicFriction);

    // Build a rectangle polygon shape
    PolygonShape poly;
    poly.type = Components::ShapeType::Polygon;
    poly.vertices.emplace_back(-halfW, -halfH);
    poly.vertices.emplace_back(-halfW,  halfH);
    poly.vertices.emplace_back( halfW,  halfH);
    poly.vertices.emplace_back( halfW, -halfH);

    registry.emplace<Components::Shape>(wallEnt, Components::ShapeType::Polygon, halfH);
    registry.emplace<PolygonShape>(wallEnt, poly);
    registry.emplace<Components::AngularPosition>(wallEnt, 0.0);
}

ScenarioConfig FluidAndPolygonsScenario::getConfig() const
{
    ScenarioConfig cfg;
    cfg.MetersPerPixel         = 1e-2;
    cfg.UniverseSizeMeters     = SimulatorConstants::ScreenLength * cfg.MetersPerPixel;
    cfg.SecondsPerTick         = 1.0 / SimulatorConstants::StepsPerSecond;
    cfg.TimeAcceleration       = 1.0;
    cfg.GridSize               = 50;
    cfg.CellSizePixels         = static_cast<double>(SimulatorConstants::ScreenLength) / cfg.GridSize;

    // We'll store total count as fluid + some polygons. 
    // The Polygons themselves might not be included in "cfg.ParticleCount" 
    // if you prefer only the fluid in that count, but we set it to the fluid amount for clarity.
    cfg.ParticleCount          = KFluidParticleCount;
    cfg.ParticleMassMean       = KFluidParticleMass;
    cfg.ParticleMassStdDev     = 0.1;    // fluid uniform mass
    cfg.GravitationalSoftener  = 0.0;
    cfg.CollisionCoeffRestitution = 0.2; // slight bounce for rigid shapes
    cfg.DragCoeff              = 0.0;
    cfg.ParticleDensity        = 0.5;
    cfg.InitialVelocityFactor  = 1.0;  // Change from 0.0 to match random_polygons.cpp

    // Use the same systems as random_polygons.cpp:
    cfg.activeSystems = {
        Systems::SystemType::FLUID,
        Systems::SystemType::BASIC_GRAVITY,
        Systems::SystemType::COLLISION,
        Systems::SystemType::DAMPENING,
        Systems::SystemType::SLEEP,
        Systems::SystemType::ROTATION,
        Systems::SystemType::MOVEMENT,
        Systems::SystemType::BOUNDARY,
        Systems::SystemType::RIGID_FLUID,
    };

    return cfg;
}

void FluidAndPolygonsScenario::createEntities(entt::registry &registry) const
{
    // 1) Create bounding walls
    double sizeM = SimulatorConstants::UniverseSizeMeters;
    double halfWall = KWallThickness * 0.5;

    // Bottom wall
    makeWall(registry,
        sizeM * 0.5,         // center x
        sizeM,               // center y at top of the coordinate system (since y=0 is top in your setup)
        sizeM * 0.5,         // half width
        halfWall,            // half height
        KFloorStaticFriction,
        KFloorDynamicFriction);

    // Top wall
    makeWall(registry,
        sizeM * 0.5,
        0.0,
        sizeM * 0.5,
        halfWall,
        KWallStaticFriction,
        KWallDynamicFriction);

    // Left wall
    makeWall(registry,
        0.0,
        sizeM * 0.5,
        halfWall,
        sizeM * 0.5,
        KWallStaticFriction,
        KWallDynamicFriction);

    // Right wall
    makeWall(registry,
        sizeM,
        sizeM * 0.5,
        halfWall,
        sizeM * 0.5,
        KWallStaticFriction,
        KWallDynamicFriction);

    // 2) Create random polygon entities on the RIGHT side of the screen
    std::default_random_engine generator(static_cast<unsigned>(std::time(nullptr)));
    std::uniform_real_distribution<double> xDist(sizeM * 0.6, sizeM * 0.8); // RIGHT side
    std::uniform_real_distribution<double> yDist(sizeM * 0.3, sizeM * 0.6); // Middle-upper area
    std::normal_distribution<double> massDist(KPolygonMassMean, KPolygonMassStdDev);
    std::uniform_int_distribution<int> colorDist(50, 200);
    std::normal_distribution<> velocityDist(0.0, KInitialVelocityScale);

    for (int i = 0; i < KPolygonCount; ++i) {
        // Random position on the RIGHT side
        double x = xDist(generator);
        double y = yDist(generator);

        auto ent = registry.create();

        // Basic properties
        double massVal = std::max(0.1, massDist(generator)); // ensure not zero or negative
        registry.emplace<Components::Position>(ent, x, y);
        registry.emplace<Components::Velocity>(ent, 
            velocityDist(generator), 
            velocityDist(generator));
        registry.emplace<Components::Mass>(ent, massVal);
        registry.emplace<Components::ParticlePhase>(ent, Components::Phase::Solid);
        registry.emplace<Components::Material>(ent, KPolyStaticFriction, KPolyDynamicFriction);
        registry.emplace<Components::Sleep>(ent);

        // Build a random polygon
        double sizePoly = 0.15 + 0.1 * (i % 3); // vary sizes a bit
        // PolygonShape poly = buildRandomConvexPolygon(generator, sizePoly);
        PolygonShape poly = buildRegularPolygon(5, sizePoly);
        registry.emplace<Components::Shape>(ent, Components::ShapeType::Polygon, sizePoly);
        registry.emplace<PolygonShape>(ent, poly);

        // Calculate moment of inertia
        double I = calculatePolygonInertia(poly.vertices, massVal);
        registry.emplace<Components::Inertia>(ent, I);

        // Angular components
        registry.emplace<Components::AngularPosition>(ent, 0.0);
        registry.emplace<Components::AngularVelocity>(ent, 0.0);

        // Random color
        int rr = colorDist(generator);
        int gg = colorDist(generator);
        int bb = colorDist(generator);
        registry.emplace<Components::Color>(ent, rr, gg, bb);
    }

    // 3) Create fluid particles on the LEFT side of the screen
    {
        int numFluid = KFluidParticleCount;
        double x_min = sizeM * 0.2;
        double x_max = sizeM * 0.4; // LEFT side of screen
        double y_min = sizeM * 0.2; // Higher up
        double y_max = sizeM * 0.5; // More vertical space

        double regionWidth  = x_max - x_min;
        double regionHeight = y_max - y_min;

        int nCols = static_cast<int>(std::sqrt(numFluid));
        int nRows = (numFluid + nCols - 1) / nCols;  // ceiling division

        double dx = regionWidth / (nCols + 1);
        double dy = regionHeight / (nRows + 1);

        std::uniform_real_distribution<double> jitterDist(-0.1, 0.1);

        int count = 0;
        for (int row = 0; row < nRows && count < numFluid; row++) {
            for (int col = 0; col < nCols && count < numFluid; col++) {
                double jitterX = jitterDist(generator) * dx;
                double jitterY = jitterDist(generator) * dy;
                double x = x_min + (col + 1) * dx + jitterX;
                double y = y_min + (row + 1) * dy + jitterY;

                auto e = registry.create();
                registry.emplace<Components::Position>(e, x, y);
                registry.emplace<Components::Velocity>(e, 0.0, 0.0);
                registry.emplace<Components::Mass>(e, KFluidParticleMass);
                registry.emplace<Components::ParticlePhase>(e, Components::Phase::Liquid);

                // No friction for fluid particles
                registry.emplace<Components::Material>(e, 0.0, 0.0);

                // Use a circle shape
                double r = 0.02;
                registry.emplace<Components::Shape>(e, Components::ShapeType::Circle, r);
                registry.emplace<CircleShape>(e, r);

                // SPH-related properties
                registry.emplace<Components::SmoothingLength>(e, 0.06);
                registry.emplace<Components::SpeedOfSound>(e, 1000.0);
                registry.emplace<Components::SPHTemp>(e);

                // Slightly varying blue color
                registry.emplace<Components::Color>(e, 20, 20 + (count % 50), 200 + (count % 50));

                count++;
            }
        }
        std::cerr << "Created " << numFluid << " fluid particles at the top.\n";
    }

    std::cerr << "Created " << KPolygonCount << " random polygons.\n";
    std::cerr << "Fluid + Polygons scenario ready.\n";
}
