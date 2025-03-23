/**
 * @file fluid_and_polygons.cpp
 * @brief A scenario placing fluid particles near the top and random polygons below,
 *        all falling under gravity in a bounded box.
 */

#include <random>
#include <cmath>
#include <ctime>
#include <iostream>

#include "scenarios/fluid_and_polygons.hpp"
#include "core/constants.hpp"
#include "entities/entity_components.hpp"
#include "math/polygon.hpp"

// Helper to create a static boundary wall with infinite mass
static void makeWall(entt::registry &registry,
                     double cx, double cy,
                     double halfW, double halfH,
                     double staticFriction, double dynamicFriction,
                     double wallMass)
{
    auto wallEnt = registry.create();
    registry.emplace<Components::Position>(wallEnt, cx, cy);
    registry.emplace<Components::Velocity>(wallEnt, 0.0, 0.0);
    registry.emplace<Components::Mass>(wallEnt, wallMass);

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

SystemConfig FluidAndPolygonsScenario::getConfig() const
{
    SystemConfig cfg;
    cfg.MetersPerPixel         = 1e-2;
    cfg.UniverseSizeMeters     = SimulatorConstants::ScreenLength * cfg.MetersPerPixel;
    cfg.SecondsPerTick         = 1.0 / SimulatorConstants::StepsPerSecond;
    cfg.TimeAcceleration       = 1.0;
    cfg.GridSize               = 50;
    cfg.CellSizePixels         = static_cast<double>(SimulatorConstants::ScreenLength) / cfg.GridSize;

    cfg.GravitationalSoftener  = 0.0;
    cfg.CollisionCoeffRestitution = 0.2; // slight bounce for rigid shapes
    cfg.DragCoeff              = 0.0;
    cfg.ParticleDensity        = 100.0;

    return cfg;
}

void FluidAndPolygonsScenario::createEntities(entt::registry &registry) const
{
    SystemConfig systemConfig = getConfig();

    // 1) Create bounding walls
    double sizeM = systemConfig.UniverseSizeMeters;
    double halfWall = scenarioConfig.wallThickness * 0.5;

    // Bottom wall
    makeWall(registry,
        sizeM * 0.5,         // center x
        sizeM,               // center y at top of the coordinate system (since y=0 is top in your setup)
        sizeM * 0.5,         // half width
        halfWall,            // half height
        scenarioConfig.floorStaticFriction,
        scenarioConfig.floorDynamicFriction,
        scenarioConfig.wallMass);

    // Top wall
    makeWall(registry,
        sizeM * 0.5,
        0.0,
        sizeM * 0.5,
        halfWall,
        scenarioConfig.wallStaticFriction,
        scenarioConfig.wallDynamicFriction,
        scenarioConfig.wallMass);

    // Left wall
    makeWall(registry,
        0.0,
        sizeM * 0.5,
        halfWall,
        sizeM * 0.5,
        scenarioConfig.wallStaticFriction,
        scenarioConfig.wallDynamicFriction,
        scenarioConfig.wallMass);

    // Right wall
    makeWall(registry,
        sizeM,
        sizeM * 0.5,
        halfWall,
        sizeM * 0.5,
        scenarioConfig.wallStaticFriction,
        scenarioConfig.wallDynamicFriction,
        scenarioConfig.wallMass);

    // 2) Create random polygon entities on the RIGHT side of the screen
    std::default_random_engine generator(static_cast<unsigned>(std::time(nullptr)));
    std::uniform_real_distribution<double> xDist(sizeM * 0.6, sizeM * 0.8); // RIGHT side
    std::uniform_real_distribution<double> yDist(sizeM * 0.3, sizeM * 0.6); // Middle-upper area
    std::normal_distribution<double> massDist(scenarioConfig.polygonMassMean, scenarioConfig.polygonMassStdDev);
    std::uniform_int_distribution<int> colorDist(50, 200);
    std::normal_distribution<> velocityDist(0.0, scenarioConfig.initialVelocityScale);

    for (int i = 0; i < scenarioConfig.polygonCount; ++i) {
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
        registry.emplace<Components::Material>(ent, scenarioConfig.polyStaticFriction, scenarioConfig.polyDynamicFriction);
        registry.emplace<Components::Sleep>(ent);

        // Build a random polygon
        double sizePoly = 0.15 + 0.1 * 1; // (i % 3); // vary sizes a bit
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
        int numFluid = scenarioConfig.fluidParticleCount;
        double x_min = sizeM * 0.1;
        double x_max = sizeM * 0.4; // LEFT side of screen
        double y_min = sizeM * 0.2; // Higher up
        double y_max = sizeM * 0.6; // More vertical space

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
                registry.emplace<Components::Mass>(e, scenarioConfig.fluidParticleMass);
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

    std::cerr << "Created " << scenarioConfig.polygonCount << " random polygons.\n";
    std::cerr << "Fluid + Polygons scenario ready.\n";
}