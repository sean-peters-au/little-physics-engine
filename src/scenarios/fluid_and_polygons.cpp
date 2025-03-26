/**
 * @file fluid_and_polygons.cpp
 * @brief A scenario placing fluid particles at the bottom and random polygons at the top,
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

ScenarioSystemConfig FluidAndPolygonsScenario::getSystemsConfig() const
{
    ScenarioSystemConfig config;
    
    // Configure shared parameters
    config.sharedConfig.MetersPerPixel = 1e-2;
    config.sharedConfig.UniverseSizeMeters = SimulatorConstants::ScreenLength * config.sharedConfig.MetersPerPixel;
    config.sharedConfig.SecondsPerTick = 1.0 / SimulatorConstants::StepsPerSecond;
    config.sharedConfig.TimeAcceleration = 1.0;
    config.sharedConfig.GridSize = 50;
    config.sharedConfig.CellSizePixels = static_cast<double>(SimulatorConstants::ScreenLength) / config.sharedConfig.GridSize;

    config.sharedConfig.GravitationalSoftener = 0.0;
    config.sharedConfig.CollisionCoeffRestitution = 0.2; // slight bounce for rigid shapes
    config.sharedConfig.DragCoeff = 0.0;
    config.sharedConfig.ParticleDensity = 100.0;

    return config;
}

void FluidAndPolygonsScenario::createEntities(entt::registry &registry) const
{
    ScenarioSystemConfig scenarioSystemConfig = getSystemsConfig();
    SharedSystemConfig sharedConfig = scenarioSystemConfig.sharedConfig;

    // 1) Create bounding walls
    double sizeM = sharedConfig.UniverseSizeMeters;
    double halfWall = scenarioEntityConfig.wallThickness * 0.5;

    // Bottom wall
    makeWall(registry,
        sizeM * 0.5,         // center x
        sizeM,               // center y at top of the coordinate system (since y=0 is top in your setup)
        sizeM * 0.5,         // half width
        halfWall,            // half height
        scenarioEntityConfig.floorStaticFriction,
        scenarioEntityConfig.floorDynamicFriction,
        scenarioEntityConfig.wallMass);

    // Top wall
    makeWall(registry,
        sizeM * 0.5,
        0.0,
        sizeM * 0.5,
        halfWall,
        scenarioEntityConfig.wallStaticFriction,
        scenarioEntityConfig.wallDynamicFriction,
        scenarioEntityConfig.wallMass);

    // Left wall
    makeWall(registry,
        0.0,
        sizeM * 0.5,
        halfWall,
        sizeM * 0.5,
        scenarioEntityConfig.wallStaticFriction,
        scenarioEntityConfig.wallDynamicFriction,
        scenarioEntityConfig.wallMass);

    // Right wall
    makeWall(registry,
        sizeM,
        sizeM * 0.5,
        halfWall,
        sizeM * 0.5,
        scenarioEntityConfig.wallStaticFriction,
        scenarioEntityConfig.wallDynamicFriction,
        scenarioEntityConfig.wallMass);

    // 2) Create random polygon entities at the TOP, centered horizontally
    std::default_random_engine generator(static_cast<unsigned>(std::time(nullptr)));
    std::uniform_real_distribution<double> xDist(sizeM * 0.3, sizeM * 0.7); // Centered horizontally
    std::uniform_real_distribution<double> yDist(sizeM * 0.05, sizeM * 0.2); // Near the top
    std::normal_distribution<double> massDist(scenarioEntityConfig.polygonMassMean, scenarioEntityConfig.polygonMassStdDev);
    std::uniform_int_distribution<int> colorDist(50, 200);
    std::normal_distribution<> velocityDist(0.0, scenarioEntityConfig.initialVelocityScale);

    for (int i = 0; i < scenarioEntityConfig.polygonCount; ++i) {
        // Random position at the top, centered
        double x = xDist(generator);
        double y = yDist(generator);

        auto ent = registry.create();

        // Basic properties
        double massVal = std::max(0.1, massDist(generator)); // ensure not zero or negative
        registry.emplace<Components::Position>(ent, x, y);
        registry.emplace<Components::Velocity>(ent, 
            velocityDist(generator) * 0.2, // small horizontal velocity
            std::abs(velocityDist(generator))); // positive (downward) vertical velocity
        registry.emplace<Components::Mass>(ent, massVal);
        registry.emplace<Components::ParticlePhase>(ent, Components::Phase::Solid);
        registry.emplace<Components::Material>(ent, scenarioEntityConfig.polyStaticFriction, scenarioEntityConfig.polyDynamicFriction);
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

    // 3) Create fluid particles at the BOTTOM of the screen
    {
        int numFluid = scenarioEntityConfig.fluidParticleCount;
        double x_min = sizeM * 0.05;
        double x_max = sizeM * 0.95;
        double y_min = sizeM * 0.75;
        double y_max = sizeM * 0.95;

        double regionWidth  = x_max - x_min;
        double regionHeight = y_max - y_min;

        // Calculate a better distribution based on the container's aspect ratio
        double aspectRatio = regionWidth / regionHeight;
        // Determine number of rows and columns based on aspect ratio
        // For N particles in an area with aspect ratio A, we want:
        // rows * cols = N and cols/rows = A
        // This gives us: rows = sqrt(N/A) and cols = sqrt(N*A)
        int nRows = static_cast<int>(std::sqrt(numFluid / aspectRatio));
        if (nRows < 1) nRows = 1;
        // Recalculate columns based on total particles needed
        int nCols = (numFluid + nRows - 1) / nRows; // Ceiling division

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
                registry.emplace<Components::Mass>(e, scenarioEntityConfig.fluidParticleMass);
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
        std::cerr << "Created " << numFluid << " fluid particles at the bottom.\n";
    }

    std::cerr << "Created " << scenarioEntityConfig.polygonCount << " random polygons at the top.\n";
    std::cerr << "Fluid + Polygons scenario ready.\n";
}