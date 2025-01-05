/**
 * @file random_polygons_scenario.cpp
 * @brief Implementation of a scenario with static boundary walls and random polygons/circles.
 *
 *        Changes:
 *        - Thinner walls (0.1 m).
 *        - Added a distinct color for walls.
 *        - Emphasized that walls remain "asleep" with infinite mass.
 */

#include "nbody/scenarios/random_polygons.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/math/polygon.hpp"
#include <random>
#include <cmath>
#include <ctime>
#include <iostream>

static constexpr double kCirclesFraction     = 0.15;
static constexpr double kRegularFraction     = 0.5;
static constexpr double kRandomPolyFraction  = 0.35;

static constexpr double kSmallShapeRatio = 0.90;
static constexpr double kSmallShapeMin = 0.1;  // Smaller minimum
static constexpr double kSmallShapeMax = 0.25;  // Smaller shapes range
static constexpr double kLargeShapeMin = 0.3;   // Larger shapes range
static constexpr double kLargeShapeMax = 0.5;  // Larger maximum

static constexpr int    kParticleCount = 10;

// Helper: build a regular polygon
PolygonShape buildRegularPolygon(int sides, double sz)
{
    PolygonShape poly;
    poly.type = Components::ShapeType::Polygon;
    double angleStep = 2.0 * M_PI / double(sides);
    for(int i = 0; i < sides; i++) {
        double angle = i * angleStep;
        double x = sz * std::cos(angle);
        double y = sz * std::sin(angle);
        poly.vertices.push_back(Vector(x, y));
    }
    return poly;
}

// Helper: build a random polygon
PolygonShape buildRandomPolygon(std::default_random_engine &gen, double sz)
{
    PolygonShape poly;
    poly.type = Components::ShapeType::Polygon;

    std::uniform_int_distribution<int> sideDist(3, 7);
    std::uniform_real_distribution<double> radiusDist(0.5 * sz, sz);

    int sides = sideDist(gen);
    double angleStep = 2.0 * M_PI / double(sides);

    double angle = 0.0;
    for(int i = 0; i < sides; i++) {
        double r = radiusDist(gen);
        double x = r * std::cos(angle);
        double y = r * std::sin(angle);
        poly.vertices.push_back(Vector(x, y));
        angle += angleStep;
    }
    return poly;
}

ScenarioConfig RandomPolygonsScenario::getConfig() const
{
    ScenarioConfig cfg;
    cfg.MetersPerPixel = 1e-2;
    cfg.UniverseSizeMeters = SimulatorConstants::ScreenLength * cfg.MetersPerPixel;
    cfg.SecondsPerTick = 1.0 / SimulatorConstants::StepsPerSecond;
    cfg.TimeAcceleration = 1.0;
    cfg.GridSize = 50;
    cfg.CellSizePixels = (double)SimulatorConstants::ScreenLength / cfg.GridSize;

    cfg.ParticleCount = kParticleCount;
    cfg.ParticleMassMean = 1.0;
    cfg.ParticleMassStdDev = 0.1;
    cfg.GravitationalSoftener = 0;
    cfg.CollisionCoeffRestitution = 0.0;
    cfg.DragCoeff = 0.0;
    cfg.ParticleDensity = 0.5;
    cfg.InitialVelocityFactor = 1.0;

    cfg.activeSystems = {
        Systems::SystemType::BASIC_GRAVITY,
        Systems::SystemType::COLLISION,
        Systems::SystemType::DAMPENING,
        Systems::SystemType::SLEEP,
        Systems::SystemType::ROTATION,
        Systems::SystemType::MOVEMENT,
    };

    return cfg;
}

/**
 * @brief Create a static "wall" entity with infinite mass, minimal thickness,
 *        set asleep, and given a distinct color. 
 */
static void makeWall(entt::registry &registry,
                     double cx, double cy,
                     double halfW, double halfH)
{
    auto wallEnt = registry.create();

    // Infinite mass => 1e30
    double mass = 1e30;

    registry.emplace<Components::Position>(wallEnt, cx, cy);
    registry.emplace<Components::Velocity>(wallEnt, 0.0, 0.0);
    registry.emplace<Components::Mass>(wallEnt, mass);

    // Mark it as a boundary + solid so systems skip it
    registry.emplace<Components::ParticlePhase>(wallEnt, Components::Phase::Solid);
    registry.emplace<Components::Boundary>(wallEnt);

    // Force it to be asleep
    auto &sleepC = registry.emplace<Components::Sleep>(wallEnt);
    sleepC.asleep = true;
    sleepC.sleepCounter = 9999999; // large

    // A friction or material if needed
    registry.emplace<Components::Material>(wallEnt, 0.5, 0.4);

    // Use a rectangle polygon
    PolygonShape poly;
    poly.type = Components::ShapeType::Polygon;
    poly.vertices.push_back(Vector(-halfW, -halfH));
    poly.vertices.push_back(Vector( halfW, -halfH));
    poly.vertices.push_back(Vector( halfW,  halfH));
    poly.vertices.push_back(Vector(-halfW,  halfH));

    registry.emplace<Components::Shape>(wallEnt, Components::ShapeType::Polygon, halfH);
    registry.emplace<PolygonShape>(wallEnt, poly);

    // Angular data (irrelevant for a static wall, but required)
    registry.emplace<Components::AngularPosition>(wallEnt, 0.0);
    registry.emplace<Components::AngularVelocity>(wallEnt, 0.0);
    registry.emplace<Components::Inertia>(wallEnt, 0.0);

    // Give the walls a distinct color (e.g. dark gray)
    registry.emplace<Components::Color>(wallEnt, 60, 60, 60);
}

void RandomPolygonsScenario::createEntities(entt::registry &registry) const
{
    std::cerr << "Creating Random Polygons scenario...\n";
    std::default_random_engine generator{static_cast<unsigned int>(time(0))};

    double universeSizeM = SimulatorConstants::UniverseSizeMeters;

    // Thin walls: 0.1m thick
    double wallThickness = 0.1;
    double halfWall = wallThickness * 0.5;

    // Left wall
    makeWall(registry, 0.0, universeSizeM*0.5, halfWall, universeSizeM*0.5);
    // Right wall
    makeWall(registry, universeSizeM, universeSizeM*0.5, halfWall, universeSizeM*0.5);
    // Top wall
    makeWall(registry, universeSizeM*0.5, 0.0, universeSizeM*0.5, halfWall);
    // Bottom wall
    makeWall(registry, universeSizeM*0.5, universeSizeM, universeSizeM*0.5, halfWall);

    // Now create circles/polygons as before
    int totalParticles = SimulatorConstants::ParticleCount;
    int circlesCount   = (int)std::round(kCirclesFraction * totalParticles);
    int regularCount   = (int)std::round(kRegularFraction * totalParticles);
    int randomCount    = totalParticles - circlesCount - regularCount;

    int side = (int)std::sqrt((double)totalParticles);
    double spacing = universeSizeM / (side + 1);

    std::normal_distribution<> velocityDist(0.0, 1.0);
    std::normal_distribution<> massDist(SimulatorConstants::ParticleMassMean,
                                        SimulatorConstants::ParticleMassStdDev);
    std::uniform_real_distribution<double> smallSizeDist(kSmallShapeMin, kSmallShapeMax);
    std::uniform_real_distribution<double> largeSizeDist(kLargeShapeMin, kLargeShapeMax);
    std::uniform_real_distribution<double> sizeSelector(0.0, 1.0); // To choose between small and large
    std::uniform_int_distribution<> colorDist(100, 255);

    int created = 0;
    int cCount = 0, rCount = 0, randCount = 0;

    for (int i = 0; i < side && created < totalParticles; ++i) {
        for (int j = 0; j < side && created < totalParticles; ++j) {
            double x_m = (i + 1) * spacing;
            double y_m = (j + 1) * spacing;

            auto entity = registry.create();
            double mass_val = massDist(generator);

            registry.emplace<Components::Position>(entity, x_m, y_m);
            registry.emplace<Components::Velocity>(entity,
                velocityDist(generator), velocityDist(generator));
            registry.emplace<Components::Mass>(entity, mass_val);
            registry.emplace<Components::ParticlePhase>(entity, Components::Phase::Solid);
            registry.emplace<Components::Sleep>(entity);
            // Lower friction
            registry.emplace<Components::Material>(entity, 0.5, 0.3);

            double sz;
            if (sizeSelector(generator) < kSmallShapeRatio) {
                sz = smallSizeDist(generator);
            } else {
                sz = largeSizeDist(generator);
            }

            double I; 

            if (cCount < circlesCount) {
                // Circle
                registry.emplace<Components::Shape>(entity, Components::ShapeType::Circle, sz);
                registry.emplace<CircleShape>(entity, sz);
                I = 0.5 * mass_val * (sz * sz);
                cCount++;
            }
            else if (rCount < regularCount) {
                // Regular polygon
                std::uniform_int_distribution<int> sidesDist(3, 6);
                int sides = sidesDist(generator);

                PolygonShape poly = buildRegularPolygon(sides, sz);
                registry.emplace<Components::Shape>(entity, Components::ShapeType::Polygon, sz);
                registry.emplace<PolygonShape>(entity, poly);

                I = 0.5 * mass_val * (sz * sz);
                rCount++;
            }
            else {
                // Random polygon
                PolygonShape poly = buildRandomPolygon(generator, sz);
                registry.emplace<Components::Shape>(entity, Components::ShapeType::Polygon, sz);
                registry.emplace<PolygonShape>(entity, poly);

                I = 0.5 * mass_val * (sz * sz);
                randCount++;
            }

            registry.emplace<Components::AngularPosition>(entity, 0.0);
            registry.emplace<Components::AngularVelocity>(entity, 0.0);
            registry.emplace<Components::Inertia>(entity, I);

            // Random color
            int rr = colorDist(generator);
            int gg = colorDist(generator);
            int bb = colorDist(generator);
            registry.emplace<Components::Color>(entity, rr, gg, bb);

            created++;
        }
    }

    std::cerr << "...Created " << created << " random polygons total.\n"
              << "   Circles:  " << cCount    << "\n"
              << "   Regular:  " << rCount    << "\n"
              << "   Random:   " << randCount << "\n";
}