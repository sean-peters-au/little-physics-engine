// File: src/scenarios/random_polygons_scenario.cpp
#include "nbody/scenarios/random_polygons.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/math/polygon.hpp"

#include <random>
#include <cmath>
#include <ctime>
#include <iostream>

/**
 * Internal "scenario constants" for easy tweaking
 * (You could also put these in getConfig() if you like.)
 */
static constexpr double kCirclesFraction   = 1.00;  // 25% circles
static constexpr double kRegularFraction   = 0.00;  // 50% "regular" polygons
static constexpr double kRandomPolyFraction= 0.00;  // 25% random polygons

static constexpr double kMinShapeSize = 0.1; // double the old 0.05
static constexpr double kMaxShapeSize = 0.4; // double the old 0.2

static constexpr int    kParticleCount = 50; // half the old 100

// Helper: builds a simple "regular polygon" shape with N sides
// Each vertex is placed on a circle of radius = `sz`
PolygonShape buildRegularPolygon(int sides, double sz)
{
    PolygonShape poly;
    poly.type = Components::ShapeType::Polygon; // We can mark it as Polygon
    double angleStep = 2.0 * M_PI / double(sides);
    for(int i = 0; i < sides; i++) {
        double angle = i * angleStep;
        double x = sz * std::cos(angle);
        double y = sz * std::sin(angle);
        poly.vertices.push_back(Vector(x, y));
    }
    return poly;
}

// Helper: builds a truly "random polygon" with random # of sides in [3..7]
// and random radial lengths in [0.5*sz..sz]
PolygonShape buildRandomPolygon(std::default_random_engine &gen, double sz)
{
    PolygonShape poly;
    poly.type = Components::ShapeType::Polygon;

    std::uniform_int_distribution<int> sideDist(3, 7); // 3..7 sides
    std::uniform_real_distribution<double> radiusDist(0.5 * sz, sz);

    int sides = sideDist(gen);
    double angleStep = 2.0 * M_PI / double(sides);

    double angle = 0.0;
    for(int i=0; i<sides; i++) {
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

    // Basic scenario scale
    cfg.MetersPerPixel = 1e-2;
    cfg.UniverseSizeMeters = SimulatorConstants::ScreenLength * cfg.MetersPerPixel;
    cfg.SecondsPerTick = 1.0 / SimulatorConstants::StepsPerSecond;
    cfg.TimeAcceleration = 1.0;
    cfg.GridSize = 50;
    cfg.CellSizePixels = (double)SimulatorConstants::ScreenLength / cfg.GridSize;

    // Adjusted scenario "constants"
    cfg.ParticleCount = kParticleCount;         // e.g. 50
    cfg.ParticleMassMean = 1.0;
    cfg.ParticleMassStdDev = 0.1;
    cfg.GravitationalSoftener = 1e-2;
    cfg.CollisionCoeffRestitution = 0.5;
    cfg.DragCoeff = 0.0;
    cfg.ParticleDensity = 0.5;
    cfg.InitialVelocityFactor = 1.0;

    // Example system config for random polygons
    cfg.activeSystems = {
        Systems::SystemType::COLLISION,
        Systems::SystemType::BASIC_GRAVITY,
        Systems::SystemType::ROTATION,
        Systems::SystemType::BOUNDARY,
        Systems::SystemType::DAMPENING,
        Systems::SystemType::MOVEMENT,
        Systems::SystemType::SLEEP
    };

    return cfg;
}

void RandomPolygonsScenario::createEntities(entt::registry &registry) const
{
    std::cerr << "Creating Random Polygons scenario...\n";
    std::default_random_engine generator{static_cast<unsigned int>(time(0))};

    // We'll just place them in a grid, as before
    int totalParticles = SimulatorConstants::ParticleCount;
    int circlesCount   = (int)std::round(kCirclesFraction * totalParticles);
    int regularCount   = (int)std::round(kRegularFraction * totalParticles);
    int randomCount    = totalParticles - circlesCount - regularCount; // leftover

    // Example: we arrange them in a NxN grid
    int side = (int)std::sqrt((double)totalParticles);
    double spacing = SimulatorConstants::UniverseSizeMeters / (side + 1);

    std::normal_distribution<> velocityDist(0.0, 1.0);
    std::normal_distribution<> massDist(SimulatorConstants::ParticleMassMean,
                                        SimulatorConstants::ParticleMassStdDev);

    // For shape size, we double the old range: [0.05..0.2] => [0.1..0.4]
    std::uniform_real_distribution<double> sizeDist(kMinShapeSize, kMaxShapeSize);
    std::uniform_int_distribution<> colorDist(100, 255);

    int created = 0;
    int cCount=0, rCount=0, randCount=0; // track how many of each we've made

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
            // a friction-ish material
            registry.emplace<Components::Material>(entity, 0.001, 0.001);

            double sz = sizeDist(generator);
            double I;  // we'll compute the inertia moment

            // Decide shape type:
            //   first circlesCount = circles
            //   then next regularCount = regular polygons
            //   else random polygons
            if (cCount < circlesCount) {
                // circle
                registry.emplace<Components::Shape>(entity, Components::ShapeType::Circle, sz);
                registry.emplace<CircleShape>(entity, sz);
                I = 0.5 * mass_val * (sz * sz);
                cCount++;
            }
            else if (rCount < regularCount) {
                // regular polygon
                // pick from {3,4,5,6}, e.g.
                std::uniform_int_distribution<int> sidesDist(3,6);
                int sides = sidesDist(generator);

                PolygonShape poly = buildRegularPolygon(sides, sz);
                registry.emplace<Components::Shape>(entity, Components::ShapeType::Polygon, sz);
                registry.emplace<PolygonShape>(entity, poly);

                // approximate inertia: ~ c * m * (sz^2)
                // for roughness, let's do 0.5*m*(sz^2)
                I = 0.5 * mass_val * (sz * sz);
                rCount++;
            }
            else {
                // random polygon
                PolygonShape poly = buildRandomPolygon(generator, sz);
                registry.emplace<Components::Shape>(entity, Components::ShapeType::Polygon, sz);
                registry.emplace<PolygonShape>(entity, poly);

                I = 0.5 * mass_val * (sz * sz);
                randCount++;
            }

            registry.emplace<Components::AngularPosition>(entity, 0.0);
            registry.emplace<Components::AngularVelocity>(entity, 0.0);
            registry.emplace<Components::Inertia>(entity, I);

            // random color
            int rr = colorDist(generator);
            int gg = colorDist(generator);
            int bb = colorDist(generator);
            registry.emplace<Components::Color>(entity, rr, gg, bb);

            created++;
        }
    }

    std::cerr << "...Created " << created << " random polygons total.\n";
    std::cerr << "   Circles:  " << cCount << "\n"
              << "   Regular:  " << rCount << "\n"
              << "   Random:   " << randCount << "\n";
}