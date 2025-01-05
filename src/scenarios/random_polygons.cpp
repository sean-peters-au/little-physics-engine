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
 * Helper function to generate a random convex polygon with N sides.
 *   - N from 3..8, or any range you like
 *   - each vertex has a random angle and radius
 *   - result is typically a "spiky" convex shape
 */
static PolygonShape buildRandomConvexPolygon(std::default_random_engine &generator,
                                             int sides,
                                             double maxRadius)
{
    PolygonShape poly;
    poly.type = Components::ShapeType::Polygon;

    // We'll store angles in ascending order to ensure a convex shape
    std::vector<double> angles;
    angles.reserve(sides);

    // For random radius [0.3..1.0] * maxRadius
    std::uniform_real_distribution<double> radiusDist(0.3, 1.0);
    // For angles [0..2*pi]
    std::uniform_real_distribution<double> angleDist(0.0, 2.0 * SimulatorConstants::Pi);

    // Generate random angles
    for (int i = 0; i < sides; ++i) {
        angles.push_back(angleDist(generator));
    }
    // Sort them ascending
    std::sort(angles.begin(), angles.end());

    // Build polygon vertices from angles/radii
    for (double ang : angles) {
        double rFactor = radiusDist(generator);  // random fraction of maxRadius
        double r = rFactor * maxRadius;
        double x = r * std::cos(ang);
        double y = r * std::sin(ang);
        poly.vertices.push_back(Vector(x, y));
    }

    return poly;
}

/**
 * The config remains basically the same, except we might turn on some boundary
 * system or other systems if desired.
 */
ScenarioConfig RandomPolygonsScenario::getConfig() const {
    ScenarioConfig cfg;

    cfg.MetersPerPixel = 1e-2;
    cfg.UniverseSizeMeters = SimulatorConstants::ScreenLength * cfg.MetersPerPixel;
    cfg.SecondsPerTick = 1.0 / SimulatorConstants::StepsPerSecond;
    cfg.TimeAcceleration = 1.0;
    cfg.GridSize = 50;
    cfg.CellSizePixels = (double)SimulatorConstants::ScreenLength / cfg.GridSize;

    cfg.ParticleCount = 100;
    cfg.ParticleMassMean = 1.0;
    cfg.ParticleMassStdDev = 0.1;
    cfg.GravitationalSoftener = 1e-2;
    cfg.CollisionCoeffRestitution = 0.5;
    cfg.DragCoeff = 0.0;
    cfg.ParticleDensity = 0.5;
    cfg.InitialVelocityFactor = 1.0;

    // ECS systems to run for random polygons
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

void RandomPolygonsScenario::createEntities(entt::registry &registry) const {
    std::cerr << "Creating Random Polygons scenario...\n";
    std::default_random_engine generator{static_cast<unsigned int>(time(0))};

    // We'll place these polygons in a grid (like bouncy balls)
    int balls_per_side = (int)std::sqrt(SimulatorConstants::ParticleCount);
    double spacing = SimulatorConstants::UniverseSizeMeters / (balls_per_side + 1);

    // Some distributions for velocity, mass, shape size, color
    std::normal_distribution<> velocityDist(0.0, 1.0);
    std::normal_distribution<> massDist(SimulatorConstants::ParticleMassMean,
                                        SimulatorConstants::ParticleMassStdDev);
    std::uniform_real_distribution<> sizeDist(0.05, 0.2);
    std::uniform_real_distribution<> shapePickDist(0.0, 1.0); // to decide circle vs polygon
    std::uniform_int_distribution<> colorDist(100, 255);

    int particles_created = 0;
    for (int i = 0; i < balls_per_side && particles_created < SimulatorConstants::ParticleCount; ++i) {
        for (int j = 0; j < balls_per_side && particles_created < SimulatorConstants::ParticleCount; ++j) {

            double x_m = (i + 1) * spacing;
            double y_m = (j + 1) * spacing;

            auto entity = registry.create();
            double mass_val = massDist(generator);

            // Basic components
            registry.emplace<Components::Position>(entity, x_m, y_m);
            registry.emplace<Components::Velocity>(entity,
                                                   velocityDist(generator),
                                                   velocityDist(generator));
            registry.emplace<Components::Mass>(entity, mass_val);
            registry.emplace<Components::ParticlePhase>(entity, Components::Phase::Solid);
            registry.emplace<Components::Sleep>(entity);
            registry.emplace<Components::Material>(entity, 0.001, 0.001);

            double shapeSeed = shapePickDist(generator);
            double sz = sizeDist(generator);

            double inertia = 0.0;

            if (shapeSeed < 0.25) {
                // 25% circles
                registry.emplace<Components::Shape>(entity, Components::ShapeType::Circle, sz);
                registry.emplace<CircleShape>(entity, sz);

                // Moment of inertia for circle: I = 0.5 * m * r^2
                inertia = 0.5 * mass_val * (sz * sz);
            } else {
                // 75% random polygons
                // Choose random side count from 3..8
                std::uniform_int_distribution<int> sideDist(3, 8);
                int nSides = sideDist(generator);

                // Build random convex polygon
                auto poly = buildRandomConvexPolygon(generator, nSides, sz);

                // Mark shape type as "Polygon" (the user can define a new type in the enum if needed)
                registry.emplace<Components::Shape>(entity, Components::ShapeType::Polygon, sz);
                registry.emplace<PolygonShape>(entity, poly);

                // Approx moment of inertia? For a generic polygon, let's guess an average formula
                // We'll do: I ~ (m * R^2) * 0.5. This is a rough guess for small polygons
                double r2 = sz * sz;
                inertia = 0.5 * mass_val * r2;
            }

            // Angular components
            registry.emplace<Components::AngularPosition>(entity, 0.0);
            registry.emplace<Components::AngularVelocity>(entity, 0.0);
            registry.emplace<Components::Inertia>(entity, inertia);

            // Random color
            registry.emplace<Components::Color>(entity,
                colorDist(generator),
                colorDist(generator),
                colorDist(generator));

            particles_created++;
        }
    }
    std::cerr << "...Created " << particles_created << " random polygons.\n";
}