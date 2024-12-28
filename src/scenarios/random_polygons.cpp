// File: src/scenarios/random_polygons_scenario.cpp
#include "nbody/scenarios/random_polygons.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/math/polygon.hpp"

#include <random>
#include <cmath>
#include <ctime>
#include <iostream>

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

    // Choose ECS systems for random polygons scenario
    cfg.activeSystems = {
        Systems::SystemType::COLLISION,
        Systems::SystemType::BASIC_GRAVITY,
        Systems::SystemType::ROTATION,
        Systems::SystemType::BOUNDARY,
        Systems::SystemType::DAMPENING,
        Systems::SystemType::MOVEMENT,
        Systems::SystemType::SLEEP,
    };

    return cfg;
}

void RandomPolygonsScenario::createEntities(entt::registry &registry) const {
    std::cerr << "Creating Random Polygons scenario...\n";
    std::default_random_engine generator{static_cast<unsigned int>(time(0))};

    int balls_per_side = (int)std::sqrt(SimulatorConstants::ParticleCount);
    double spacing = SimulatorConstants::UniverseSizeMeters / (balls_per_side + 1);

    std::normal_distribution<> velocityDist(0.0, 1.0);
    std::normal_distribution<> massDist(SimulatorConstants::ParticleMassMean,
                                        SimulatorConstants::ParticleMassStdDev);

    std::uniform_real_distribution<> sizeDist(0.05, 0.2);
    std::uniform_real_distribution<> shapeDist(0.0, 1.0);
    std::uniform_int_distribution<> colorDist(100, 255);

    int particles_created = 0;
    for (int i = 0; i < balls_per_side && particles_created < SimulatorConstants::ParticleCount; ++i) {
        for (int j = 0; j < balls_per_side && particles_created < SimulatorConstants::ParticleCount; ++j) {
            double x_m = (i + 1) * spacing;
            double y_m = (j + 1) * spacing;

            auto entity = registry.create();
            double mass_val = massDist(generator);

            registry.emplace<Components::Position>(entity, x_m, y_m);
            registry.emplace<Components::Velocity>(entity, velocityDist(generator), velocityDist(generator));
            registry.emplace<Components::Mass>(entity, mass_val);
            registry.emplace<Components::ParticlePhase>(entity, Components::Phase::Solid);
            registry.emplace<Components::Sleep>(entity);
            registry.emplace<Components::Material>(entity, 0.001, 0.001);

            double s = shapeDist(generator);
            double sz = sizeDist(generator);

            double I;
            if (s < 0.5) {
                // Circle
                registry.emplace<Components::Shape>(entity, Components::ShapeType::Circle, sz);
                registry.emplace<CircleShape>(entity, sz);
                I = 0.5 * mass_val * (sz * sz);
            } else {
                // Square
                registry.emplace<Components::Shape>(entity, Components::ShapeType::Square, sz);
                PolygonShape poly;
                poly.type = Components::ShapeType::Square;
                poly.vertices = {
                    Vector(-sz, -sz),
                    Vector(sz, -sz),
                    Vector(sz, sz),
                    Vector(-sz, sz)
                };
                registry.emplace<PolygonShape>(entity, poly);
                I = (4.0 / 3.0) * mass_val * (sz * sz);
            }

            registry.emplace<Components::AngularPosition>(entity, 0.0);
            registry.emplace<Components::AngularVelocity>(entity, 0.0);
            registry.emplace<Components::Inertia>(entity, I);

            registry.emplace<Components::Color>(entity,
                colorDist(generator),
                colorDist(generator),
                colorDist(generator));

            particles_created++;
        }
    }
    std::cerr << "...Created " << particles_created << " random polygons.\n";
}