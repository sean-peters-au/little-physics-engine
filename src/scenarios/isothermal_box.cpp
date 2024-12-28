// File: src/scenarios/isothermal_box_scenario.cpp
#include "nbody/scenarios/isothermal_box.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include <random>
#include <cmath>
#include <ctime>
#include <iostream>

ScenarioConfig IsothermalBoxScenario::getConfig() const {
    ScenarioConfig cfg;

    cfg.MetersPerPixel = 1e6;
    cfg.UniverseSizeMeters = SimulatorConstants::ScreenLength * cfg.MetersPerPixel;
    cfg.SecondsPerTick = 1.0 / SimulatorConstants::StepsPerSecond;
    cfg.TimeAcceleration = 1.0;
    cfg.GridSize = 50;
    cfg.CellSizePixels = (double)SimulatorConstants::ScreenLength / cfg.GridSize;

    cfg.ParticleCount = 500;
    cfg.ParticleMassMean = 1e20;
    cfg.ParticleMassStdDev = 1e19;
    cfg.GravitationalSoftener = 1e6;
    cfg.CollisionCoeffRestitution = 0.5;
    cfg.DragCoeff = 0.0;
    cfg.ParticleDensity = 0.5;
    cfg.InitialVelocityFactor = 0.0;

    // Chosen ECS systems for an isothermal box
    cfg.activeSystems = {
        Systems::SystemType::SPH,
        Systems::SystemType::GRID_THERMODYNAMICS,
        Systems::SystemType::MOVEMENT
    };

    return cfg;
}

void IsothermalBoxScenario::createEntities(entt::registry &registry) const {
    std::cerr << "Creating Isothermal Box scenario...\n";
    std::default_random_engine generator{static_cast<unsigned int>(time(0))};

    double box_size = SimulatorConstants::UniverseSizeMeters;
    double spacing = box_size / std::sqrt(SimulatorConstants::ParticleCount);

    double mass_mean = SimulatorConstants::ParticleMassMean;
    std::normal_distribution<> massDist(mass_mean, mass_mean * 0.1);

    int particles = 0;
    int sideCount = (int)std::sqrt(SimulatorConstants::ParticleCount);
    for (int i = 0; i < sideCount; ++i) {
        for (int j = 0; j < sideCount; ++j) {
            double x_m = i * spacing + spacing * 0.5;
            double y_m = j * spacing + spacing * 0.5;

            if (x_m < box_size && y_m < box_size) {
                auto entity = registry.create();
                registry.emplace<Components::Position>(entity, x_m, y_m);
                registry.emplace<Components::Velocity>(entity, 0.0, 0.0);
                registry.emplace<Components::ParticlePhase>(entity, Components::Phase::Gas);
                registry.emplace<Components::Mass>(entity, massDist(generator));
                registry.emplace<Components::Shape>(entity, Components::ShapeType::Circle, 0.5);
                registry.emplace<Components::Color>(entity, 0, 200, 200);

                particles++;
                if (particles >= SimulatorConstants::ParticleCount) {
                    std::cerr << "...Created " << particles << " isothermal box particles.\n";
                    return;
                }
            }
        }
    }
    std::cerr << "...Created " << particles << " isothermal box particles.\n";
}