#include <random>
#include <cmath>
#include <ctime>
#include <iostream>

#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/scenarios/keplerian_disk.hpp"
#include "nbody/scenarios/scenario_config.hpp"

ScenarioConfig KeplerianDiskScenario::getConfig() const {
    ScenarioConfig cfg;

    cfg.MetersPerPixel = 1e7;
    cfg.UniverseSizeMeters = SimulatorConstants::ScreenLength * cfg.MetersPerPixel;

    double const centralMass = 1e25;
    double const innerR = 100 * cfg.MetersPerPixel;
    double const period = 2 * SimulatorConstants::Pi * std::sqrt(std::pow(innerR, 3) /
                  (SimulatorConstants::RealG * centralMass));

    cfg.SecondsPerTick = 1.0 / SimulatorConstants::StepsPerSecond;
    cfg.TimeAcceleration = period / (5.0 * SimulatorConstants::StepsPerSecond) * 20.0;

    cfg.GridSize = 100;
    cfg.CellSizePixels = static_cast<double>(SimulatorConstants::ScreenLength) / cfg.GridSize;

    cfg.ParticleCount = 1000;
    cfg.ParticleMassMean = 1e22;
    cfg.ParticleMassStdDev = 1e21;
    cfg.GravitationalSoftener = 2e7;
    cfg.CollisionCoeffRestitution = 0.5;
    cfg.DragCoeff = 1e-11;
    cfg.ParticleDensity = 0.1;
    cfg.InitialVelocityFactor = 1.0;

    // ECS systems
    cfg.activeSystems = {
        Systems::SystemType::BARNES_HUT,
        Systems::SystemType::GRID_THERMODYNAMICS,
        Systems::SystemType::MOVEMENT
    };

    return cfg;
}

void KeplerianDiskScenario::createEntities(entt::registry &registry) const {
    createCentralBody(registry);
    createKeplerianDisk(registry);
}

void KeplerianDiskScenario::createCentralBody(entt::registry &registry) {
    auto center = registry.create();

    double cx = (SimulatorConstants::ScreenLength / 2.0) * SimulatorConstants::MetersPerPixel;
    double cy = (SimulatorConstants::ScreenLength / 2.0) * SimulatorConstants::MetersPerPixel;

    registry.emplace<Components::Position>(center, cx, cy);
    registry.emplace<Components::Velocity>(center, 0.0, 0.0);
    registry.emplace<Components::Mass>(center, SimulatorConstants::ParticleMassMean * 100.0);
    registry.emplace<Components::ParticlePhase>(center, Components::Phase::Solid);
    registry.emplace<Components::Shape>(center, Components::ShapeType::Circle, 1e7);
    registry.emplace<Components::Color>(center, 255, 255, 0);
}

namespace {
    double keplerVel(double r, double massCentral) {
        return std::sqrt(SimulatorConstants::RealG * massCentral / r);
    }
    double diskHeight(double rpix) {
        const double inr = 100.0;
        const double scale = inr / 20.0;
        return scale * std::pow(rpix / inr, 1.25);
    }
    double diskDensity(double rpix) {
        const double inr = 100.0;
        return std::pow(inr / rpix, 15.0 / 8.0);
    }
}

void KeplerianDiskScenario::createKeplerianDisk(entt::registry &registry) {
    std::cerr << "Creating Keplerian Disk...\n";

    std::default_random_engine gen{static_cast<unsigned int>(time(nullptr))};
    double const cx = (SimulatorConstants::ScreenLength / 2.0) * SimulatorConstants::MetersPerPixel;
    double const cy = (SimulatorConstants::ScreenLength / 2.0) * SimulatorConstants::MetersPerPixel;

    double const minRpix = 100.0;
    double const maxRpix = SimulatorConstants::ScreenLength / 2.5;
    double const minRm = SimulatorConstants::pixelsToMeters(minRpix);

    double const centralMass = SimulatorConstants::ParticleMassMean * 100.0;
    std::uniform_real_distribution<> angleDist(0, 2 * SimulatorConstants::Pi);

    int created = 0;
    while (created < SimulatorConstants::ParticleCount - 1) {
        double rpix;
        double rm;
        double densThresh;
        do {
            std::uniform_real_distribution<> radDist(minRpix, maxRpix);
            rpix = radDist(gen);
            rm = SimulatorConstants::pixelsToMeters(rpix);

            std::uniform_real_distribution<> dDist(0, 1);
            densThresh = dDist(gen);
        } while (densThresh > diskDensity(rpix));

        double const angle = angleDist(gen);

        double const maxH = diskHeight(rpix);
        double const maxHm = SimulatorConstants::pixelsToMeters(maxH);
        std::normal_distribution<> hDist(0.0, maxHm / 3.0);
        double const hOff = hDist(gen);

        double x = cx + rm * std::cos(angle);
        double y = cy + rm * std::sin(angle) + hOff;

        double const baseVel = keplerVel(rm, centralMass);
        std::normal_distribution<> velVar(1.0, 0.01);
        double const speed = baseVel * velVar(gen);

        double vx = -speed * std::sin(angle);
        double vy =  speed * std::cos(angle);

        std::normal_distribution<> rVar(0.0, speed * 0.001);
        double const rv = rVar(gen);
        vx += rv * std::cos(angle);
        vy += rv * std::sin(angle);

        auto e = registry.create();
        registry.emplace<Components::Position>(e, x, y);
        registry.emplace<Components::Velocity>(e, vx, vy);
        registry.emplace<Components::ParticlePhase>(e, Components::Phase::Gas);

        double const baseMass = SimulatorConstants::ParticleMassMean;
        double const massFactor = std::pow(minRm / rm, 0.5);
        std::normal_distribution<> mVar(massFactor * baseMass, baseMass * 0.1);
        registry.emplace<Components::Mass>(e, mVar(gen));

        registry.emplace<Components::Shape>(e, Components::ShapeType::Circle,
                                            SimulatorConstants::MetersPerPixel * 0.5);
        registry.emplace<Components::Color>(e, 150, 150, 255);

        created++;
    }
    std::cerr << "...Created " << created << " disk particles.\n";
}