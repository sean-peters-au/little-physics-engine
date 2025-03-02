#include <random>
#include <cmath>
#include <ctime>
#include <iostream>

#include "entt/entt.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/scenarios/keplerian_disk.hpp"
#include "nbody/core/system_config.hpp"

SystemConfig KeplerianDiskScenario::getConfig() const {
    SystemConfig cfg;

    cfg.MetersPerPixel = 1e7;
    cfg.UniverseSizeMeters = SimulatorConstants::ScreenLength * cfg.MetersPerPixel;

    double const innerR = diskConfig.innerRadiusPixels * cfg.MetersPerPixel;
    double const period = 2 * SimulatorConstants::Pi * std::sqrt(std::pow(innerR, 3) /
                  (SimulatorConstants::RealG * diskConfig.centralMass));

    cfg.SecondsPerTick = 1.0 / SimulatorConstants::StepsPerSecond;
    cfg.TimeAcceleration = period / (diskConfig.orbitalPeriodFraction * SimulatorConstants::StepsPerSecond) * 20.0;

    cfg.GridSize = 100;
    cfg.CellSizePixels = static_cast<double>(SimulatorConstants::ScreenLength) / cfg.GridSize;

    cfg.GravitationalSoftener = 2e7;
    cfg.CollisionCoeffRestitution = 0.5;
    cfg.DragCoeff = 1e-11;
    cfg.ParticleDensity = 0.1;

    // ECS systems
    cfg.activeSystems = {
        Systems::SystemType::BARNES_HUT,
        Systems::SystemType::MOVEMENT
    };

    return cfg;
}

void KeplerianDiskScenario::createEntities(entt::registry &registry) const {
    createCentralBody(registry, diskConfig);
    createKeplerianDisk(registry, diskConfig);
}

void KeplerianDiskScenario::createCentralBody(entt::registry &registry, const KeplerianDiskConfig& config) {
    auto center = registry.create();

    double cx = (SimulatorConstants::ScreenLength / 2.0) * 1e7; // Use a constant or get from scenario config
    double cy = (SimulatorConstants::ScreenLength / 2.0) * 1e7;

    registry.emplace<Components::Position>(center, cx, cy);
    registry.emplace<Components::Velocity>(center, 0.0, 0.0);
    registry.emplace<Components::Mass>(center, config.centralMass);
    registry.emplace<Components::ParticlePhase>(center, Components::Phase::Solid);
    registry.emplace<Components::Shape>(center, Components::ShapeType::Circle, 2e7);
    registry.emplace<Components::Color>(center, 255, 255, 0);
}

namespace {
    double keplerVel(double r, double massCentral) {
        return std::sqrt(SimulatorConstants::RealG * massCentral / r);
    }
    
    double diskHeight(double rpix, const KeplerianDiskConfig& config) {
        const double inr = config.innerRadiusPixels;
        const double scale = inr / config.heightScaleFactor;
        return scale * std::pow(rpix / inr, config.heightPowerLaw);
    }
    
    double diskDensity(double rpix, const KeplerianDiskConfig& config) {
        const double inr = config.innerRadiusPixels;
        return std::pow(inr / rpix, config.densityPowerLaw);
    }
}

void KeplerianDiskScenario::createKeplerianDisk(entt::registry &registry, const KeplerianDiskConfig& config) {
    std::cerr << "Creating Keplerian Disk...\n";

    std::default_random_engine gen{static_cast<unsigned int>(time(nullptr))};
    double metersPerPixel = 1e7; // Use a constant or get from scenario config
    double const cx = (SimulatorConstants::ScreenLength / 2.0) * metersPerPixel;
    double const cy = (SimulatorConstants::ScreenLength / 2.0) * metersPerPixel;

    double const minRpix = config.innerRadiusPixels;
    double const maxRpix = SimulatorConstants::ScreenLength / config.outerRadiusFactor;
    double const minRm = SimulatorConstants::pixelsToMeters(minRpix, metersPerPixel);

    double const centralMass = config.centralMass;
    std::uniform_real_distribution<> angleDist(0, 2 * SimulatorConstants::Pi);

    int created = 0;
    
    while (created < config.particleCount - 1) {
        double rpix;
        double rm;
        double densThresh;
        do {
            std::uniform_real_distribution<> radDist(minRpix, maxRpix);
            rpix = radDist(gen);
            rm = SimulatorConstants::pixelsToMeters(rpix, metersPerPixel);

            std::uniform_real_distribution<> dDist(0, 1);
            densThresh = dDist(gen);
        } while (densThresh > diskDensity(rpix, config));

        double const angle = angleDist(gen);

        double const maxH = diskHeight(rpix, config);
        double const maxHm = SimulatorConstants::pixelsToMeters(maxH, metersPerPixel);
        std::normal_distribution<> hDist(0.0, maxHm / 3.0);
        double const hOff = hDist(gen);

        double x = cx + rm * std::cos(angle);
        double y = cy + rm * std::sin(angle) + hOff;

        double const baseVel = keplerVel(rm, centralMass);
        std::normal_distribution<> velVar(1.0, config.velocityDispersionFactor);
        double const speed = baseVel * velVar(gen);

        double vx = -speed * std::sin(angle);
        double vy =  speed * std::cos(angle);

        std::normal_distribution<> rVar(0.0, speed * config.radialVelocityFactor);
        double const rv = rVar(gen);
        vx += rv * std::cos(angle);
        vy += rv * std::sin(angle);

        auto e = registry.create();
        registry.emplace<Components::Position>(e, x, y);
        registry.emplace<Components::Velocity>(e, vx, vy);
        registry.emplace<Components::ParticlePhase>(e, Components::Phase::Solid);

        double const baseMass = config.particleMassMean;
        double const massFactor = std::pow(minRm / rm, config.massRadialPowerLaw);
        std::normal_distribution<> mVar(massFactor * baseMass, config.particleMassStdDev);
        registry.emplace<Components::Mass>(e, mVar(gen));

        registry.emplace<Components::Shape>(e, Components::ShapeType::Circle,
                                            metersPerPixel * 0.5);
        registry.emplace<Components::Color>(e, 255, 255, 255);

        created++;
    }
    std::cerr << "...Created " << created << " disk particles.\n";
}