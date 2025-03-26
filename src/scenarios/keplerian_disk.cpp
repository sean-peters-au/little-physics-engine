#include <random>
#include <cmath>
#include <ctime>
#include <iostream>

#include "entt/entt.hpp"
#include "core/constants.hpp"
#include "entities/entity_components.hpp"
#include "math/polygon.hpp"
#include "scenarios/keplerian_disk.hpp"
#include "systems/shared_system_config.hpp"

ScenarioSystemConfig KeplerianDiskScenario::getSystemsConfig() const {
    ScenarioSystemConfig config;

    config.sharedConfig.MetersPerPixel = 1e7;
    config.sharedConfig.UniverseSizeMeters = SimulatorConstants::ScreenLength * config.sharedConfig.MetersPerPixel;

    double const innerR = scenarioEntityConfig.innerRadiusPixels * config.sharedConfig.MetersPerPixel;
    double const period = 2 * SimulatorConstants::Pi * std::sqrt(std::pow(innerR, 3) /
                  (SimulatorConstants::RealG * scenarioEntityConfig.centralMass));

    config.sharedConfig.SecondsPerTick = 1.0 / SimulatorConstants::StepsPerSecond;
    config.sharedConfig.TimeAcceleration = period / (scenarioEntityConfig.orbitalPeriodFraction * SimulatorConstants::StepsPerSecond) * 20.0;

    config.sharedConfig.GridSize = 100;
    config.sharedConfig.CellSizePixels = static_cast<double>(SimulatorConstants::ScreenLength) / config.sharedConfig.GridSize;

    config.sharedConfig.GravitationalSoftener = 2e7;
    config.sharedConfig.CollisionCoeffRestitution = 0.5;
    config.sharedConfig.DragCoeff = 1e-11;
    config.sharedConfig.ParticleDensity = 0.1;

    return config;
}

void KeplerianDiskScenario::createEntities(entt::registry &registry) const {
    ScenarioSystemConfig scenarioSystemConfig = getSystemsConfig();
    SharedSystemConfig sharedConfig = scenarioSystemConfig.sharedConfig;

    createCentralBody(registry, scenarioEntityConfig, sharedConfig);
    createKeplerianDisk(registry, scenarioEntityConfig, sharedConfig);
}

void KeplerianDiskScenario::createCentralBody(entt::registry &registry, const KeplerianDiskConfig& config, const SharedSystemConfig& sysConfig) {
    auto center = registry.create();

    double cx = (SimulatorConstants::ScreenLength / 2.0) * sysConfig.MetersPerPixel;
    double cy = (SimulatorConstants::ScreenLength / 2.0) * sysConfig.MetersPerPixel;

    registry.emplace<Components::Position>(center, cx, cy);
    registry.emplace<Components::Velocity>(center, 0.0, 0.0);
    registry.emplace<Components::Mass>(center, config.centralMass);
    registry.emplace<Components::ParticlePhase>(center, Components::Phase::Gas);
    
    double const bodySize = 2.0 * sysConfig.MetersPerPixel;
    registry.emplace<Components::Shape>(center, Components::ShapeType::Circle, bodySize);
    registry.emplace<CircleShape>(center, bodySize);
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

void KeplerianDiskScenario::createKeplerianDisk(entt::registry &registry, const KeplerianDiskConfig& config, const SharedSystemConfig& sysConfig) {
    std::cerr << "Creating Keplerian Disk...\n";

    std::default_random_engine gen{static_cast<unsigned int>(time(nullptr))};
    
    double const cx = (SimulatorConstants::ScreenLength / 2.0) * sysConfig.MetersPerPixel;
    double const cy = (SimulatorConstants::ScreenLength / 2.0) * sysConfig.MetersPerPixel;

    double const minRpix = config.innerRadiusPixels;
    double const maxRpix = SimulatorConstants::ScreenLength / config.outerRadiusFactor;
    double const minRm = SimulatorConstants::pixelsToMeters(minRpix, sysConfig.MetersPerPixel);

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
            rm = SimulatorConstants::pixelsToMeters(rpix, sysConfig.MetersPerPixel);

            std::uniform_real_distribution<> dDist(0, 1);
            densThresh = dDist(gen);
        } while (densThresh > diskDensity(rpix, config));

        double const angle = angleDist(gen);

        double const maxH = diskHeight(rpix, config);
        double const maxHm = SimulatorConstants::pixelsToMeters(maxH, sysConfig.MetersPerPixel);
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
        registry.emplace<Components::ParticlePhase>(e, Components::Phase::Gas);

        double const baseMass = config.particleMassMean;
        double const massFactor = std::pow(minRm / rm, config.massRadialPowerLaw);
        std::normal_distribution<> mVar(massFactor * baseMass, config.particleMassStdDev);
        registry.emplace<Components::Mass>(e, mVar(gen));

        double const particleSize = sysConfig.MetersPerPixel * 0.5;
        registry.emplace<Components::Shape>(e, Components::ShapeType::Circle, particleSize);
        registry.emplace<CircleShape>(e, particleSize);
        registry.emplace<Components::Color>(e, 255, 255, 255);

        created++;
    }
    std::cerr << "...Created " << created << " disk particles.\n";
}