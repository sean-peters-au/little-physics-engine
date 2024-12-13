#include "ecs_simulator.h"
#include "simulator_constants.h"
#include "systems/movement_system.h"
#include "systems/barnes_hut_system.h"
#include "systems/grid_thermodynamics_system.h"
#include "systems/sph_system.h"
#include "systems/basic_gravity_system.h"
#include <random>
#include <cmath>
#include <iostream>

ECSSimulator::ECSSimulator() : currentScenario(SimulatorConstants::SimulationType::CELESTIAL_GAS) {}

ECSSimulator::ECSSimulator(CoordinateSystem* /*coordSystem*/)
    : currentScenario(SimulatorConstants::SimulationType::CELESTIAL_GAS) {}

void ECSSimulator::setScenario(SimulatorConstants::SimulationType scenario) {
    currentScenario = scenario;
}

void ECSSimulator::reset() {
    registry.clear();
    SimulatorConstants::initializeConstants(currentScenario);
    init();
}

void ECSSimulator::init() {
    registry.clear(); 
    // Initialize scenario
    switch (currentScenario) {
        case SimulatorConstants::SimulationType::CELESTIAL_GAS:
            // Create a central body + keplerian disk
            createCentralBody();
            createKeplerianDisk();
            break;
        case SimulatorConstants::SimulationType::ISOTHERMAL_BOX:
            createIsothermalBox();
            break;
        case SimulatorConstants::SimulationType::BOUNCY_BALLS:
            createBouncyBalls();
            break;
        default:
            // Fallback
            createIsothermalBox();
            break;
    }
}

void ECSSimulator::tick() {
    // Run systems in the order specified by ActiveSystems
    for (const auto& system : SimulatorConstants::ActiveSystems) {
        switch (system) {
            case SimulatorConstants::ECSSystem::BASIC_GRAVITY:
                Systems::BasicGravitySystem::update(registry);
                break;
            case SimulatorConstants::ECSSystem::BARNES_HUT:
                Systems::BarnesHutSystem::update(registry);
                break;
            case SimulatorConstants::ECSSystem::SPH:
                Systems::SPHSystem::update(registry);
                break;
            case SimulatorConstants::ECSSystem::GRID_THERMODYNAMICS:
                Systems::GridThermodynamicsSystem::update(registry);
                break;
            case SimulatorConstants::ECSSystem::MOVEMENT:
                Systems::MovementSystem::update(registry);
                break;
        }
    }
}

void ECSSimulator::createCentralBody() {
    auto center = registry.create();
    double center_x_m = (SimulatorConstants::ScreenLength / 2.0) * SimulatorConstants::MetersPerPixel;
    double center_y_m = (SimulatorConstants::ScreenLength / 2.0) * SimulatorConstants::MetersPerPixel;

    registry.emplace<Components::Position>(center, center_x_m, center_y_m);
    registry.emplace<Components::Velocity>(center, 0.0, 0.0);
    registry.emplace<Components::Mass>(center, SimulatorConstants::ParticleMassMean * 100.0);
    registry.emplace<Components::ParticlePhase>(center, Components::Phase::Solid);
}

double ECSSimulator::calculateKeplerianVelocity(double radius_meters, double central_mass) const {
    return std::sqrt(SimulatorConstants::RealG * central_mass / radius_meters);
}

double ECSSimulator::calculateDiskHeight(double radius, double /*max_radius*/) const {
    const double inner_radius = 100.0;
    const double height_scale = inner_radius / 20.0;
    return height_scale * std::pow(radius / inner_radius, 5.0/4.0);
}

double ECSSimulator::calculateDiskDensity(double radius, double /*max_radius*/) const {
    const double inner_radius = 100.0; 
    return std::pow(inner_radius / radius, 15.0/8.0);
}

void ECSSimulator::createKeplerianDisk() {
    std::default_random_engine generator{static_cast<unsigned int>(time(0))};
    double center_x_m = (SimulatorConstants::ScreenLength / 2.0) * SimulatorConstants::MetersPerPixel;
    double center_y_m = (SimulatorConstants::ScreenLength / 2.0) * SimulatorConstants::MetersPerPixel;

    double min_radius_pixels = 100.0;
    double max_radius_pixels = SimulatorConstants::ScreenLength / 2.5;
    double min_radius_m = SimulatorConstants::pixelsToMeters(min_radius_pixels);

    double central_mass = SimulatorConstants::ParticleMassMean * 100.0;
    std::uniform_real_distribution<> angle_dist(0, 2 * SimulatorConstants::Pi);

    int particles_created = 0;
    while (particles_created < SimulatorConstants::ParticleCount - 1) {
        double radius_pixels, radius_meters;
        double density_threshold;
        do {
            std::uniform_real_distribution<> radius_dist(min_radius_pixels, max_radius_pixels);
            radius_pixels = radius_dist(generator);
            radius_meters = SimulatorConstants::pixelsToMeters(radius_pixels);
            std::uniform_real_distribution<> density_dist(0, 1);
            density_threshold = density_dist(generator);
        } while (density_threshold > calculateDiskDensity(radius_pixels, max_radius_pixels));

        double angle = angle_dist(generator);
        double max_height_pixels = calculateDiskHeight(radius_pixels, max_radius_pixels);
        double max_height_m = SimulatorConstants::pixelsToMeters(max_height_pixels);

        std::normal_distribution<> height_dist(0.0, max_height_m/3.0);
        double height_offset_m = height_dist(generator);

        double x_m = center_x_m + radius_meters * cos(angle);
        double y_m = center_y_m + radius_meters * sin(angle) + height_offset_m;

        double base_velocity_m_s = calculateKeplerianVelocity(radius_meters, central_mass);

        std::normal_distribution<> vel_variation(1.0, 0.01);
        double speed_m_s = base_velocity_m_s * vel_variation(generator);

        double vx_m_s = -speed_m_s * sin(angle);
        double vy_m_s = speed_m_s * cos(angle);

        std::normal_distribution<> radial_vel_dist(0.0, speed_m_s * 0.001);
        double radial_vel = radial_vel_dist(generator);
        vx_m_s += radial_vel * cos(angle);
        vy_m_s += radial_vel * sin(angle);

        auto entity = registry.create();
        registry.emplace<Components::Position>(entity, x_m, y_m);
        registry.emplace<Components::Velocity>(entity, vx_m_s, vy_m_s);
        registry.emplace<Components::ParticlePhase>(entity, Components::Phase::Gas);

        double base_mass = SimulatorConstants::ParticleMassMean;
        double mass_factor = std::pow(min_radius_m / radius_meters, 0.5);
        std::normal_distribution<> mass_variation(mass_factor * base_mass, base_mass * 0.1);
        registry.emplace<Components::Mass>(entity, mass_variation(generator));

        particles_created++;
    }
}

void ECSSimulator::createBouncyBalls() {
    std::default_random_engine generator{static_cast<unsigned int>(time(0))};
    
    // Create a grid of balls with some spacing
    int balls_per_side = static_cast<int>(std::sqrt(SimulatorConstants::ParticleCount));
    double spacing = SimulatorConstants::UniverseSizeMeters / (balls_per_side + 1);
    
    // Small random initial velocity (±1 m/s in each direction)
    std::normal_distribution<> velocity_dist(0.0, 1.0);
    
    // Mass distribution around 1kg
    std::normal_distribution<> mass_dist(SimulatorConstants::ParticleMassMean, 
                                       SimulatorConstants::ParticleMassStdDev);

    int particles_created = 0;
    for (int i = 0; i < balls_per_side && particles_created < SimulatorConstants::ParticleCount; i++) {
        for (int j = 0; j < balls_per_side && particles_created < SimulatorConstants::ParticleCount; j++) {
            // Position in meters, offset by spacing to keep away from edges
            double x_m = (i + 1) * spacing;
            double y_m = (j + 1) * spacing;

            // Create entity and add components
            auto entity = registry.create();
            registry.emplace<Components::Position>(entity, x_m, y_m);
            registry.emplace<Components::Velocity>(entity, 
                velocity_dist(generator), 
                velocity_dist(generator));
            registry.emplace<Components::Mass>(entity, mass_dist(generator));
            registry.emplace<Components::ParticlePhase>(entity, Components::Phase::Solid);

            particles_created++;
        }
    }
}

void ECSSimulator::createIsothermalBox() {
    std::default_random_engine generator{static_cast<unsigned int>(time(0))};
    double box_size = SimulatorConstants::UniverseSizeMeters;
    double spacing = box_size / std::sqrt(SimulatorConstants::ParticleCount);

    double mass_mean = SimulatorConstants::ParticleMassMean;
    std::normal_distribution<> mass_dist(mass_mean, mass_mean * 0.1);

    int particles = 0;
    for (int i = 0; i < static_cast<int>(std::sqrt(SimulatorConstants::ParticleCount)); i++) {
        for (int j = 0; j < static_cast<int>(std::sqrt(SimulatorConstants::ParticleCount)); j++) {
            double x_m = i * spacing + spacing*0.5;
            double y_m = j * spacing + spacing*0.5;
            if (x_m < box_size && y_m < box_size) {
                auto entity = registry.create();
                registry.emplace<Components::Position>(entity, x_m, y_m);
                registry.emplace<Components::Velocity>(entity, 0.0, 0.0);
                registry.emplace<Components::ParticlePhase>(entity, Components::Phase::Gas);
                registry.emplace<Components::Mass>(entity, mass_dist(generator));
                particles++;
                if (particles >= SimulatorConstants::ParticleCount) return;
            }
        }
    }
}