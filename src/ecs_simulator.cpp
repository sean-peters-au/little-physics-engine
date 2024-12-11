#include "ecs_simulator.h"
#include "simulator_constants.h"
#include "systems/movement_system.h"
#include "systems/barnes_hut_system.h"
#include "systems/grid_thermodynamics_system.h"
#include <random>
#include <cmath>
#include <iostream>

ECSSimulator::ECSSimulator() {}

ECSSimulator::ECSSimulator(CoordinateSystem* /*coordSystem*/) {}

void ECSSimulator::init() {
    createCentralBody();
    createKeplerianDisk();
}

void ECSSimulator::createCentralBody() {
    auto center = registry.create();
    // Convert center from pixels to meters
    double center_x_m = (SimulatorConstants::ScreenLength / 2.0) * SimulatorConstants::MetersPerPixel;
    double center_y_m = (SimulatorConstants::ScreenLength / 2.0) * SimulatorConstants::MetersPerPixel;
    
    registry.emplace<Components::Position>(center, center_x_m, center_y_m);
    registry.emplace<Components::Velocity>(center, 0.0, 0.0);
    registry.emplace<Components::Mass>(center, SimulatorConstants::ParticleMassMean * 100.0);
}

double ECSSimulator::calculateKeplerianVelocity(double radius_meters, double central_mass) const {
    return std::sqrt(SimulatorConstants::RealG * central_mass / radius_meters);
}

double ECSSimulator::calculateDiskHeight(double radius, double /*max_radius*/) const {
    // Disk height follows h ∝ r^(5/4) for typical accretion disks
    // Scale it so the height at the inner radius is 1/20 of the radius
    // and increases outward according to the power law
    const double inner_radius = 100.0;  // pixels
    const double height_scale = inner_radius / 20.0;
    return height_scale * std::pow(radius / inner_radius, 5.0/4.0);
}

double ECSSimulator::calculateDiskDensity(double radius, double /*max_radius*/) const {
    // Surface density follows Σ ∝ r^(-15/8) for typical accretion disks
    // Normalize so density at inner radius is 1.0
    const double inner_radius = 100.0;  // pixels
    return std::pow(inner_radius / radius, 15.0/8.0);
}

void ECSSimulator::createKeplerianDisk() {
    std::default_random_engine generator{static_cast<unsigned int>(time(0))};

    // Physical center
    double center_x_m = (SimulatorConstants::ScreenLength / 2.0) * SimulatorConstants::MetersPerPixel;
    double center_y_m = (SimulatorConstants::ScreenLength / 2.0) * SimulatorConstants::MetersPerPixel;

    double min_radius_pixels = 100.0;
    double min_radius_m = SimulatorConstants::pixelsToMeters(min_radius_pixels);
    double max_radius_pixels = SimulatorConstants::ScreenLength / 2.5;
    double max_radius_m = SimulatorConstants::pixelsToMeters(max_radius_pixels);

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
            double density_ratio = calculateDiskDensity(radius_pixels, max_radius_pixels);
            
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

        // Base orbital velocity in m/s
        double base_velocity_m_s = calculateKeplerianVelocity(radius_meters, central_mass);

        // Add small random variation
        std::normal_distribution<> vel_variation(1.0, 0.01);
        double speed_m_s = base_velocity_m_s * vel_variation(generator);

        // Tangential velocity (in m/s)
        double vx_m_s = -speed_m_s * sin(angle);
        double vy_m_s = speed_m_s * cos(angle);

        // Radial perturbation
        std::normal_distribution<> radial_vel_dist(0.0, speed_m_s * 0.001);
        double radial_vel = radial_vel_dist(generator);
        vx_m_s += radial_vel * cos(angle);
        vy_m_s += radial_vel * sin(angle);

        // Create particle
        auto entity = registry.create();
        registry.emplace<Components::Position>(entity, x_m, y_m);
        registry.emplace<Components::Velocity>(entity, vx_m_s, vy_m_s);

        double base_mass = SimulatorConstants::ParticleMassMean;
        double mass_factor = std::pow(min_radius_m / radius_meters, 0.5);
        std::normal_distribution<> mass_variation(mass_factor * base_mass, base_mass * 0.1);
        registry.emplace<Components::Mass>(entity, mass_variation(generator));

        particles_created++;
    }
}

void ECSSimulator::tick() {
    // First calculate gravitational forces using Barnes-Hut
    Systems::BarnesHutSystem::update(registry);
    
    // Then apply local thermodynamic effects using the grid system
    Systems::GridThermodynamicsSystem::update(registry);
    
    // Finally update positions
    Systems::MovementSystem::update(registry);
} 