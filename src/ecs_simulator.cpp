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
    double center_x = SimulatorConstants::ScreenLength / 2.0;
    double center_y = SimulatorConstants::ScreenLength / 2.0;
    
    registry.emplace<Components::Position>(center, center_x, center_y);
    registry.emplace<Components::Velocity>(center, 0.0, 0.0);
    registry.emplace<Components::Mass>(center, SimulatorConstants::ParticleMassMean * 100.0);
    
    std::cout << "Central body mass: " << SimulatorConstants::ParticleMassMean * 100.0 << " kg\n";
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
    
    const double center_x = SimulatorConstants::ScreenLength / 2.0;
    const double center_y = SimulatorConstants::ScreenLength / 2.0;
    const double min_radius = 100.0;  // pixels
    const double max_radius = SimulatorConstants::ScreenLength / 2.5;
    const double central_mass = SimulatorConstants::ParticleMassMean * 100.0;
    
    // Debug initial conditions
    std::cout << "\nInitial conditions:\n"
              << "  MetersPerPixel: " << SimulatorConstants::MetersPerPixel << "\n"
              << "  TimeAcceleration: " << SimulatorConstants::TimeAcceleration << " seconds/tick\n"
              << "  SecondsPerTick: " << SimulatorConstants::SecondsPerTick << "\n"
              << "  Inner radius: " << min_radius << " pixels = " 
              << SimulatorConstants::pixelsToMeters(min_radius) << " meters\n";
    
    // Create distribution for particle placement
    std::uniform_real_distribution<> angle_dist(0, 2 * SimulatorConstants::Pi);
    
    // Create particles
    int particles_created = 0;
    while (particles_created < SimulatorConstants::ParticleCount - 1) {
        // Use rejection sampling to create proper density distribution
        double radius;
        double density_threshold;
        do {
            std::uniform_real_distribution<> radius_dist(min_radius, max_radius);
            radius = radius_dist(generator);
            
            // Calculate normalized density for rejection sampling
            double density_ratio = calculateDiskDensity(radius, max_radius);
            
            // Random threshold for rejection sampling
            std::uniform_real_distribution<> density_dist(0, 1);
            density_threshold = density_dist(generator);
        } while (density_threshold > calculateDiskDensity(radius, max_radius));
        
        // Generate angle and height
        double angle = angle_dist(generator);
        double max_height = calculateDiskHeight(radius, max_radius);
        std::normal_distribution<> height_dist(0.0, max_height/3.0);  // 3σ within max height
        double height_offset = height_dist(generator);
        
        // Calculate position
        double x = center_x + radius * cos(angle);
        double y = center_y + radius * sin(angle) + height_offset;
        
        // Skip if outside screen bounds
        if (x < 0 || x >= SimulatorConstants::ScreenLength ||
            y < 0 || y >= SimulatorConstants::ScreenLength) {
            continue;
        }
        
        // Calculate Keplerian orbital velocity in m/s
        double radius_meters = SimulatorConstants::pixelsToMeters(radius);
        double base_velocity = calculateKeplerianVelocity(radius_meters, central_mass);
        
        // Add small random variations to velocity (1% variation)
        std::normal_distribution<> vel_variation(1.0, 0.01);
        double speed = base_velocity * vel_variation(generator);
        
        // Convert from m/s to pixels/tick:
        // 1. Convert speed from m/s to pixels/s
        double pixels_per_second = SimulatorConstants::metersToPixels(speed);
        // 2. Multiply by time_factor to get pixels/tick
        double time_factor = SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration;
        double pixels_per_tick = pixels_per_second * time_factor;
        
        // Tangential velocity for circular orbit
        double vx = -pixels_per_tick * sin(angle);  // Negative for counterclockwise rotation
        double vy = pixels_per_tick * cos(angle);
        
        // Add small radial velocity component (0.1% of orbital velocity)
        std::normal_distribution<> radial_vel_dist(0.0, pixels_per_tick * 0.001);
        double radial_vel = radial_vel_dist(generator);
        vx += radial_vel * cos(angle);
        vy += radial_vel * sin(angle);
        
        // Create particle
        auto entity = registry.create();
        registry.emplace<Components::Position>(entity, x, y);
        registry.emplace<Components::Velocity>(entity, vx, vy);  // Now in pixels/tick
        
        // Mass varies with radius (heavier particles tend to sink inward)
        double base_mass = SimulatorConstants::ParticleMassMean;
        double mass_factor = std::pow(min_radius / radius, 1.0/2.0);  // Lighter at outer radii
        std::normal_distribution<> mass_variation(mass_factor * base_mass, base_mass * 0.1);
        registry.emplace<Components::Mass>(entity, mass_variation(generator));
        
        if (particles_created == 0) {  // Debug output for first particle
            double orbital_period = 2 * SimulatorConstants::Pi * std::sqrt(std::pow(radius_meters, 3) / 
                                                                         (SimulatorConstants::RealG * central_mass));
            double expected_pixels_per_tick = (2 * SimulatorConstants::Pi * radius) / (orbital_period / time_factor);
            
            std::cout << "\nFirst particle details:\n"
                     << "  Position: (" << x << ", " << y << ") pixels\n"
                     << "  Radius: " << radius << " pixels = " << radius_meters << " meters\n"
                     << "  Orbital velocity: " << base_velocity << " m/s\n"
                     << "  Conversion steps:\n"
                     << "    1. Speed in pixels/s: " << pixels_per_second << "\n"
                     << "    2. Time factor: " << time_factor << " seconds/tick\n"
                     << "    3. Final speed: " << pixels_per_tick << " pixels/tick\n"
                     << "  Actual velocity: (" << vx << ", " << vy << ") pixels/tick\n"
                     << "  Expected speed: " << expected_pixels_per_tick << " pixels/tick\n"
                     << "  Orbital period: " << orbital_period << " seconds\n"
                     << "  Mass: " << mass_factor * base_mass << " kg\n";
        }
        
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