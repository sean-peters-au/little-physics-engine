#include "ecs_simulator.h"
#include "simulator_constants.h"
#include "systems/movement_system.h"
#include "systems/barnes_hut_system.h"
#include <random>
#include <cmath>
#include <iostream>

ECSSimulator::ECSSimulator() {}

ECSSimulator::ECSSimulator(CoordinateSystem* /*coordSystem*/) {}

void ECSSimulator::generateParticles() {
    std::default_random_engine generator{static_cast<unsigned int>(time(0))};
    std::normal_distribution<> mass_dist(SimulatorConstants::ParticleMassMean, 
                                       SimulatorConstants::ParticleMassStdDev);
    
    // Create a massive central body
    auto center = registry.create();
    registry.emplace<Components::Position>(center, 
        SimulatorConstants::ScreenLength / 2.0, 
        SimulatorConstants::ScreenLength / 2.0);
    registry.emplace<Components::Velocity>(center, 0.0, 0.0);
    registry.emplace<Components::Mass>(center, SimulatorConstants::ParticleMassMean * 100.0);
    
    std::cout << "Central body mass: " << SimulatorConstants::ParticleMassMean * 100.0 << " kg\n";
    
    // Create orbiting particles in a spiral pattern
    std::uniform_real_distribution<> angle_dist(0, 2 * SimulatorConstants::Pi);
    std::uniform_real_distribution<> radius_dist(50, SimulatorConstants::ScreenLength / 2.5);
    
    for (int i = 0; i < SimulatorConstants::ParticleCount - 1; ++i) {
        auto entity = registry.create();
        
        double radius = radius_dist(generator);
        double angle = angle_dist(generator);
        
        // Calculate position in pixels
        double x = SimulatorConstants::ScreenLength / 2.0 + radius * cos(angle);
        double y = SimulatorConstants::ScreenLength / 2.0 + radius * sin(angle);
        
        // Convert radius to meters for orbital velocity calculation
        double radius_meters = SimulatorConstants::pixelsToMeters(radius);
        double central_mass = SimulatorConstants::ParticleMassMean * 100.0;
        
        // Calculate orbital velocity using real physics (m/s)
        double speed = std::sqrt(SimulatorConstants::RealG * central_mass / radius_meters) * 
                      SimulatorConstants::InitialVelocityFactor;
        
        // Add a small random perturbation to make it more interesting
        std::normal_distribution<> speed_variation(1.0, 0.1);
        speed *= speed_variation(generator);
        
        // Perpendicular velocity for orbital motion (in m/s)
        double vx = -sin(angle) * speed;
        double vy = cos(angle) * speed;

        if (i == 1) {  // Debug output for first orbiting particle
            std::cout << "Sample particle:\n"
                     << "  Position: (" << x << ", " << y << ") pixels\n"
                     << "  Radius: " << radius_meters << " meters\n"
                     << "  Speed: " << speed << " m/s\n"
                     << "  Velocity: (" << vx << ", " << vy << ") m/s\n"
                     << "  Mass: " << mass_dist(generator) << " kg\n";
        }
        
        registry.emplace<Components::Position>(entity, x, y);
        registry.emplace<Components::Velocity>(entity, vx, vy);
        registry.emplace<Components::Mass>(entity, mass_dist(generator));
    }
}

void ECSSimulator::init() {
    generateParticles();
}

void ECSSimulator::tick() {
    Systems::BarnesHutSystem::update(registry);
    Systems::MovementSystem::update(registry);
} 