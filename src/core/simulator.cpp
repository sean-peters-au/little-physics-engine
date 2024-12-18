#include "nbody/core/simulator.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/systems/sleep.hpp"
#include "nbody/systems/rotation.hpp"
#include "nbody/systems/movement.hpp"
#include "nbody/systems/barnes_hut.hpp"
#include "nbody/systems/thermodynamics.hpp"
#include "nbody/systems/sph.hpp"
#include "nbody/systems/gravity.hpp"
#include "nbody/systems/sleep.hpp"

// Include the new collision pipeline headers
#include "nbody/systems/collision/broad_phase_system.hpp"
#include "nbody/systems/collision/narrow_phase_system.hpp"
#include "nbody/systems/collision/solid_collision_response_system.hpp"
#include "nbody/systems/collision/liquid_collision_response_system.hpp"
#include "nbody/systems/collision/gas_collision_response_system.hpp"

// Include collision_data.hpp which defines CandidatePair, CollisionManifold, etc.
#include "nbody/systems/collision/collision_data.hpp"

#include "nbody/components/basic.hpp"
#include <nbody/math/polygon.hpp>

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
    // Store current state entity and component if it exists
    Components::SimulatorState savedState;  // Use default constructor
    savedState.timeScale = 1.0;
    savedState.baseTimeAcceleration = SimulatorConstants::TimeAcceleration;
    
    auto stateView = registry.view<Components::SimulatorState>();
    if (!stateView.empty()) {
        savedState = registry.get<Components::SimulatorState>(stateView.front());
    }

    // Clear registry and reinitialize constants
    registry.clear();
    SimulatorConstants::initializeConstants(currentScenario);

    // Recreate state entity with saved state
    auto stateEntity = registry.create();
    registry.emplace<Components::SimulatorState>(stateEntity, savedState);

    // Initialize scenario
    init();
}

void ECSSimulator::init() {
    // Don't clear registry here anymore since reset() handles it
    
    // Initialize scenario
    switch (currentScenario) {
        case SimulatorConstants::SimulationType::CELESTIAL_GAS:
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
            case SimulatorConstants::ECSSystem::COLLISION: {
                int solverIterations = 5; // try multiple passes
                for (int i = 0; i < solverIterations; i++) {
                    std::vector<CandidatePair> candidatePairs;
                    Systems::BroadPhaseSystem::update(registry, candidatePairs);
                    
                    CollisionManifold manifold;
                    Systems::NarrowPhaseSystem::update(registry, candidatePairs, manifold);
                    
                    if (manifold.collisions.empty()) {
                        // No collisions to solve, break early
                        break;
                    }

                    // Lower restitution further here if needed by scenario
                    // E.g., no direct code here, but you can tweak restitution in systems

                    // Run collision responses
                    Systems::SolidCollisionResponseSystem::update(registry, manifold);
                    Systems::LiquidCollisionResponseSystem::update(registry, manifold);
                    Systems::GasCollisionResponseSystem::update(registry, manifold);
                }
                break;
            }
            case SimulatorConstants::ECSSystem::ROTATION:
                Systems::RotationSystem::update(registry);
                break;
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
            case SimulatorConstants::ECSSystem::SLEEP:
                Systems::SleepSystem::update(registry);
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
    // Default shape for central body (just make it large circle)
    registry.emplace<Components::Shape>(center, Components::ShapeType::Circle, 1e7);

    // Add bright yellow color for central body
    registry.emplace<Components::Color>(center, 255, 255, 0);
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

        // For disk particles: just assign them as small circles
        registry.emplace<Components::Shape>(entity, Components::ShapeType::Circle, SimulatorConstants::MetersPerPixel * 0.5);

        // Add blue-white color for gas particles
        registry.emplace<Components::Color>(entity, 150, 150, 255);

        particles_created++;
    }
}

void ECSSimulator::createBouncyBalls() {
    std::default_random_engine generator{static_cast<unsigned int>(time(0))};
    
    int balls_per_side = static_cast<int>(std::sqrt(SimulatorConstants::ParticleCount));
    double spacing = SimulatorConstants::UniverseSizeMeters / (balls_per_side + 1);
    
    std::normal_distribution<> velocity_dist(0.0, 1.0);
    std::normal_distribution<> mass_dist(SimulatorConstants::ParticleMassMean, SimulatorConstants::ParticleMassStdDev);

    // Random size distribution
    std::uniform_real_distribution<> size_dist(0.05, 0.2);
    // Random shape distribution
    std::uniform_real_distribution<> shape_dist(0.0, 1.0);

    // Random color distribution (ensuring minimum brightness)
    std::uniform_int_distribution<> color_dist(100, 255);  // Minimum 100 to avoid too dark colors

    int particles_created = 0;
    for (int i = 0; i < balls_per_side && particles_created < SimulatorConstants::ParticleCount; i++) {
        for (int j = 0; j < balls_per_side && particles_created < SimulatorConstants::ParticleCount; j++) {
            double x_m = (i + 1) * spacing;
            double y_m = (j + 1) * spacing;

            auto entity = registry.create();
            double mass_val = mass_dist(generator);

            registry.emplace<Components::Position>(entity, x_m, y_m);
            registry.emplace<Components::Velocity>(entity, velocity_dist(generator), velocity_dist(generator));
            registry.emplace<Components::Mass>(entity, mass_val);
            registry.emplace<Components::ParticlePhase>(entity, Components::Phase::Solid);
            registry.emplace<Components::Sleep>(entity);

            double s = shape_dist(generator);
            double sz = size_dist(generator);

            double I; // moment of inertia
            if (s < 0.5)
            {
                // Circle
                registry.emplace<Components::Shape>(entity, Components::ShapeType::Circle, sz);
                registry.emplace<CircleShape>(entity, sz); // Now we have a circle radius stored
                I = 0.5 * mass_val * (sz * sz); // circle inertia
            }
            else
            {
                // Square
                registry.emplace<Components::Shape>(entity, Components::ShapeType::Square, sz);
                PolygonShape poly;
                poly.type = Components::ShapeType::Square;
                poly.vertices = {
                    Vector(-sz, -sz),
                    Vector(sz, -sz),
                    Vector(sz, sz),
                    Vector(-sz, sz)};
                registry.emplace<PolygonShape>(entity, poly);
                I = (4.0 / 3.0) * mass_val * (sz * sz); // square inertia
            }
            // Add angular components
            registry.emplace<Components::AngularPosition>(entity, 0.0); // start angle = 0
            registry.emplace<Components::AngularVelocity>(entity, 0.0);
            registry.emplace<Components::Inertia>(entity, I);

            // Add random color
            registry.emplace<Components::Color>(entity, 
                color_dist(generator),  // r
                color_dist(generator),  // g
                color_dist(generator)   // b
            );

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
                registry.emplace<Components::Shape>(entity, Components::ShapeType::Circle, 0.5);

                // Add cyan-ish color for gas particles
                registry.emplace<Components::Color>(entity, 0, 200, 200);

                particles++;
                if (particles >= SimulatorConstants::ParticleCount) return;
            }
        }
    }
}

void ECSSimulator::createBoundaries() {
    double size = SimulatorConstants::UniverseSizeMeters;

    auto createWall = [&](double x, double y, double w, double h) {
        auto wall = registry.create();
        registry.emplace<Components::Position>(wall, x, y);
        registry.emplace<Components::Mass>(wall, 1e30); // effectively infinite mass
        registry.emplace<Components::ParticlePhase>(wall, Components::Phase::Solid);

        PolygonShape poly;
        poly.type = Components::ShapeType::Square;
        // Create a rectangle centered at (x,y) of width w and height h
        // local coords:
        double halfW = w*0.5;
        double halfH = h*0.5;
        poly.vertices = {
            Vector(-halfW, -halfH),
            Vector( halfW, -halfH),
            Vector( halfW,  halfH),
            Vector(-halfW,  halfH)
        };
        registry.emplace<PolygonShape>(wall, poly);
        registry.emplace<Components::Shape>(wall, Components::ShapeType::Square, std::max(w,h)*0.5);

        // Infinite inertia for a static body isn't needed, no AngularVelocity or Inertia required
        // Just don't add those for walls.
    };

    // Create left wall
    createWall(0.0, size*0.5, 1.0, size); // Very thin vertical wall on left
    // Right wall
    createWall(size, size*0.5, 1.0, size);
    // Bottom wall
    createWall(size*0.5, 0.0, size, 1.0);
    // Top wall
    createWall(size*0.5, size, size, 1.0);
}
