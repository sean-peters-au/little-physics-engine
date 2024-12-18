#include <gtest/gtest.h>
#include "nbody/systems/barnes_hut.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include <iostream>

using namespace Systems;

class BarnesHutTest : public ::testing::Test {
protected:
    entt::registry registry;
    
    // Helper to create a particle
    entt::entity createParticle(double x, double y, double mass) {
        auto entity = registry.create();
        registry.emplace<Components::Position>(entity, x, y);
        registry.emplace<Components::Velocity>(entity, 0.0, 0.0);
        registry.emplace<Components::Mass>(entity, mass);
        return entity;
    }

    void SetUp() override {
        // Initialize simulation constants for celestial gas scenario
        SimulatorConstants::initializeConstants(SimulatorConstants::SimulationType::CELESTIAL_GAS);
    }
};

TEST_F(BarnesHutTest, QuadTreeNodeContains) {
    BarnesHutSystem::QuadTreeNode node;
    node.boundary_x = 0.0;
    node.boundary_y = 0.0;
    node.boundary_size = 100.0;

    // Test points inside
    EXPECT_TRUE(node.contains(0.0, 0.0));     // Corner
    EXPECT_TRUE(node.contains(50.0, 50.0));   // Center
    EXPECT_TRUE(node.contains(99.9, 99.9));   // Near far corner

    // Test points outside
    EXPECT_FALSE(node.contains(-1.0, 50.0));  // Left
    EXPECT_FALSE(node.contains(100.0, 50.0)); // Right
    EXPECT_FALSE(node.contains(50.0, -1.0));  // Top
    EXPECT_FALSE(node.contains(50.0, 100.0)); // Bottom
}

TEST_F(BarnesHutTest, QuadTreeNodeGetQuadrant) {
    BarnesHutSystem::QuadTreeNode node;
    node.boundary_x = 0.0;
    node.boundary_y = 0.0;
    node.boundary_size = 100.0;

    // Test each quadrant
    EXPECT_EQ(node.getQuadrant(25.0, 25.0), 0);  // NW
    EXPECT_EQ(node.getQuadrant(75.0, 25.0), 1);  // NE
    EXPECT_EQ(node.getQuadrant(25.0, 75.0), 2);  // SW
    EXPECT_EQ(node.getQuadrant(75.0, 75.0), 3);  // SE
}

TEST_F(BarnesHutTest, TreeConstruction) {
    // Create a simple system of 2 particles close enough to interact
    [[maybe_unused]] auto p1 = createParticle(100.0, 100.0, 1e40);  // Super heavy particle
    auto p2 = createParticle(110.0, 100.0, 1e30);   // Heavy particle

    // Run multiple updates to ensure forces accumulate
    for (int i = 0; i < 1000; i++) {  // Increased iterations
        BarnesHutSystem::update(registry);
        
        // Debug output every 100 iterations
        if (i % 100 == 0) {
            auto& vel = registry.get<Components::Velocity>(p2);
            std::cout << "Iteration " << i << ", Velocity: (" << vel.x << ", " << vel.y << ")\n";
        }
    }

    // Verify particle has moved
    auto& vel2 = registry.get<Components::Velocity>(p2);
    EXPECT_NE(vel2.x, 0.0);
}

TEST_F(BarnesHutTest, GravitationalAttraction) {
    // Create two particles with extreme mass difference
    [[maybe_unused]] auto p1 = createParticle(100.0, 100.0, 1e40);  // Super heavy particle
    auto p2 = createParticle(200.0, 100.0, 1e30);   // Heavy particle

    // Initial position
    double initial_x = 200.0;

    // Run multiple updates
    for (int i = 0; i < 1000; i++) {  // Increased iterations
        BarnesHutSystem::update(registry);
        
        // Debug output every 100 iterations
        if (i % 100 == 0) {
            auto& pos = registry.get<Components::Position>(p2);
            std::cout << "Iteration " << i << ", Position: (" << pos.x << ", " << pos.y << ")\n";
        }
    }

    // The lighter particle should have moved towards the heavier one
    auto& pos2 = registry.get<Components::Position>(p2);
    EXPECT_LT(pos2.x, initial_x);
}

TEST_F(BarnesHutTest, CenterOfMassCalculation) {
    // Create two super massive particles
    createParticle(50.0, 50.0, 1e40);
    createParticle(150.0, 150.0, 1e40);

    // Create test particle
    auto p3 = createParticle(100.0, 50.0, 1e30);
    double initial_y = 50.0;

    // Run multiple updates
    for (int i = 0; i < 1000; i++) {  // Increased iterations
        BarnesHutSystem::update(registry);
        
        // Debug output every 100 iterations
        if (i % 100 == 0) {
            auto& pos = registry.get<Components::Position>(p3);
            std::cout << "Iteration " << i << ", Position: (" << pos.x << ", " << pos.y << ")\n";
        }
    }
    
    // The test particle should be pulled towards the center
    auto& pos3 = registry.get<Components::Position>(p3);
    EXPECT_GT(pos3.y, initial_y);
}

TEST_F(BarnesHutTest, BoundaryConditions) {
    // Create a particle outside the simulation boundaries
    auto entity = createParticle(-1.0, -1.0, 1.0);

    // Run an update
    BarnesHutSystem::update(registry);

    // Verify the particle was handled gracefully (no crashes)
    EXPECT_TRUE(registry.valid(entity));
}

TEST_F(BarnesHutTest, EmptyTreeHandling) {
    // Run update on empty registry
    BarnesHutSystem::update(registry);
    
    // Should complete without errors
    SUCCEED();
} 