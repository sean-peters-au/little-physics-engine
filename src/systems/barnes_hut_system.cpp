#include "systems/barnes_hut_system.h"
#include <chrono>
#include <iostream>
#include <cmath>

namespace Systems {

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;

void BarnesHutSystem::update(entt::registry& registry) {
    TimePoint start_time = Clock::now();
    
    // Build the quadtree
    auto root = buildTree(registry);
    auto after_build = Clock::now();
    
    // Calculate forces for each particle
    auto view = registry.view<Components::Position, Components::Velocity, Components::Mass>();
    for (auto [entity, pos, vel, mass] : view.each()) {
        calculateForce(root.get(), entity, pos, vel, mass);
    }
    auto after_forces = Clock::now();

    // Print timing every 60 frames
    static int frame_count = 0;
    frame_count++;
    if (frame_count % 60 == 0) {
        std::cout << "Barnes-Hut timing breakdown (ms):\n"
                  << "  Build tree: " << std::chrono::duration_cast<std::chrono::milliseconds>(after_build - start_time).count() << "\n"
                  << "  Calculate forces: " << std::chrono::duration_cast<std::chrono::milliseconds>(after_forces - after_build).count() << "\n"
                  << "  Total: " << std::chrono::duration_cast<std::chrono::milliseconds>(after_forces - start_time).count() << "\n";
    }
}

std::unique_ptr<BarnesHutSystem::QuadTreeNode> BarnesHutSystem::buildTree(const entt::registry& registry) {
    // Create root node covering the entire screen
    auto root = std::make_unique<QuadTreeNode>();
    root->registry = &registry;  // Store registry pointer
    root->boundary_x = 0;
    root->boundary_y = 0;
    root->boundary_size = SimulatorConstants::ScreenLength;
    
    // Insert all particles
    auto view = registry.view<Components::Position, Components::Mass>();
    for (auto [entity, pos, mass] : view.each()) {
        if (pos.x >= 0 && pos.x < SimulatorConstants::ScreenLength &&
            pos.y >= 0 && pos.y < SimulatorConstants::ScreenLength) {
            insertParticle(root.get(), entity, pos, mass);
        }
    }
    
    return root;
}

void BarnesHutSystem::insertParticle(QuadTreeNode* node, entt::entity entity,
                                   const Components::Position& pos,
                                   const Components::Mass& mass) {
    if (!node->contains(pos.x, pos.y)) {
        return;  // Particle is outside this node's boundary
    }
    
    // If node is empty, store the particle here
    if (node->total_mass == 0.0) {
        node->total_mass = mass.value;
        node->center_of_mass_x = SimulatorConstants::pixelsToMeters(pos.x);
        node->center_of_mass_y = SimulatorConstants::pixelsToMeters(pos.y);
        node->particle = entity;
        return;
    }
    
    // If this is a leaf node with a particle, subdivide it
    if (node->is_leaf) {
        // Store the existing particle's data
        auto old_entity = node->particle;
        auto& old_pos = node->registry->get<Components::Position>(old_entity);
        auto& old_mass = node->registry->get<Components::Mass>(old_entity);
        
        // Subdivide and reinsert both particles
        subdivide(node);
        insertParticle(node, old_entity, old_pos, old_mass);
        insertParticle(node, entity, pos, mass);
    } else {
        // Update center of mass
        double new_total_mass = node->total_mass + mass.value;
        node->center_of_mass_x = (node->center_of_mass_x * node->total_mass + 
                                SimulatorConstants::pixelsToMeters(pos.x) * mass.value) / new_total_mass;
        node->center_of_mass_y = (node->center_of_mass_y * node->total_mass + 
                                SimulatorConstants::pixelsToMeters(pos.y) * mass.value) / new_total_mass;
        node->total_mass = new_total_mass;
        
        // Insert into appropriate quadrant
        int quadrant = node->getQuadrant(pos.x, pos.y);
        QuadTreeNode* child = nullptr;
        switch (quadrant) {
            case 0: child = node->nw.get(); break;
            case 1: child = node->ne.get(); break;
            case 2: child = node->sw.get(); break;
            case 3: child = node->se.get(); break;
        }
        if (child) {  // Only insert if child exists
            insertParticle(child, entity, pos, mass);
        }
    }
}

void BarnesHutSystem::subdivide(QuadTreeNode* node) {
    double half_size = node->boundary_size / 2;
    node->is_leaf = false;
    
    // Create the four children
    node->nw = std::make_unique<QuadTreeNode>();
    node->nw->registry = node->registry;  // Pass registry pointer to child
    node->nw->boundary_x = node->boundary_x;
    node->nw->boundary_y = node->boundary_y;
    node->nw->boundary_size = half_size;
    
    node->ne = std::make_unique<QuadTreeNode>();
    node->ne->registry = node->registry;  // Pass registry pointer to child
    node->ne->boundary_x = node->boundary_x + half_size;
    node->ne->boundary_y = node->boundary_y;
    node->ne->boundary_size = half_size;
    
    node->sw = std::make_unique<QuadTreeNode>();
    node->sw->registry = node->registry;  // Pass registry pointer to child
    node->sw->boundary_x = node->boundary_x;
    node->sw->boundary_y = node->boundary_y + half_size;
    node->sw->boundary_size = half_size;
    
    node->se = std::make_unique<QuadTreeNode>();
    node->se->registry = node->registry;  // Pass registry pointer to child
    node->se->boundary_x = node->boundary_x + half_size;
    node->se->boundary_y = node->boundary_y + half_size;
    node->se->boundary_size = half_size;
}

void BarnesHutSystem::calculateForce(const QuadTreeNode* node,
                                   entt::entity entity,
                                   const Components::Position& pos,
                                   Components::Velocity& vel,
                                   const Components::Mass& mass) {
    if (!node || node->total_mass == 0.0) return;  // Added null check
    
    // Convert position to meters for force calculation
    double pos_x_meters = SimulatorConstants::pixelsToMeters(pos.x);
    double pos_y_meters = SimulatorConstants::pixelsToMeters(pos.y);
    
    // Calculate distance to center of mass
    double dx = node->center_of_mass_x - pos_x_meters;
    double dy = node->center_of_mass_y - pos_y_meters;
    double dist_sq = dx*dx + dy*dy + SimulatorConstants::GravitationalSoftener * SimulatorConstants::GravitationalSoftener;
    
    // If this is a leaf node or the node is far enough away (using Barnes-Hut criterion)
    if (node->is_leaf || (node->boundary_size * node->boundary_size / dist_sq < THETA * THETA)) {
        // Skip self-interaction
        if (node->is_leaf && node->particle == entity) return;
        
        // Calculate gravitational force using real units
        double force = SimulatorConstants::RealG * node->total_mass / (dist_sq * std::sqrt(dist_sq));
        
        // Convert acceleration back to pixels/second²
        double acc_x = force * dx;
        double acc_y = force * dy;
        
        // Apply time acceleration and simulation tick time
        double time_factor = SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration;
        vel.x += SimulatorConstants::metersToPixels(acc_x * time_factor);
        vel.y += SimulatorConstants::metersToPixels(acc_y * time_factor);
    } else {
        // Recursively calculate forces from children
        calculateForce(node->nw.get(), entity, pos, vel, mass);
        calculateForce(node->ne.get(), entity, pos, vel, mass);
        calculateForce(node->sw.get(), entity, pos, vel, mass);
        calculateForce(node->se.get(), entity, pos, vel, mass);
    }
}

} 