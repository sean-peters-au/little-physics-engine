#include <chrono>
#include <iostream>
#include <cmath>

#include "nbody/components/sim.hpp"
#include "nbody/systems/barnes_hut.hpp"
#include "nbody/core/debug.hpp"
#include "nbody/components/basic.hpp"

namespace Systems {

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;

void BarnesHutSystem::update(entt::registry& registry) {
    TimePoint start_time = Clock::now();

    auto sv = registry.view<Components::SimulatorState>();
    if (sv.empty()) {
        std::cerr << "Warning: No SimulatorState found in BarnesHutSystem::update.\n";
        return; // Skip if no state
    }

    const auto& state = registry.get<Components::SimulatorState>(sv.front());
    
    auto root = buildTree(registry);
    auto after_build = Clock::now();
    
    auto view = registry.view<Components::Position, Components::Velocity, Components::Mass>();
    for (auto [entity, pos, vel, mass] : view.each()) {
        // If boundary, skip
        if (registry.all_of<Components::Boundary>(entity)) {
            continue;
        }
        calculateForce(root.get(), entity, pos, vel, mass, state);
    }
}

std::unique_ptr<BarnesHutSystem::QuadTreeNode> BarnesHutSystem::buildTree(const entt::registry& registry) {
    auto root = std::make_unique<QuadTreeNode>();
    root->registry = &registry; 
    root->boundary_x = 0.0;
    root->boundary_y = 0.0;
    root->boundary_size = SimulatorConstants::UniverseSizeMeters;

    // Insert all particles except boundaries
    auto view = registry.view<Components::Position, Components::Mass>();
    int count=0;
    for (auto [entity, pos, mass] : view.each()) {
        if (registry.all_of<Components::Boundary>(entity)) continue;
        if (pos.x >= 0 && pos.x < SimulatorConstants::UniverseSizeMeters &&
            pos.y >= 0 && pos.y < SimulatorConstants::UniverseSizeMeters)
        {
            insertParticle(root.get(), entity, pos, mass);
            count++;
        }
    }
    return root;
}

void BarnesHutSystem::insertParticle(QuadTreeNode* node, entt::entity entity,
                                   const Components::Position& pos,
                                   const Components::Mass& mass) {
    if (!node->contains(pos.x, pos.y)) {
        return;  
    }
    
    if (node->total_mass == 0.0) {
        node->total_mass = mass.value;
        node->center_of_mass_x = pos.x;
        node->center_of_mass_y = pos.y;
        node->particle = entity;
        return;
    }

    if (node->is_leaf) {
        auto old_entity = node->particle;
        auto& old_pos = node->registry->get<Components::Position>(old_entity);
        auto& old_mass = node->registry->get<Components::Mass>(old_entity);

        subdivide(node);
        insertParticle(node, old_entity, old_pos, old_mass);
        insertParticle(node, entity, pos, mass);
    } else {
        double new_total_mass = node->total_mass + mass.value;
        node->center_of_mass_x = (node->center_of_mass_x * node->total_mass + pos.x * mass.value) / new_total_mass;
        node->center_of_mass_y = (node->center_of_mass_y * node->total_mass + pos.y * mass.value) / new_total_mass;
        node->total_mass = new_total_mass;
        
        int quadrant = node->getQuadrant(pos.x, pos.y);
        QuadTreeNode* child = nullptr;
        switch (quadrant) {
            case 0: child = node->nw.get(); break;
            case 1: child = node->ne.get(); break;
            case 2: child = node->sw.get(); break;
            case 3: child = node->se.get(); break;
        }
        if (child) {  
            insertParticle(child, entity, pos, mass);
        }
    }
}

void BarnesHutSystem::subdivide(QuadTreeNode* node) {
    double half_size = node->boundary_size / 2;
    node->is_leaf = false;

    node->nw = std::make_unique<QuadTreeNode>();
    node->nw->registry = node->registry;  
    node->nw->boundary_x = node->boundary_x;
    node->nw->boundary_y = node->boundary_y;
    node->nw->boundary_size = half_size;
    
    node->ne = std::make_unique<QuadTreeNode>();
    node->ne->registry = node->registry; 
    node->ne->boundary_x = node->boundary_x + half_size;
    node->ne->boundary_y = node->boundary_y;
    node->ne->boundary_size = half_size;
    
    node->sw = std::make_unique<QuadTreeNode>();
    node->sw->registry = node->registry;  
    node->sw->boundary_x = node->boundary_x;
    node->sw->boundary_y = node->boundary_y + half_size;
    node->sw->boundary_size = half_size;
    
    node->se = std::make_unique<QuadTreeNode>();
    node->se->registry = node->registry;  
    node->se->boundary_x = node->boundary_x + half_size;
    node->se->boundary_y = node->boundary_y + half_size;
    node->se->boundary_size = half_size;
}

void BarnesHutSystem::calculateForce(const QuadTreeNode* node,
                                   entt::entity entity,
                                   const Components::Position& pos,
                                   Components::Velocity& vel,
                                   const Components::Mass& mass,
                                   const Components::SimulatorState& state) {
    if (!node || node->total_mass == 0.0) return;
    
    double dx = pos.x - node->center_of_mass_x;
    double dy = pos.y - node->center_of_mass_y;
    double dist_sq = dx*dx + dy*dy + SimulatorConstants::GravitationalSoftener * SimulatorConstants::GravitationalSoftener;
    double dist = std::sqrt(dist_sq);

    if (node->is_leaf || (node->boundary_size * node->boundary_size / dist_sq < THETA * THETA)) {
        if (node->is_leaf && node->particle == entity) return;

        double force = SimulatorConstants::RealG * node->total_mass * mass.value / dist_sq;
        DebugStats::updateForce(force);
        
        double acc_x = force * (dx / (mass.value * dist));
        double acc_y = force * (dy / (mass.value * dist));
        
        double dt = SimulatorConstants::SecondsPerTick * state.baseTimeAcceleration * state.timeScale;
        
        vel.x += acc_x * dt;
        vel.y += acc_y * dt;
    } else {
        calculateForce(node->nw.get(), entity, pos, vel, mass, state);
        calculateForce(node->ne.get(), entity, pos, vel, mass, state);
        calculateForce(node->sw.get(), entity, pos, vel, mass, state);
        calculateForce(node->se.get(), entity, pos, vel, mass, state);
    }
}

} // namespace Systems