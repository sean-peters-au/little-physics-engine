#include <chrono>
#include <iostream>
#include <cmath>

#include "nbody/components/sim.hpp"
#include "nbody/systems/barnes_hut.hpp"
#include "nbody/core/debug.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/profile.hpp"

namespace Systems {

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;

BarnesHutSystem::BarnesHutSystem() {
    // Initialize with default configurations
}

void BarnesHutSystem::update(entt::registry& registry) {
    PROFILE_SCOPE("BarnesHutSystem");

    auto sv = registry.view<Components::SimulatorState>();
    if (sv.empty()) {
        std::cerr << "Warning: No SimulatorState found in BarnesHutSystem::update.\n";
        return; // Skip if no state
    }

    const auto& state = registry.get<Components::SimulatorState>(sv.front());
    
    auto root = buildTree(registry);
    auto afterBuild = Clock::now();
    
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
    root->boundary_size = sysConfig.UniverseSizeMeters;

    // Insert all particles except boundaries
    auto view = registry.view<Components::Position, Components::Mass>();
    int count=0;
    for (auto [entity, pos, mass] : view.each()) {
        if (registry.all_of<Components::Boundary>(entity)) { continue; }
        
        if (pos.x >= 0 && pos.x < sysConfig.UniverseSizeMeters &&
            pos.y >= 0 && pos.y < sysConfig.UniverseSizeMeters)
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
        auto oldEntity = node->particle;
        const auto& oldPos = node->registry->get<Components::Position>(oldEntity);
        const auto& oldMass = node->registry->get<Components::Mass>(oldEntity);

        subdivide(node);
        insertParticle(node, oldEntity, oldPos, oldMass);
        insertParticle(node, entity, pos, mass);
    } else {
        double const newTotalMass = node->total_mass + mass.value;
        node->center_of_mass_x = (node->center_of_mass_x * node->total_mass + pos.x * mass.value) / newTotalMass;
        node->center_of_mass_y = (node->center_of_mass_y * node->total_mass + pos.y * mass.value) / newTotalMass;
        node->total_mass = newTotalMass;
        
        int const quadrant = node->getQuadrant(pos.x, pos.y);
        QuadTreeNode* child = nullptr;
        switch (quadrant) {
            case 0: child = node->nw.get(); break;
            case 1: child = node->ne.get(); break;
            case 2: child = node->sw.get(); break;
            case 3: child = node->se.get(); break;
        }
        if (child != nullptr) {  
            insertParticle(child, entity, pos, mass);
        }
    }
}

void BarnesHutSystem::subdivide(QuadTreeNode* node) {
    double const halfSize = node->boundary_size / 2;
    node->is_leaf = false;

    node->nw = std::make_unique<QuadTreeNode>();
    node->nw->registry = node->registry;  
    node->nw->boundary_x = node->boundary_x;
    node->nw->boundary_y = node->boundary_y;
    node->nw->boundary_size = halfSize;
    
    node->ne = std::make_unique<QuadTreeNode>();
    node->ne->registry = node->registry; 
    node->ne->boundary_x = node->boundary_x + halfSize;
    node->ne->boundary_y = node->boundary_y;
    node->ne->boundary_size = halfSize;
    
    node->sw = std::make_unique<QuadTreeNode>();
    node->sw->registry = node->registry;  
    node->sw->boundary_x = node->boundary_x;
    node->sw->boundary_y = node->boundary_y + halfSize;
    node->sw->boundary_size = halfSize;
    
    node->se = std::make_unique<QuadTreeNode>();
    node->se->registry = node->registry;  
    node->se->boundary_x = node->boundary_x + halfSize;
    node->se->boundary_y = node->boundary_y + halfSize;
    node->se->boundary_size = halfSize;
}

void BarnesHutSystem::calculateForce(const QuadTreeNode* node,
                                   entt::entity entity,
                                   const Components::Position& pos,
                                   Components::Velocity& vel,
                                   const Components::Mass& mass,
                                   const Components::SimulatorState& state) {
    if ((node == nullptr) || node->total_mass == 0.0) { return; }
    
    double const dx = pos.x - node->center_of_mass_x;
    double const dy = pos.y - node->center_of_mass_y;
    double const distSq = dx*dx + dy*dy + sysConfig.GravitationalSoftener * sysConfig.GravitationalSoftener;
    double const dist = std::sqrt(distSq);

    if (node->is_leaf || (node->boundary_size * node->boundary_size / distSq < specificConfig.theta * specificConfig.theta)) {
        if (node->is_leaf && node->particle == entity) { return; }

        double const force = SimulatorConstants::RealG * node->total_mass * mass.value / distSq;
        DebugStats::updateForce(force);
        
        double const accX = force * (dx / (mass.value * dist));
        double const accY = force * (dy / (mass.value * dist));
        
        double const dt = sysConfig.SecondsPerTick * state.baseTimeAcceleration * state.timeScale;
        
        vel.x += accX * dt;
        vel.y += accY * dt;
    } else {
        calculateForce(node->nw.get(), entity, pos, vel, mass, state);
        calculateForce(node->ne.get(), entity, pos, vel, mass, state);
        calculateForce(node->sw.get(), entity, pos, vel, mass, state);
        calculateForce(node->se.get(), entity, pos, vel, mass, state);
    }
}

} // namespace Systems