/**
 * @fileoverview
 * @brief Barnes-Hut algorithm implementation for gravitational N-body simulation.
 *
 * - Builds a quadtree of all non-boundary entities with mass and position.
 * - Each node stores total mass, center of mass, and a flag tracking if all
 *   masses in that node are below a "small mass" threshold (optional).
 * - During force calculation, large distant nodes are approximated as a point,
 *   and nodes composed solely of "small" particles may be skipped entirely.
 */

#include "systems/barnes_hut.hpp"

#include <cmath>
#include <chrono>
#include <iostream>

#include "core/constants.hpp"
#include "core/debug.hpp"
#include "core/profile.hpp"

#include "entities/entity_components.hpp"
#include "entities/sim_components.hpp"

namespace Systems {

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;

BarnesHutSystem::BarnesHutSystem() {
    // No special setup; rely on default config
}

void BarnesHutSystem::update(entt::registry& registry) {
    PROFILE_SCOPE("BarnesHutSystem");

    // Fetch the simulation state (time scaling, etc.)
    auto stateView = registry.view<Components::SimulatorState>();
    if (stateView.empty()) {
        std::cerr << "[BarnesHut] Warning: No SimulatorState found. Skipping update.\n";
        return;
    }
    const auto& simState = stateView.get<Components::SimulatorState>(stateView.front());

    // Build the quadtree
    auto root = buildTree(registry);
    auto afterBuild = Clock::now();  // Potentially used for profiling

    // Prepare to iterate all non-boundary bodies that we want to apply forces to
    // This improves performance vs. .each() by skipping boundaries at the view level.
    auto bodyView = registry.view<Components::Position, Components::Velocity, Components::Mass>(entt::exclude<Components::Boundary>);

    // For each entity in the view, compute gravitational force from the quadtree
    for (auto entity : bodyView) {
        auto& pos = bodyView.get<Components::Position>(entity);
        auto& vel = bodyView.get<Components::Velocity>(entity);
        auto& mass = bodyView.get<Components::Mass>(entity);

        calculateForce(root.get(), entity, pos, vel, mass, simState);
    }
}

std::unique_ptr<BarnesHutSystem::QuadTreeNode> BarnesHutSystem::buildTree(const entt::registry& registry) {
    // Create the root node
    auto root = std::make_unique<QuadTreeNode>();
    root->registry = &registry;
    root->boundaryX = 0.0;
    root->boundaryY = 0.0;
    root->boundarySize = sysConfig.UniverseSizeMeters;
    // allSmall defaults to true until we find a mass >= config.smallMassThreshold

    // Insert all relevant particles into the quadtree
    // We'll skip boundaries at the view level and also skip out-of-bounds coords
    auto insertView = registry.view<Components::Position, Components::Mass>(entt::exclude<Components::Boundary>);
    for (auto entity : insertView) {
        const auto& pos = insertView.get<Components::Position>(entity);
        const auto& mass = insertView.get<Components::Mass>(entity);

        // Only insert if inside Universe bounds
        if (pos.x >= 0.0 && pos.x < sysConfig.UniverseSizeMeters &&
            pos.y >= 0.0 && pos.y < sysConfig.UniverseSizeMeters)
        {
            insertParticle(root.get(), entity, pos, mass);
        }
    }

    return root;
}

void BarnesHutSystem::insertParticle(QuadTreeNode* node,
                                     entt::entity entity,
                                     const Components::Position& pos,
                                     const Components::Mass& mass)
{
    // If position not contained, do nothing
    if (!node->contains(pos.x, pos.y)) {
        return;
    }

    // If node is empty (totalMass == 0) => store first entity
    if (node->totalMass == 0.0) {
        node->totalMass = mass.value;
        node->centerOfMassX = pos.x;
        node->centerOfMassY = pos.y;
        node->singleParticle = entity;
        // Check if this mass is >= threshold
        if (mass.value >= specificConfig.smallMassThreshold) {
            node->allSmall = false;
        }
        return;
    }

    // If this node is currently a leaf, we must subdivide to store both
    if (node->isLeaf) {
        auto oldEntity = node->singleParticle;
        const auto& oldPos = node->registry->get<Components::Position>(oldEntity);
        const auto& oldMass = node->registry->get<Components::Mass>(oldEntity);

        subdivide(node);

        // Re-insert the old occupant
        insertParticle(node, oldEntity, oldPos, oldMass);

        // Also insert the new occupant
        insertParticle(node, entity, pos, mass);
    }
    else {
        // We are an internal node: update total mass & COM
        double newTotal = node->totalMass + mass.value;
        node->centerOfMassX =
            (node->centerOfMassX * node->totalMass + pos.x * mass.value) / newTotal;
        node->centerOfMassY =
            (node->centerOfMassY * node->totalMass + pos.y * mass.value) / newTotal;
        node->totalMass = newTotal;

        // If we see a mass >= threshold, entire node is not "allSmall"
        if (mass.value >= specificConfig.smallMassThreshold) {
            node->allSmall = false;
        }

        // Insert into correct child quadrant
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
    node->isLeaf = false;

    double halfSize = node->boundarySize * 0.5;
    double x = node->boundaryX;
    double y = node->boundaryY;

    node->nw = std::make_unique<QuadTreeNode>();
    node->nw->registry = node->registry;
    node->nw->boundaryX = x;
    node->nw->boundaryY = y;
    node->nw->boundarySize = halfSize;

    node->ne = std::make_unique<QuadTreeNode>();
    node->ne->registry = node->registry;
    node->ne->boundaryX = x + halfSize;
    node->ne->boundaryY = y;
    node->ne->boundarySize = halfSize;

    node->sw = std::make_unique<QuadTreeNode>();
    node->sw->registry = node->registry;
    node->sw->boundaryX = x;
    node->sw->boundaryY = y + halfSize;
    node->sw->boundarySize = halfSize;

    node->se = std::make_unique<QuadTreeNode>();
    node->se->registry = node->registry;
    node->se->boundaryX = x + halfSize;
    node->se->boundaryY = y + halfSize;
    node->se->boundarySize = halfSize;
}

void BarnesHutSystem::calculateForce(const QuadTreeNode* node,
                                     entt::entity entity,
                                     const Components::Position& pos,
                                     Components::Velocity& vel,
                                     const Components::Mass& mass,
                                     const Components::SimulatorState& state)
{
    // If node is null or empty, no contribution
    if (!node || node->totalMass == 0.0) {
        return;
    }

    // If the entire node is "small" mass, skip it for performance
    if (node->allSmall && (specificConfig.smallMassThreshold > 0.0)) {
        return;
    }

    double dx = node->centerOfMassX - pos.x;
    double dy = node->centerOfMassY - pos.y;

    // Add gravitational softener to avoid singularities
    double distSq = dx * dx + dy * dy + sysConfig.GravitationalSoftener * sysConfig.GravitationalSoftener;
    double dist = std::sqrt(distSq);

    // Check Barnes-Hut criterion: if node is sufficiently far away or is a leaf
    // => approximate by the node's center of mass
    double sizeSq = node->boundarySize * node->boundarySize;
    double thetaSq = specificConfig.theta * specificConfig.theta;

    bool useApprox = node->isLeaf || (sizeSq / distSq < thetaSq);
    if (useApprox) {
        // If node is a leaf that only has the same entity, skip
        if (node->isLeaf && node->singleParticle == entity) {
            return;
        }

        // Calculate gravitational force
        double force = SimulatorConstants::RealG * node->totalMass * mass.value / distSq;
        DebugStats::updateForce(force);

        double invDistMass = force / (mass.value * dist);
        double accX = dx * invDistMass;
        double accY = dy * invDistMass;

        double dt = sysConfig.SecondsPerTick * state.baseTimeAcceleration * state.timeScale;
        vel.x += accX * dt;
        vel.y += accY * dt;
    } else {
        // Otherwise, recurse into children
        calculateForce(node->nw.get(), entity, pos, vel, mass, state);
        calculateForce(node->ne.get(), entity, pos, vel, mass, state);
        calculateForce(node->sw.get(), entity, pos, vel, mass, state);
        calculateForce(node->se.get(), entity, pos, vel, mass, state);
    }
}

}  // namespace Systems