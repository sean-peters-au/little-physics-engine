/**
 * @file position_solver.cpp
 * @brief Position-based constraint solver for fluid-solid boundary conditions
 */

#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

#include <entt/entt.hpp>

#include "nbody/core/profile.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/math/vector_math.hpp"
#include "nbody/math/polygon.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sph.hpp"
#include "nbody/components/sim.hpp"
#include "nbody/systems/rigid_fluid/position_solver.hpp"
#include "nbody/systems/rigid_fluid/utils.hpp"

namespace Systems {

// Config parameters for position solver
namespace FluidRigidConfig {
    constexpr double SafetyMargin = 0.05;     // Safety margin to push particles
    constexpr double MinPushDistance = 0.01;   // Minimum push distance
    constexpr double RelaxationFactor = 0.3;   // Relaxation to spread corrections over frames
}

// Main update function
void RigidFluidPositionSolver::update(entt::registry& registry) {
    PROFILE_SCOPE("RigidFluidPositionSolver::update");
    
    // Collect entities using shared utilities
    auto fluidParticles = RigidFluidUtils::collectFluidParticles(registry);
    auto rigidBodies = RigidFluidUtils::collectRigidBodies(registry);
    
    // Early exit if no work
    if (fluidParticles.empty() || rigidBodies.empty()) {
        return;
    }
    
    // Process each fluid particle
    for (auto& fluid : fluidParticles) {
        Vector fluidPos(fluid.pos);
        Vector totalCorrection(0, 0);
        bool needsCorrection = false;
        
        // Check against each rigid body
        for (const auto& rigid : rigidBodies) {
            // Circle collision
            if (rigid.shape.type == Components::ShapeType::Circle) {
                double distance = (fluidPos - Vector(*rigid.pos)).length();
                double minDistance = rigid.shape.size + fluid.radius;
                
                if (distance < minDistance) {
                    // Calculate push direction (away from circle center)
                    Vector pushDir = fluidPos - Vector(*rigid.pos);
                    
                    if (pushDir.length() < 1e-6) {
                        // If exactly at center, push in a default direction
                        pushDir = Vector(0, -1);
                    } else {
                        pushDir = pushDir.normalized();
                    }
                    
                    // Calculate push distance
                    double pushDistance = minDistance - distance + FluidRigidConfig::SafetyMargin;
                    pushDistance = std::max(pushDistance, FluidRigidConfig::MinPushDistance);
                    
                    Vector correction = pushDir * pushDistance * FluidRigidConfig::RelaxationFactor;
                    totalCorrection += correction;
                    needsCorrection = true;
                }
            }
            // Polygon collision
            else if (rigid.shape.type == Components::ShapeType::Polygon && rigid.polygon) {
                // Transform polygon to world space
                auto worldPoly = RigidFluidUtils::getWorldSpacePolygon(*rigid.polygon, 
                                                                     Vector(*rigid.pos), 
                                                                     rigid.angPos->angle);
                
                // Check if fluid particle is inside polygon
                if (RigidFluidUtils::isPointInPolygon(fluidPos, worldPoly)) {
                    // Find closest point on polygon boundary
                    Vector closestBoundary = RigidFluidUtils::closestPointOnPolygon(fluidPos, worldPoly);
                    
                    // Calculate outward normal and push distance
                    Vector pushDir = RigidFluidUtils::calculateOutwardNormal(fluidPos, 
                                                                           closestBoundary, 
                                                                           worldPoly);
                    double pushDistance = (closestBoundary - fluidPos).length() + 
                                       FluidRigidConfig::SafetyMargin + fluid.radius;
                    
                    // Ensure minimum push
                    pushDistance = std::max(pushDistance, FluidRigidConfig::MinPushDistance);
                    
                    Vector correction = pushDir * pushDistance * FluidRigidConfig::RelaxationFactor;
                    totalCorrection += correction;
                    needsCorrection = true;
                }
            }
        }
        
        // Apply correction if needed
        if (needsCorrection) {
            // We need to modify the actual ECS position component
            auto* posComponent = registry.try_get<Components::Position>(fluid.id);
            if (posComponent) {
                posComponent->x += totalCorrection.x;
                posComponent->y += totalCorrection.y;
            }
        }
    }
}

} // namespace Systems
