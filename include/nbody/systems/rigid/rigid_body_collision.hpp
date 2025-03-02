/**
 * @file rigid_body_collision.hpp
 * @brief Main collision detection and resolution system for rigid bodies
 *
 * This module orchestrates the complete collision pipeline:
 * 1. Broad-phase: Quick AABB-based filtering of potential collisions
 * 2. Narrow-phase: Exact collision detection using GJK/EPA
 * 3. Contact solving: Iterative velocity-based constraint resolution
 * 4. Position correction: Baumgarte stabilization for penetration
 */

#pragma once

#include <entt/entt.hpp>
#include "nbody/systems/i_system.hpp"

namespace Systems {

/**
 * @struct RigidBodyCollisionConfig
 * @brief Configuration parameters specific to the collision system
 */
struct RigidBodyCollisionConfig {
    // Number of velocity solver iterations
    int solverIterations = 5;
    
    // Number of position correction passes
    int positionalSolverIterations = 2;
    
    // Position correction strength factor
    double baumgarte = 0.5;
    
    // Penetration tolerance before correction
    double slop = 0.001;
};

/**
 * @brief Main collision system coordinating detection and resolution
 *
 * Implements a complete collision handling pipeline that:
 * - Uses spatial partitioning for broad-phase filtering
 * - Performs exact collision tests with GJK/EPA
 * - Resolves collisions with an iterative impulse solver
 * - Corrects penetrations with position-based stabilization
 * - Supports both dynamic and static bodies
 * - Handles different particle phases (solid, fluid)
 */
class RigidBodyCollisionSystem : public ISystem {
public:
    /**
     * @brief Constructor with default configuration
     */
    RigidBodyCollisionSystem();
    
    /**
     * @brief Virtual destructor
     */
    ~RigidBodyCollisionSystem() override = default;
    
    /**
     * @brief Executes the complete collision pipeline
     * 
     * @param registry ECS registry containing physics components
     * 
     * @note The system automatically breaks early if no collisions are detected
     *       during broad-phase to avoid unnecessary computation
     */
    void update(entt::registry &registry) override;
    
    /**
     * @brief Sets the system configuration
     * @param config System configuration parameters
     */
    void setSystemConfig(const SystemConfig& config) override;
    
    /**
     * @brief Sets collision-specific configuration
     * @param config Collision specific configuration
     */
    void setCollisionConfig(const RigidBodyCollisionConfig& config);

private:
    SystemConfig sysConfig;
    RigidBodyCollisionConfig collisionConfig;
};

} // namespace Systems
