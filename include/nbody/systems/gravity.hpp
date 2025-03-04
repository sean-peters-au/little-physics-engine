/**
 * @file gravity.hpp
 * @brief Basic gravitational force system for physics simulation
 *
 * This system implements a simple uniform gravitational field that applies
 * a constant downward acceleration to all entities with mass. It's a basic
 * approximation suitable for small-scale simulations where gravitational
 * variations are negligible.
 * 
 * Required components:
 * - ParticlePhase (for identification)
 * - Velocity (to modify)
 * - Mass (for future extensibility)
 */

#ifndef BASIC_GRAVITY_SYSTEM_H
#define BASIC_GRAVITY_SYSTEM_H

#include <entt/entt.hpp>
#include "nbody/systems/i_system.hpp"

namespace Systems {

/**
 * @struct GravityConfig
 * @brief Configuration parameters specific to the gravity system
 */
struct GravityConfig {
    // Gravitational acceleration in m/s²
    double gravitationalAcceleration = 9.8;
};

/**
 * @class BasicGravitySystem
 * @brief System that applies uniform gravitational acceleration
 * 
 * Applies constant downward acceleration, scaled by simulation time parameters.
 */
class BasicGravitySystem : public ConfigurableSystem<GravityConfig> {
public:
    /**
     * @brief Constructor with default configuration
     */
    BasicGravitySystem();
    
    /**
     * @brief Virtual destructor
     */
    ~BasicGravitySystem() override = default;
    
    /**
     * @brief Updates velocities of all entities affected by gravity
     * @param registry EnTT registry containing entities and components
     */
    void update(entt::registry& registry) override;
};

} // namespace Systems

#endif