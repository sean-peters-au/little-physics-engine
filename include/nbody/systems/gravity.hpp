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

namespace Systems {

/**
 * @brief System that applies uniform gravitational acceleration
 * 
 * Applies constant downward acceleration (9.8 m/s²), scaled by simulation time parameters.
 */
class BasicGravitySystem {
public:
    /**
     * @brief Updates velocities of all entities affected by gravity
     * @param registry EnTT registry containing entities and components
     */
    static void update(entt::registry& registry);
};

} // namespace Systems

#endif