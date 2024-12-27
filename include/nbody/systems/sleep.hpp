/**
 * @file sleep.hpp
 * @brief Sleep state management system for physics optimization
 *
 * This system manages the sleep state of physics entities to optimize simulation
 * performance. Entities with very low linear and angular velocities can be put
 * to "sleep" to avoid unnecessary physics calculations.
 * 
 * Required components:
 * - Velocity (to check linear motion)
 * - ParticlePhase (to identify eligible entities)
 * - Mass (required for physics bodies)
 * - Sleep (to store sleep state)
 * 
 * Optional components:
 * - AngularVelocity (to check rotational motion)
 */

#ifndef SLEEP_SYSTEM_HPP
#define SLEEP_SYSTEM_HPP

#include <entt/entt.hpp>

namespace Systems {

/**
 * @brief System that manages entity sleep states
 * 
 * Sleep conditions:
 * - Linear velocity below 0.5 units/s
 * - Angular velocity below 0.5 rad/s
 * - Conditions met for 60 consecutive frames
 */
class SleepSystem {
public:
    /**
     * @brief Updates sleep states as described above
     * @param registry EnTT registry containing entities and components
     */
    static void update(entt::registry& registry);
};

} // namespace Systems

#endif