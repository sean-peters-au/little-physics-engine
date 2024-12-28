/**
 * @file dampening_system.hpp
 * @brief System for applying linear and angular velocity damping
 *
 * This system handles:
 * - Reducing entity velocity by a small factor each frame
 * - Reducing entity angular velocity if present
 * - Skips entities that are asleep
 *
 * Required components:
 * - Position (not strictly needed for damping, but included for consistency)
 * - Velocity (to apply linear damping)
 *
 * Optional components:
 * - AngularVelocity (to apply rotational damping)
 * - Sleep (to skip asleep entities)
 */

#ifndef DAMPENING_SYSTEM_HPP
#define DAMPENING_SYSTEM_HPP

#include <entt/entt.hpp>

namespace Systems {

/**
 * @brief Dampens linear and angular velocity for non-sleeping entities
 */
class DampeningSystem {
public:
    /**
     * @brief Applies velocity and angular velocity damping
     * @param registry EnTT registry containing entities and components
     */
    static void update(entt::registry &registry);
};

} // namespace Systems

#endif 