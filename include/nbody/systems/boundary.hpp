/**
 * @file boundary_system.hpp
 * @brief System for handling universe boundary collisions
 *
 * This system handles:
 * - Checking if entities are outside the allowed universe bounds
 * - Bouncing entities off boundaries
 * - Applying a bounce damping factor
 * - Clamping speed after bounce if it exceeds a certain threshold
 *
 * Required components:
 * - Position (to read/modify)
 * - Velocity (to read/modify)
 *
 * Optional components:
 * - Sleep (to skip asleep entities)
 */

#ifndef BOUNDARY_SYSTEM_HPP
#define BOUNDARY_SYSTEM_HPP

#include <entt/entt.hpp>

namespace Systems {

/**
 * @brief Handles boundary collision checks and bounces
 *
 * Skips entities that are asleep. Applies a bounce damping
 * factor to velocity. Clamps velocity if it becomes too large.
 */
class BoundarySystem {
public:
    /**
     * @brief Checks and processes boundary collisions
     * @param registry EnTT registry containing entities and components
     */
    static void update(entt::registry &registry);
};

} // namespace Systems

#endif 