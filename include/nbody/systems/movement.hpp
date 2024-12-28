/**
 * @file movement_system.hpp
 * @brief System for updating positions based on velocity
 *
 * This system handles:
 * - Position updates using velocity
 * - Skips entities that are asleep
 *
 * Required components:
 * - Position (to modify)
 * - Velocity (to read)
 *
 * Optional components:
 * - Sleep (to check if entity is asleep)
 */

#ifndef MOVEMENT_SYSTEM_HPP
#define MOVEMENT_SYSTEM_HPP

#include <entt/entt.hpp>

namespace Systems {

/**
 * @brief Updates entity positions according to their velocity
 *
 * Ignores entities that are marked as asleep.
 */
class MovementSystem {
public:
    /**
     * @brief Updates positions for all non-sleeping entities
     * @param registry EnTT registry containing entities and components
     */
    static void update(entt::registry &registry);
};

} // namespace Systems

#endif 