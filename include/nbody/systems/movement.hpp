/**
 * @file movement.hpp
 * @brief Linear motion and boundary collision system
 *
 * This system handles:
 * - Position updates based on velocity
 * - Universe boundary collisions with bouncing
 * - Linear and angular velocity damping
 * - Sleep state integration
 * 
 * Required components:
 * - Position (to modify)
 * - Velocity (to read/modify)
 * 
 * Optional components:
 * - Sleep (to check sleep state)
 * - AngularVelocity (for rotational damping)
 */

#ifndef MOVEMENT_SYSTEM_HPP
#define MOVEMENT_SYSTEM_HPP

#include <entt/entt.hpp>

namespace Systems {

/**
 * @brief System that manages linear motion and boundaries
 * 
 * Key parameters:
 * - 5m boundary margin
 * - 0.7 bounce damping
 * - 1% velocity damping per frame
 */
class MovementSystem {
public:
    /**
     * @brief Updates positions and handles boundary collisions
     * @param registry EnTT registry containing entities and components
     */
    static void update(entt::registry &registry);
};

}

#endif