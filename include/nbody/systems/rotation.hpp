/**
 * @file rotation.hpp
 * @brief Angular motion system for physics simulation
 *
 * This system manages rotational dynamics of physics entities, including:
 * - Angular position updates based on velocity
 * - Angular velocity damping
 * - Angular velocity clamping
 * - Angle normalization to [0, 2π)
 * 
 * Required components:
 * - AngularPosition (to modify)
 * - AngularVelocity (to read/modify)
 */

#ifndef ROTATION_SYSTEM_H
#define ROTATION_SYSTEM_H

#include <entt/entt.hpp>

namespace Systems {

/**
 * @brief System that manages rotational motion
 * 
 * Implements angular physics with:
 * - 2% damping per frame
 * - Speed limiting to ±20 rad/s
 * - Angle wrapping to [0, 2π)
 */
class RotationSystem {
public:
    /**
     * @brief Updates angular motion as described above
     * @param registry EnTT registry containing entities and components
     */
    static void update(entt::registry &registry);
};

} // namespace Systems

#endif