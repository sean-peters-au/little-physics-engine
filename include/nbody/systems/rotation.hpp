/**
 * @file rotation_system.hpp
 * @brief System for updating rotations based on angular velocity
 *
 * This system handles:
 * - Rotation updates using angular velocity
 * - Skips entities that are asleep
 *
 * Required components:
 * - Rotation (to modify)
 * - AngularVelocity (to read)
 *
 * Optional components:
 * - Sleep (to check if entity is asleep)
 */

#pragma once

#include <entt/entt.hpp>
#include "nbody/systems/i_system.hpp"

namespace Systems {

/**
 * @struct RotationConfig
 * @brief Configuration parameters specific to the rotation system
 */
struct RotationConfig {
    // Damping factor applied to angular velocity each frame (0-1)
    double angularDamping = 0.98;
    
    // Maximum allowed angular velocity in radians per second
    double maxAngularSpeed = 20.0;
};

/**
 * @class RotationSystem
 * @brief Updates entity rotations according to their angular velocity
 *
 * Ignores entities that are marked as asleep or are boundaries.
 */
class RotationSystem : public ConfigurableSystem<RotationConfig> {
public:
    /**
     * @brief Constructor with default configuration
     */
    RotationSystem();
    
    /**
     * @brief Virtual destructor
     */
    ~RotationSystem() override = default;
    
    /**
     * @brief Updates rotations for all non-sleeping entities
     * @param registry EnTT registry containing entities and components
     */
    void update(entt::registry &registry) override;
};

} // namespace Systems