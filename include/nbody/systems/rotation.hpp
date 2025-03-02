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
 * @brief System that manages rotational motion
 * 
 * Implements angular physics with configurable:
 * - Angular damping per frame
 * - Speed limiting to a maximum value
 * - Angle wrapping to [0, 2π)
 */
class RotationSystem : public ISystem {
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
     * @brief Updates angular motion for all entities with rotation components
     * @param registry EnTT registry containing entities and components
     */
    void update(entt::registry &registry) override;
    
    /**
     * @brief Sets the system configuration
     * @param config System configuration parameters
     */
    void setSystemConfig(const SystemConfig& config) override;
    
    /**
     * @brief Sets rotation-specific configuration
     * @param config Rotation specific configuration
     */
    void setRotationConfig(const RotationConfig& config);

private:
    SystemConfig sysConfig;
    RotationConfig rotationConfig;
};

} // namespace Systems

#endif