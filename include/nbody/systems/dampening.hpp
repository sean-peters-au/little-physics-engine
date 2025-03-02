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
#include "nbody/systems/i_system.hpp"

namespace Systems {

/**
 * @struct DampeningConfig
 * @brief Configuration parameters specific to the dampening system
 */
struct DampeningConfig {
    // Linear velocity dampening factor (per-frame multiplier)
    double linearDampingFactor = 0.99;
    
    // Angular velocity dampening factor (per-frame multiplier)
    double angularDampingFactor = 0.99;
};

/**
 * @class DampeningSystem
 * @brief Dampens linear and angular velocity for non-sleeping entities
 */
class DampeningSystem : public ISystem {
public:
    /**
     * @brief Constructor with default configuration
     */
    DampeningSystem();
    
    /**
     * @brief Virtual destructor
     */
    ~DampeningSystem() override = default;
    
    /**
     * @brief Applies velocity and angular velocity damping
     * @param registry EnTT registry containing entities and components
     */
    void update(entt::registry &registry) override;
    
    /**
     * @brief Sets the system configuration
     * @param config System configuration parameters
     */
    void setSystemConfig(const SystemConfig& config) override;
    
    /**
     * @brief Sets dampening-specific configuration
     * @param config Dampening specific configuration
     */
    void setDampeningConfig(const DampeningConfig& config);

private:
    SystemConfig sysConfig;
    DampeningConfig dampeningConfig;
};

} // namespace Systems

#endif 