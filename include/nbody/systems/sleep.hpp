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
#include "nbody/systems/i_system.hpp"

namespace Systems {

/**
 * @struct SleepConfig
 * @brief Configuration parameters specific to the sleep system
 */
struct SleepConfig {
    // Velocity threshold for sleep eligibility (units/s)
    double linearSleepThreshold = 0.5;
    
    // Angular velocity threshold for sleep eligibility (rad/s)
    double angularSleepThreshold = 0.5;
    
    // Number of consecutive frames below threshold before sleeping
    int sleepFramesThreshold = 60;
};

/**
 * @class SleepSystem
 * @brief System that manages entity sleep states
 * 
 * Sleep conditions:
 * - Linear velocity below threshold
 * - Angular velocity below threshold
 * - Conditions met for a configurable number of consecutive frames
 */
class SleepSystem : public ISystem {
public:
    /**
     * @brief Constructor with default configuration
     */
    SleepSystem();
    
    /**
     * @brief Virtual destructor
     */
    ~SleepSystem() override = default;
    
    /**
     * @brief Updates sleep states according to configuration
     * @param registry EnTT registry containing entities and components
     */
    void update(entt::registry& registry) override;
    
    /**
     * @brief Sets the system configuration
     * @param config System configuration parameters
     */
    void setSystemConfig(const SystemConfig& config) override;
    
    /**
     * @brief Sets sleep-specific configuration
     * @param config Sleep specific configuration
     */
    void setSleepConfig(const SleepConfig& config);

private:
    SystemConfig sysConfig;
    SleepConfig sleepConfig;
};

} // namespace Systems

#endif