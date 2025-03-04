/**
 * @file sleep.hpp
 * @brief System for putting entities to sleep when they're not moving
 *
 * This system handles:
 * - Checking if entities have been still for a while
 * - Putting entities to sleep to save computation
 * - Waking entities when they're disturbed
 *
 * Required components:
 * - Position (to track movement)
 * - Velocity (to check if moving)
 * - Sleep (to modify sleep state)
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
    // Velocity threshold below which an entity is considered "still"
    double velocityThreshold = 0.01;
    
    // Number of consecutive frames an entity must be "still" to sleep
    int framesBeforeSleep = 60;
    
    // Distance threshold for position change that wakes an entity
    double wakeDistance = 0.1;
};

/**
 * @class SleepSystem
 * @brief Manages entity sleep states to optimize performance
 *
 * Puts entities to sleep when they haven't moved for a while,
 * and wakes them when they're disturbed by other entities.
 */
class SleepSystem : public ConfigurableSystem<SleepConfig> {
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
     * @brief Updates sleep states for all entities
     * @param registry EnTT registry containing entities and components
     */
    void update(entt::registry &registry) override;
};

} // namespace Systems

#endif