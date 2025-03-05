#pragma once

#include <entt/entt.hpp>
#include "systems/i_system.hpp"

namespace Systems {

/**
 * @struct SleepConfig
 * @brief Configuration parameters specific to the sleep system
 */
struct SleepConfig {
    // Linear velocity threshold below which an entity is considered for sleep
    double linearSleepThreshold = 0.5;
    // Angular velocity threshold below which an entity is considered for sleep
    double angularSleepThreshold = 0.5;
    // Number of consecutive frames an entity must be "still" to sleep
    int framesBeforeSleep = 60;
    // (Optional) Distance threshold for position-based waking (unused here)
    double wakeDistance = 0.1;
};

/**
 * @class SleepSystem
 * @brief Manages entity sleep states to optimize performance
 *
 * Checks both linear and angular velocities. If both are under their respective
 * thresholds for `framesBeforeSleep` consecutive frames, the entity is put to sleep.
 * Once asleep, velocity is zeroed out until the entity is awakened by a higher velocity.
 */
class SleepSystem : public ConfigurableSystem<SleepConfig> {
public:
    /**
     * @brief Constructor with default configuration
     */
    SleepSystem() = default;
    
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