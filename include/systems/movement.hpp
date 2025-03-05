/**
 * @file movement_system.hpp
 * @brief System for updating positions of solid and gas particles based on velocity
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

#pragma once

#include <entt/entt.hpp>
#include "systems/i_system.hpp"

namespace Systems {

/**
 * @struct MovementConfig
 * @brief Configuration parameters specific to the movement system
 */
struct MovementConfig {
    // No specific configuration for now, but we include this
    // for consistency with other systems and future extensions
};

/**
 * @class MovementSystem
 * @brief Updates entity positions according to their velocity
 *
 * Ignores entities that are marked as asleep or are boundaries.
 */
class MovementSystem : public ConfigurableSystem<MovementConfig> {
public:
    /**
     * @brief Constructor with default configuration
     */
    MovementSystem();
    
    /**
     * @brief Virtual destructor
     */
    ~MovementSystem() override = default;
    
    /**
     * @brief Updates positions for all non-sleeping entities
     * @param registry EnTT registry containing entities and components
     */
    void update(entt::registry &registry) override;
};

} // namespace Systems
