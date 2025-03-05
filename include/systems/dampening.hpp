/**
 * @file dampening.hpp
 * @brief System for applying velocity dampening to entities
 *
 * This system handles:
 * - Applying linear velocity dampening
 * - Skipping entities that are asleep
 *
 * Required components:
 * - Velocity (to modify)
 *
 * Optional components:
 * - Sleep (to check if entity is asleep)
 */

#pragma once

#include <entt/entt.hpp>
#include "systems/i_system.hpp"

namespace Systems {

/**
 * @struct DampeningConfig
 * @brief Configuration parameters specific to the dampening system
 */
struct DampeningConfig {
    // Damping factor applied to velocity each frame (0-1)
    double linearDamping = 0.99;
};

/**
 * @class DampeningSystem
 * @brief Applies velocity dampening to entities
 *
 * Reduces velocity by a configurable factor each frame to simulate
 * drag or friction effects. Ignores entities that are asleep.
 */
class DampeningSystem : public ConfigurableSystem<DampeningConfig> {
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
     * @brief Applies dampening to velocities of all non-sleeping entities
     * @param registry EnTT registry containing entities and components
     */
    void update(entt::registry &registry) override;
};

} // namespace Systems
