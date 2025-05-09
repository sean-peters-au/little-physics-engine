/**
 * @file boundary_system.hpp
 * @brief System for handling universe boundary collisions
 *
 * This system handles:
 * - Checking if entities are outside the allowed universe bounds
 * - Bouncing entities off boundaries
 * - Applying a bounce damping factor
 * - Clamping speed after bounce if it exceeds a certain threshold
 *
 * Required components:
 * - Position (to read/modify)
 * - Velocity (to read/modify)
 *
 * Optional components:
 * - Sleep (to skip asleep entities)
 */

#pragma once

#include <entt/entt.hpp>
#include "systems/i_system.hpp"

namespace Systems {

/**
 * @struct BoundaryConfig
 * @brief Configuration parameters specific to the boundary system
 */
struct BoundaryConfig {
    // Margin from edge in pixels
    double marginPixels = 15.0;
    
    // Damping factor applied to velocity on bounce (0-1)
    double bounceDamping = 0.7;
    
    // Maximum speed after bounce (if exceeded, velocity is normalized)
    double maxSpeed = 1.0;
};

/**
 * @class BoundarySystem
 * @brief Handles boundary collision checks and bounces
 *
 * Skips entities that are asleep. Applies a bounce damping
 * factor to velocity. Clamps velocity if it becomes too large.
 */
class BoundarySystem : public ConfigurableSystem<BoundaryConfig> {
public:
    /**
     * @brief Constructor with default configuration
     */
    BoundarySystem();
    
    /**
     * @brief Virtual destructor
     */
    ~BoundarySystem() override = default;
    
    /**
     * @brief Updates positions and velocities of entities that hit boundaries
     * @param registry EnTT registry containing entities and components
     */
    void update(entt::registry &registry) override;
};

} // namespace Systems
