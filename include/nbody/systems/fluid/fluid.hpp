#pragma once

#include <entt/entt.hpp>

namespace Systems {
    
/**
 * @class FluidSystem
 * @brief SPH-based fluid solver system, using single-precision arrays and a uniform grid for neighbor search.
 *
 * - Gathers all Liquid-phase particles that have SmoothingLength, SPHTemp, etc.
 * - Builds a grid to find neighbors.
 * - Computes densities and pressures.
 * - Computes forces (pressure, gravity, possibly viscosity).
 * - Writes force back to velocities in ECS.
 *
 * The fluid can eventually interact with rigid bodies by querying collisions or by
 * applying pressure on boundary shapes (future extension).
 */
class FluidSystem {
public:
    /**
     * @brief Main update function called each frame/tick
     * @param registry ECS registry
     */
    static void update(entt::registry &registry);
};

} // namespace Systems
