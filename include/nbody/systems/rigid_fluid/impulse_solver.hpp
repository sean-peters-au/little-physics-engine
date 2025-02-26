#pragma once

#include <entt/entt.hpp>

namespace Systems {

/**
 * @brief Solver that handles physical interactions between fluid and rigid bodies
 * 
 * Computes and applies forces like:
 * - Drag forces from fluid to rigid bodies
 * - Buoyancy forces on rigid bodies
 * - Pressure forces from rigid bodies to fluid
 * - Viscous friction at boundaries
 */
class RigidFluidImpulseSolver {
public:
    static void update(entt::registry &registry);
};

} // namespace Systems 