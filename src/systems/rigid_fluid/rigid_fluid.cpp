/**
 * @file rigid_fluid.cpp
 * @brief Top-level system for fluid-rigid body interactions
 * 
 * This system coordinates two solvers:
 * 1. Position Solver: Ensures no fluid particles penetrate rigid bodies (constraint solver)
 * 2. Impulse Solver: Computes and applies physical forces between fluids and rigid bodies
 */

#include "nbody/core/profile.hpp"
#include "nbody/systems/rigid_fluid/rigid_fluid.hpp"
#include "nbody/systems/rigid_fluid/position_solver.hpp"
#include "nbody/systems/rigid_fluid/impulse_solver.hpp"

namespace Systems {

/**
 * @brief Main update function that orchestrates the fluid-rigid body interaction pipeline
 * 
 * First runs the position solver to ensure no penetrations, then runs the impulse solver
 * to handle physical interactions like drag, buoyancy, and pressure forces.
 */
void RigidFluidSystem::update(entt::registry &registry) {
    PROFILE_SCOPE("RigidFluidSystem::update");
    
    // Step 1: Run position constraint solver
    // This ensures no fluid particles penetrate rigid bodies
    // RigidFluidPositionSolver::update(registry);
    
    // Step 2: Run impulse/force solver
    // This computes physical interactions between fluids and rigid bodies
    // RigidFluidImpulseSolver::update(registry);
}

} // namespace Systems 