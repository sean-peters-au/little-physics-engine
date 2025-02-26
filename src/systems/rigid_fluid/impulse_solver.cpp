/**
 * @file impulse_solver.cpp
 * @brief Handles the physical force interactions between fluid particles and rigid bodies
 */

#include <vector>
#include <algorithm>
#include <cmath>

#include <entt/entt.hpp>

#include "nbody/core/profile.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/math/vector_math.hpp"
#include "nbody/math/polygon.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sph.hpp"
#include "nbody/components/sim.hpp"
#include "nbody/systems/rigid_fluid/impulse_solver.hpp"
#include "nbody/systems/rigid_fluid/utils.hpp"

namespace Systems {

namespace FluidRigidForces {
    // Force parameters
    constexpr double DragCoefficient = 0.5;       // Fluid drag
    constexpr double BuoyancyFactor = 0.0;        // Buoyancy strength
    constexpr double ViscousFriction = 0.1;       // Friction at boundaries
    
    // Safety limits
    constexpr double MaxForce = 5.0;              // Maximum force per particle
    constexpr double MaxTorque = 1.0;             // Maximum torque per particle
}

// Calculate drag force with better scaling
Vector calculateDragForce(const Vector& relativeVel, double fluidDensity, double dt, int particleCount) {
    // Calculate drag magnitude: 0.5 * coefficient * density * vel^2
    double velMag = relativeVel.length();
    
    if (velMag < 1e-6) {
        return Vector(0, 0);
    }
    
    // Scale coefficient by 1/sqrt(particleCount) to reduce cumulative effects
    double scaledCoefficient = FluidRigidForces::DragCoefficient / std::sqrt(std::max(1, particleCount));
    
    double dragMag = 0.5 * scaledCoefficient * fluidDensity * velMag * velMag * dt;
    
    // Limit maximum drag
    dragMag = std::min(dragMag, FluidRigidForces::MaxForce);
    
    // FIXED: Apply force in same direction as velocity (not opposite)
    // The current code applies the force that the fluid exerts on the rigid body,
    // but we need the force that the rigid body experiences due to the fluid
    return relativeVel * (dragMag / velMag);
}

// Calculate buoyancy force based on submerged volume
Vector calculateBuoyancyForce(double submergedVolume, double fluidDensity, double dt) {
    // Simple buoyancy: density * volume * g * upward direction
    double buoyancyMag = fluidDensity * submergedVolume * 9.8 * 
                       FluidRigidForces::BuoyancyFactor * dt;
    
    // Apply upward
    return Vector(0, -buoyancyMag);
}

void RigidFluidImpulseSolver::update(entt::registry& registry) {
    PROFILE_SCOPE("RigidFluidImpulseSolver::update");
    
    // Get time step
    double dt = SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration;
    
    // Collect data using shared utilities
    auto fluidParticles = RigidFluidUtils::collectFluidParticles(registry);
    auto rigidBodies = RigidFluidUtils::collectRigidBodies(registry);
    
    // Early exit if no work to do
    if (fluidParticles.empty() || rigidBodies.empty()) {
        return;
    }
    
    // Calculate fluid rest density (could be a constant)
    double fluidRestDensity = 1000.0;
    
    // Process each rigid body
    for (auto& rigid : rigidBodies) {
        Vector totalForce(0, 0);
        double totalTorque = 0.0;
        int interactingParticles = 0;
        
        // Process each fluid particle
        for (const auto& fluid : fluidParticles) {
            // Broad phase AABB test
            if (fluid.pos.x < rigid.minX || fluid.pos.x > rigid.maxX ||
                fluid.pos.y < rigid.minY || fluid.pos.y > rigid.maxY) {
                continue;
            }
            
            // Calculate interaction radius (based on smoothing length)
            double interactionRadius = fluid.smoothingLength * 2.0;
            
            // Calculate relative position
            Vector rigidPos(*rigid.pos);
            Vector fluidPos(fluid.pos);
            Vector relativePos = fluidPos - rigidPos;
            
            // Skip distant particles
            double distSq = relativePos.dotProduct(relativePos);
            if (distSq > interactionRadius * interactionRadius) {
                continue;
            }
            
            // Calculate relative velocity
            Vector rigidVel(*rigid.vel);
            Vector fluidVel(fluid.vel);
            
            // Add rotational velocity for the rigid body point
            Vector rotVel(-rigid.angVel->omega * relativePos.y, 
                          rigid.angVel->omega * relativePos.x);
            
            Vector relativeVel = fluidVel - (rigidVel + rotVel);
            
            // Calculate forces
            
            // 1. Drag force
            Vector dragForce = calculateDragForce(relativeVel, fluid.density, dt, interactingParticles);
            
            // 2. Buoyancy (approximated as a small upward force per particle)
            // A more accurate approach would calculate submerged volume
            double particleVolume = fluid.mass / fluidRestDensity;
            Vector buoyancyForce = calculateBuoyancyForce(particleVolume, fluid.density, dt);
            
            // Sum forces
            Vector particleForce = dragForce + buoyancyForce;
            
            // Limit maximum force from one particle
            double forceMag = particleForce.length();
            if (forceMag > FluidRigidForces::MaxForce) {
                particleForce = particleForce * (FluidRigidForces::MaxForce / forceMag);
            }
            
            // Add to total force
            totalForce += particleForce;
            
            // Calculate torque (cross product in 2D)
            double torque = relativePos.cross(particleForce);
            
            // Limit maximum torque
            torque = std::clamp(torque, -FluidRigidForces::MaxTorque, FluidRigidForces::MaxTorque);
            
            totalTorque += torque;
            interactingParticles++;
        }
        
        // Before applying forces, scale based on particle count
        if (interactingParticles > 0) {
            // Apply force scaling based on particle count
            double forceScale = 1.0 / std::sqrt(interactingParticles);
            totalForce = totalForce * forceScale;
            totalTorque = totalTorque * forceScale;
            
            // Apply linear force with damping
            double invMass = 1.0 / rigid.mass.value;
            rigid.vel->x += totalForce.x * invMass;
            rigid.vel->y += totalForce.y * invMass;
            
            // Add velocity damping to prevent excessive speeds
            constexpr double dampingFactor = 0.98;
            rigid.vel->x *= dampingFactor;
            rigid.vel->y *= dampingFactor;
            
            // Apply angular force with damping
            if (rigid.inertia.I > 0) {
                double invInertia = 1.0 / rigid.inertia.I;
                rigid.angVel->omega += totalTorque * invInertia;
                rigid.angVel->omega *= dampingFactor;
            }
        }
    }
}

} // namespace Systems 