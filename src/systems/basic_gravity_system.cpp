#include "systems/basic_gravity_system.h"
#include "components.h"
#include "simulator_constants.h"
#include <cmath>

namespace Systems {
    void BasicGravitySystem::update(entt::registry& registry) {
        // Uniform gravitational acceleration in m/s² downward
        double g = 9.8; 
        double dt = SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration;

        auto view = registry.view<Components::ParticlePhase, Components::Velocity, Components::Mass>();
        for (auto [entity, phase, vel, mass] : view.each()) {
            if (phase.phase == Components::Phase::Gas) {
                // Add acceleration in y-direction (negative if you prefer top-down screen)
                vel.y += g * dt; 
            }
        }
    }
}