#include "nbody/systems/gravity.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sim.hpp"
#include "nbody/core/constants.hpp"
#include <cmath>

namespace Systems {
    void BasicGravitySystem::update(entt::registry& registry) {
        // Get simulator state
        const auto& state = registry.get<Components::SimulatorState>(
            registry.view<Components::SimulatorState>().front()
        );

        // Uniform gravitational acceleration in m/s² downward
        double g = 9.8; 
        double dt = SimulatorConstants::SecondsPerTick * 
                   state.baseTimeAcceleration * state.timeScale;

        auto view = registry.view<Components::ParticlePhase, Components::Velocity, Components::Mass>();
        for (auto [entity, phase, vel, mass] : view.each()) {
            vel.y += g * dt; 
        }
    }
}