#include "nbody/systems/boundary.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/core/profile.hpp"
#include <cmath>

namespace Systems {

void BoundarySystem::update(entt::registry &registry) {
    PROFILE_SCOPE("BoundarySystem");
    // Convert margin to meters
    double margin_m = 15.0 * SimulatorConstants::MetersPerPixel;
    double universe_size_m = SimulatorConstants::UniverseSizeMeters;
    double bounce_damping = 0.7;

    // View of entities with Position and Velocity
    auto view = registry.view<Components::Position, Components::Velocity>();

    for (auto [entity, pos, vel] : view.each()) {
        // Check whether entity is asleep
        bool asleep = false;
        if (registry.any_of<Components::Sleep>(entity)) {
            auto &sleepC = registry.get<Components::Sleep>(entity);
            asleep = sleepC.asleep;
        }

        if (!asleep) {
            bool bounced = false;

            // Check left boundary
            if (pos.x < margin_m) {
                pos.x = margin_m;
                vel.x = std::abs(vel.x) * bounce_damping;
                bounced = true;
            }
            // Check right boundary
            else if (pos.x > universe_size_m - margin_m) {
                pos.x = universe_size_m - margin_m;
                vel.x = -std::abs(vel.x) * bounce_damping;
                bounced = true;
            }

            // Check top boundary
            if (pos.y < margin_m) {
                pos.y = margin_m;
                vel.y = std::abs(vel.y) * bounce_damping;
                bounced = true;
            }
            // Check bottom boundary
            else if (pos.y > universe_size_m - margin_m) {
                pos.y = universe_size_m - margin_m;
                vel.y = -std::abs(vel.y) * bounce_damping;
                bounced = true;
            }

            // If bounced, optionally clamp speed if it gets too large
            if (bounced) {
                double speed_after = std::sqrt((vel.x * vel.x) + (vel.y * vel.y));
                if (speed_after > 1.0) {
                    // Normalize velocity to speed 1.0
                    vel.x *= 1.0 / speed_after;
                    vel.y *= 1.0 / speed_after;
                }
            }

            // Update components in registry
            registry.replace<Components::Position>(entity, pos);
            registry.replace<Components::Velocity>(entity, vel);
        }
    }
}

} // namespace Systems 