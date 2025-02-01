#include "nbody/systems/boundary.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/core/profile.hpp"
#include <cmath>

namespace Systems {

void BoundarySystem::update(entt::registry &registry) {
    PROFILE_SCOPE("BoundarySystem");
    // Convert margin to meters
    double const marginM = 15.0 * SimulatorConstants::MetersPerPixel;
    double const universeSizeM = SimulatorConstants::UniverseSizeMeters;
    double const bounceDamping = 0.7;

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
            if (pos.x < marginM) {
                pos.x = marginM;
                vel.x = std::abs(vel.x) * bounceDamping;
                bounced = true;
            }
            // Check right boundary
            else if (pos.x > universeSizeM - marginM) {
                pos.x = universeSizeM - marginM;
                vel.x = -std::abs(vel.x) * bounceDamping;
                bounced = true;
            }

            // Check top boundary
            if (pos.y < marginM) {
                pos.y = marginM;
                vel.y = std::abs(vel.y) * bounceDamping;
                bounced = true;
            }
            // Check bottom boundary
            else if (pos.y > universeSizeM - marginM) {
                pos.y = universeSizeM - marginM;
                vel.y = -std::abs(vel.y) * bounceDamping;
                bounced = true;
            }

            // If bounced, optionally clamp speed if it gets too large
            if (bounced) {
                double const speedAfter = std::sqrt((vel.x * vel.x) + (vel.y * vel.y));
                if (speedAfter > 1.0) {
                    // Normalize velocity to speed 1.0
                    vel.x *= 1.0 / speedAfter;
                    vel.y *= 1.0 / speedAfter;
                }
            }

            // Update components in registry
            registry.replace<Components::Position>(entity, pos);
            registry.replace<Components::Velocity>(entity, vel);
        }
    }
}

} // namespace Systems 