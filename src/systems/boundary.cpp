#include "nbody/systems/boundary.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/core/profile.hpp"
#include <cmath>

namespace Systems {

void BoundarySystem::update(entt::registry &registry) {
    PROFILE_SCOPE("BoundarySystem");

    // Convert margin to meters
    const double marginM      = 15.0 * SimulatorConstants::MetersPerPixel;
    const double universeSizeM = SimulatorConstants::UniverseSizeMeters;
    const double bounceDamping = 0.7;

    // Create a view with Position and Velocity components.
    auto view = registry.view<Components::Position, Components::Velocity>();

    // Iterate over entities with direct reference access.
    // The use of "auto &&" ensures that pos and vel are bound as references.
    for (auto &&[entity, pos, vel] : view.each()) {
        // If the entity has a Sleep component and is asleep, skip it.
        if (auto* sleep = registry.try_get<Components::Sleep>(entity); sleep && sleep->asleep) {
            continue;
        }

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

        if (bounced) {
            // Compute the squared speed first to avoid the costly sqrt call
            double speedSq = (vel.x * vel.x) + (vel.y * vel.y);
            if (speedSq > 1.0) {
                double speed    = std::sqrt(speedSq);
                double invSpeed = 1.0 / speed;
                vel.x *= invSpeed;
                vel.y *= invSpeed;
            }
        }

        // Because we are modifying components in place,
        // there is no need to call registry.replace.
    }
}

} // namespace Systems 