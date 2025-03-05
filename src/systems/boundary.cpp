#include "nbody/systems/boundary.hpp"
#include "nbody/entities/entity_components.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/core/profile.hpp"
#include <cmath>

namespace Systems {

BoundarySystem::BoundarySystem() {
    // Initialize with default configurations
}

void BoundarySystem::update(entt::registry &registry) {
    PROFILE_SCOPE("BoundarySystem");

    // Convert margin to meters
    const double marginM = specificConfig.marginPixels * sysConfig.MetersPerPixel;
    const double universeSizeM = sysConfig.UniverseSizeMeters;
    const double bounceDamping = specificConfig.bounceDamping;
    const double maxSpeed = specificConfig.maxSpeed;

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

        // If we bounced and the velocity is too high, normalize it
        if (bounced) {
            double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);
            if (speed > maxSpeed) {
                vel.x = (vel.x / speed) * maxSpeed;
                vel.y = (vel.y / speed) * maxSpeed;
            }
        }
    }
}

} // namespace Systems 