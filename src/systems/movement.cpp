#include "nbody/systems/movement.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include <cmath>
#include <iostream>

void Systems::MovementSystem::update(entt::registry &registry) {
    auto view = registry.view<Components::Position, Components::Velocity>();
    static int frame_count = 0;
    frame_count++;

    // Time step in real seconds
    double dt = SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration;

    // Convert margin to meters
    double margin_m = 5.0 * SimulatorConstants::MetersPerPixel;
    double universe_size_m = SimulatorConstants::UniverseSizeMeters;
    double bounce_damping = 0.7;

    for (auto [entity, pos, vel] : view.each()) {
        // If asleep, skip movement (as they won't move anyway)
        bool asleep = false;
        if (registry.any_of<Components::Sleep>(entity)) {
            auto &sleepC = registry.get<Components::Sleep>(entity);
            asleep = sleepC.asleep;
        }

        if (!asleep) {
            pos.x += vel.x * dt;
            pos.y += vel.y * dt;

            // Bounce off boundaries (in meters)
            bool bounced = false;
            if (pos.x < margin_m) {
                pos.x = margin_m;
                vel.x = std::abs(vel.x) * bounce_damping;
                bounced = true;
            } else if (pos.x > universe_size_m - margin_m) {
                pos.x = universe_size_m - margin_m;
                vel.x = -std::abs(vel.x) * bounce_damping;
                bounced = true;
            }

            if (pos.y < margin_m) {
                pos.y = margin_m;
                vel.y = std::abs(vel.y) * bounce_damping;
                bounced = true;
            } else if (pos.y > universe_size_m - margin_m) {
                pos.y = universe_size_m - margin_m;
                vel.y = -std::abs(vel.y) * bounce_damping;
                bounced = true;
            }

            if (bounced) {
                double speed_after = std::sqrt((vel.x * vel.x) + (vel.y * vel.y));
                if (speed_after > 1.0) {
                    vel.x *= 1.0 / speed_after;
                    vel.y *= 1.0 / speed_after;
                }
            }
        }

        registry.replace<Components::Position>(entity, pos);
        registry.replace<Components::Velocity>(entity, vel);
    }

    // Apply damping to all awake objects
    auto awakeView = registry.view<Components::Position, Components::Velocity>();
    for (auto [entity, pos, vel] : awakeView.each()) {
        bool asleep = false;
        if (registry.any_of<Components::Sleep>(entity)) {
            auto &sleepC = registry.get<Components::Sleep>(entity);
            asleep = sleepC.asleep;
        }

        if (!asleep) {
            // Linear damping
            vel.x *= 0.99;
            vel.y *= 0.99;

            // Angular damping if angular velocity present
            if (registry.any_of<Components::AngularVelocity>(entity)) {
                auto &angVel = registry.get<Components::AngularVelocity>(entity);
                angVel.omega *= 0.99;
                registry.replace<Components::AngularVelocity>(entity, angVel);
            }

            registry.replace<Components::Velocity>(entity, vel);
        }
    }
}