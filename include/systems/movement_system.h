#ifndef MOVEMENT_SYSTEM_H
#define MOVEMENT_SYSTEM_H

#include <entt/entt.hpp>
#include "components.h"
#include "simulator_constants.h"
#include "debug_helpers.h"

namespace Systems
{
    class MovementSystem
    {
    public:
        static void update(entt::registry &registry)
        {
            auto view = registry.view<Components::Position, Components::Velocity>();
            static int frame_count = 0;
            frame_count++;

            // Time step in real seconds
            double dt = SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration;

            // Convert margin to meters
            double margin_m = 5.0 * SimulatorConstants::MetersPerPixel;
            double universe_size_m = SimulatorConstants::UniverseSizeMeters;
            double bounce_damping = 0.7;

            for (auto [entity, pos, vel] : view.each())
            {
                double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);
                DebugStats::updateMovement(speed, vel.x, vel.y);

                // Update position using dt
                pos.x += vel.x * dt;
                pos.y += vel.y * dt;

                // Bounce off boundaries (in meters)
                bool bounced = false;
                if (pos.x < margin_m)
                {
                    pos.x = margin_m;
                    vel.x = std::abs(vel.x) * bounce_damping;
                    bounced = true;
                }
                else if (pos.x > universe_size_m - margin_m)
                {
                    pos.x = universe_size_m - margin_m;
                    vel.x = -std::abs(vel.x) * bounce_damping;
                    bounced = true;
                }

                if (pos.y < margin_m)
                {
                    pos.y = margin_m;
                    vel.y = std::abs(vel.y) * bounce_damping;
                    bounced = true;
                }
                else if (pos.y > universe_size_m - margin_m)
                {
                    pos.y = universe_size_m - margin_m;
                    vel.y = -std::abs(vel.y) * bounce_damping;
                    bounced = true;
                }

                if (bounced)
                {
                    double speed_after = std::sqrt(vel.x * vel.x + vel.y * vel.y);
                    if (speed_after > 1.0)
                    {
                        vel.x *= 1.0 / speed_after;
                        vel.y *= 1.0 / speed_after;
                    }
                }
            }

            if (frame_count % 60 == 0)
            {
                DebugStats::printMovementStats();
            }
        }
    };
}

#endif