#ifndef MOVEMENT_SYSTEM_H
#define MOVEMENT_SYSTEM_H

#include <entt/entt.hpp>
#include "components.h"
#include "simulator_constants.h"
#include "debug_helpers.h"

namespace Systems {
    class MovementSystem {
    public:
        static void update(entt::registry& registry) {
            auto view = registry.view<Components::Position, Components::Velocity>();
            
            // Reset debug stats at start of frame
            static int frame_count = 0;
            frame_count++;
            if (frame_count % 60 == 0) {
                DebugStats::reset();
            }
            
            // Track center of mass for debugging
            double com_x = 0, com_y = 0;
            double total_mass = 0;
            
            for (auto [entity, pos, vel] : view.each()) {
                // Track motion statistics
                double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);
                DebugStats::updateMovement(speed, vel.x, vel.y);
                
                // Leapfrog position update: x(t + dt) = x(t) + v(t + dt/2)dt
                // Note: velocity is already at half-step from Barnes-Hut
                pos.x += vel.x;
                pos.y += vel.y;
                
                // Screen boundary handling with bounce and energy loss
                const double BOUNCE_DAMPING = 0.7;
                const double MARGIN = 5.0;
                
                bool bounced = false;
                if (pos.x < MARGIN) {
                    pos.x = MARGIN;
                    vel.x = std::abs(vel.x) * BOUNCE_DAMPING;
                    bounced = true;
                } else if (pos.x > SimulatorConstants::ScreenLength - MARGIN) {
                    pos.x = SimulatorConstants::ScreenLength - MARGIN;
                    vel.x = -std::abs(vel.x) * BOUNCE_DAMPING;
                    bounced = true;
                }
                
                if (pos.y < MARGIN) {
                    pos.y = MARGIN;
                    vel.y = std::abs(vel.y) * BOUNCE_DAMPING;
                    bounced = true;
                } else if (pos.y > SimulatorConstants::ScreenLength - MARGIN) {
                    pos.y = SimulatorConstants::ScreenLength - MARGIN;
                    vel.y = -std::abs(vel.y) * BOUNCE_DAMPING;
                    bounced = true;
                }
                
                // If particle bounced, reset its velocity to maintain stability
                if (bounced) {
                    double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);
                    if (speed > 1.0) {  // Clamp speed after bounce
                        vel.x *= 1.0/speed;
                        vel.y *= 1.0/speed;
                    }
                }
                
                // Track center of mass
                if (auto* mass = registry.try_get<Components::Mass>(entity)) {
                    com_x += pos.x * mass->value;
                    com_y += pos.y * mass->value;
                    total_mass += mass->value;
                }
            }
            
            // Output debug info every 60 frames
            if (frame_count % 60 == 0) {
                DebugStats::printMovementStats();
                if (total_mass > 0) {
                    com_x /= total_mass;
                    com_y /= total_mass;
                    DEBUG_MSG(DEBUG_LEVEL_VERBOSE,
                        "Center of mass: (" << com_x << ", " << com_y << ") pixels\n"
                    );
                }
            }
        }
    };
}

#endif 