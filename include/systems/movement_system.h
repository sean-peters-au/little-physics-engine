#ifndef MOVEMENT_SYSTEM_H
#define MOVEMENT_SYSTEM_H

#include <entt/entt.hpp>
#include "components.h"
#include "simulator_constants.h"

namespace Systems {
    class MovementSystem {
    public:
        static void update(entt::registry& registry) {
            auto view = registry.view<Components::Position, Components::Velocity>();
            
            for (auto [entity, pos, vel] : view.each()) {
                // Convert velocity from m/s to pixels/tick, applying time acceleration
                double time_factor = SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration;
                double dx = SimulatorConstants::metersToPixels(vel.x * time_factor);
                double dy = SimulatorConstants::metersToPixels(vel.y * time_factor);
                
                // Update position
                pos.x += dx;
                pos.y += dy;
                
                // Screen boundary handling with bounce and energy loss
                const double BOUNCE_DAMPING = 0.7;  // More energy loss for gas simulation
                const double MARGIN = 5.0;
                
                if (pos.x < MARGIN) {
                    pos.x = MARGIN;
                    vel.x = std::abs(vel.x) * BOUNCE_DAMPING;  // Reverse and dampen velocity
                } else if (pos.x > SimulatorConstants::ScreenLength - MARGIN) {
                    pos.x = SimulatorConstants::ScreenLength - MARGIN;
                    vel.x = -std::abs(vel.x) * BOUNCE_DAMPING;
                }
                
                if (pos.y < MARGIN) {
                    pos.y = MARGIN;
                    vel.y = std::abs(vel.y) * BOUNCE_DAMPING;
                } else if (pos.y > SimulatorConstants::ScreenLength - MARGIN) {
                    pos.y = SimulatorConstants::ScreenLength - MARGIN;
                    vel.y = -std::abs(vel.y) * BOUNCE_DAMPING;
                }
                
                // Apply gas drag (simplified model)
                if (SimulatorConstants::DragCoeff > 0) {
                    double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);
                    if (speed > 0) {
                        double drag = SimulatorConstants::DragCoeff * 
                                    SimulatorConstants::ParticleDensity * 
                                    speed * speed;
                        double drag_factor = std::max(0.0, 1.0 - (drag * time_factor / speed));
                        vel.x *= drag_factor;
                        vel.y *= drag_factor;
                    }
                }
            }
        }
    };
}

#endif 