#include "nbody/systems/sleep.hpp"
#include "nbody/components/basic.hpp"

#include <cmath>
#include <iostream>

// Thresholds
static const double LINEAR_SLEEP_THRESHOLD = 0.5;
static const double ANGULAR_SLEEP_THRESHOLD = 0.5;
static const int SLEEP_FRAMES = 60;

void Systems::SleepSystem::update(entt::registry &registry) {
    auto view = registry.view<Components::Velocity, Components::AngularVelocity, Components::ParticlePhase, Components::Mass, Components::Sleep>();
    for (auto [entity, vel, angVel, phase, mass, sleep] : view.each()) {
        if (phase.phase != Components::Phase::Solid && phase.phase != Components::Phase::Liquid && phase.phase != Components::Phase::Gas) {
            continue; // Only sleep typical bodies
        }
        if (registry.all_of<Components::Boundary>(entity)) {
            continue; // Don't sleep boundaries
        }

        // Compute linear speed
        double speed = std::sqrt(vel.x*vel.x + vel.y*vel.y);

        // Check angular velocity if exists
        double angularSpeed = 0.0;
        if (registry.all_of<Components::AngularVelocity>(entity)) {
            auto &angVel = registry.get<Components::AngularVelocity>(entity);
            angularSpeed = std::fabs(angVel.omega);
        }

        if (speed < LINEAR_SLEEP_THRESHOLD && angularSpeed < ANGULAR_SLEEP_THRESHOLD) {
            // Candidate for sleeping
            if (!sleep.asleep) {
                sleep.sleepCounter++;
                if (sleep.sleepCounter > SLEEP_FRAMES) {
                    sleep.asleep = true;
                }
            }
        } else {
            // Entity is active, reset sleep counter
            sleep.sleepCounter = 0;
            sleep.asleep = false;
        }
        
        // If asleep, set velocity, angular velocity to zero
        if (sleep.asleep) {
            vel.x = 0;
            vel.y = 0;
            angVel.omega = 0;
        }
    }
}