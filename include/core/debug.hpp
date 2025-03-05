#pragma once

#include <iostream>

// Set to 1 to enable debug output, 0 to disable
#define ENABLE_DEBUG 0

// Debug levels
#define DEBUG_LEVEL_NONE 0
#define DEBUG_LEVEL_BASIC 1
#define DEBUG_LEVEL_VERBOSE 2

// Set current debug level
#define CURRENT_DEBUG_LEVEL DEBUG_LEVEL_VERBOSE

// Debug macros
#define DEBUG_MSG(level, x) do { \
    if (ENABLE_DEBUG && level <= CURRENT_DEBUG_LEVEL) { \
        std::cout << x; \
    } \
} while(0)

// Helper class for collecting debug stats
class DebugStats {
public:
    static void reset() {
        max_force = 0.0;
        total_force = 0.0;
        force_count = 0;
        max_speed = 0.0;
        avg_dx = 0.0;
        avg_dy = 0.0;
        moving_particles = 0;
        total_particles = 0;
    }

    static void updateForce(double force) {
        max_force = std::max(max_force, force);
        total_force += force;
        force_count++;
    }

    static void updateMovement(double speed, double dx, double dy) {
        if (speed > 0) {
            moving_particles++;
            max_speed = std::max(max_speed, speed);
            avg_dx += std::abs(dx);
            avg_dy += std::abs(dy);
        }
        total_particles++;
    }

    static void printForceStats() {
        DEBUG_MSG(DEBUG_LEVEL_BASIC, 
            "Force stats:\n"
            "  Max force: " << max_force << " N\n"
            "  Avg force: " << (force_count > 0 ? total_force / force_count : 0) << " N\n"
            "  Force calculations: " << force_count << "\n"
        );
    }

    static void printMovementStats() {
        if (moving_particles > 0) {
            avg_dx /= moving_particles;
            avg_dy /= moving_particles;
        }
        DEBUG_MSG(DEBUG_LEVEL_BASIC,
            "Movement stats:\n"
            "  Moving particles: " << moving_particles << "/" << total_particles << "\n"
            "  Max speed: " << max_speed << " pixels/tick\n"
            "  Avg motion: (" << avg_dx << ", " << avg_dy << ") pixels/tick\n"
        );
    }

private:
    static double max_force;
    static double total_force;
    static int force_count;
    static double max_speed;
    static double avg_dx;
    static double avg_dy;
    static int moving_particles;
    static int total_particles;
}; 