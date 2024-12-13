#include "nbody/core/debug.hpp"

// Initialize static members
double DebugStats::max_force = 0.0;
double DebugStats::total_force = 0.0;
int DebugStats::force_count = 0;
double DebugStats::max_speed = 0.0;
double DebugStats::avg_dx = 0.0;
double DebugStats::avg_dy = 0.0;
int DebugStats::moving_particles = 0;
int DebugStats::total_particles = 0; 