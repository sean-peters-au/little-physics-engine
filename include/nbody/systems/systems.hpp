#pragma once

#include <vector>
#include <string>

/**
 * @brief Defines available ECS systems for your simulation.
 */
namespace Systems {

/**
 * @enum SystemType
 * @brief The high-level ECS systems that can be activated in a scenario.
 */
enum class SystemType {
    BASIC_GRAVITY,
    BARNES_HUT,
    COLLISION,
    ROTATION,
    GRID_THERMODYNAMICS,
    MOVEMENT,
    SLEEP,
    DAMPENING,
    BOUNDARY,
    FLUID,
    RIGID_FLUID,
};

// (Optional) If you had logic to map a scenario to these systems, you'd do it here.
// e.g. std::vector<SystemType> getActiveSystems(...);
// but here we rely on ScenarioConfig for that.

} // namespace Systems
