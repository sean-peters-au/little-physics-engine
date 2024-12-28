#ifndef NBODY_SYSTEMS_HPP
#define NBODY_SYSTEMS_HPP

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
    SPH,
    GRID_THERMODYNAMICS,
    MOVEMENT,
    SLEEP,
    DAMPENING,
    BOUNDARY
};

// (Optional) If you had logic to map a scenario to these systems, you'd do it here.
// e.g. std::vector<SystemType> getActiveSystems(...);
// but here we rely on ScenarioConfig for that.

} // namespace Systems

#endif // NBODY_SYSTEMS_HPP