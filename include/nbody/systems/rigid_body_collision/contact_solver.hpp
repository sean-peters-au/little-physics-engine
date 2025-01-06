#ifndef CONTACT_SOLVER_HPP
#define CONTACT_SOLVER_HPP

#include <entt/entt.hpp>

#include "nbody/systems/rigid_body_collision/collision_data.hpp"
#include "nbody/systems/rigid_body_collision/contact_manager.hpp"

namespace RigidBodyCollision {

/**
 * @brief Velocity-only solver with a “split impulse” approach, plus friction.
 */
class ContactSolver {
public:
    /**
     * @brief Solve collisions from the manager’s manifold (velocity impulses).
     */
    static void solveContactConstraints(entt::registry &registry,
                                        const ContactManager &manager,
                                        double baumgarte,
                                        double slop);
};

} // namespace RigidBodyCollision

#endif // CONTACT_SOLVER_HPP