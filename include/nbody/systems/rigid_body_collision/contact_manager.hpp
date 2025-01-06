#ifndef CONTACT_MANAGER_HPP
#define CONTACT_MANAGER_HPP

#include <entt/entt.hpp>

#include "nbody/systems/rigid_body_collision/collision_data.hpp"

namespace RigidBodyCollision {

/**
 * @brief Placeholder for a persistent contact manager.
 *        Currently does nothing but pass the manifold along.
 *        Later, we’ll implement warm-starting, storing impulses, etc.
 */
class ContactManager {
public:
    ContactManager() = default;

    /**
     * @brief Update our persistent contacts with this frame’s new collisions.
     * @param manifold The collisions found by narrowPhase
     */
    void updateContacts(const CollisionManifold &manifold) {
        // For now, do nothing—no persistent storage yet.
        m_currentManifold = manifold;
    }

    /**
     * @brief Return the current collisions.
     *        This is just a trivial pass-through in this version.
     */
    const CollisionManifold &getCurrentManifold() const {
        return m_currentManifold;
    }

    /**
     * @brief Clear out stale contacts, if needed.
     *        For now, do nothing.
     */
    void cleanupStaleContacts(entt::registry &/*registry*/) {
        // No operation
    }

private:
    CollisionManifold m_currentManifold;
};

} // namespace RigidBodyCollision

#endif // CONTACT_MANAGER_HPP