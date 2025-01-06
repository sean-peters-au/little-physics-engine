/**
 * @file contact_manager.hpp
 * @brief Contact persistence and warm-starting system for rigid body collision
 *
 * This module manages collision contacts between physics frames, enabling impulse
 * warm-starting for improved solver stability. It acts as a bridge between the
 * narrow-phase collision detection and the constraint solver, maintaining contact
 * history and accumulated impulses.
 */

#ifndef CONTACT_MANAGER_HPP
#define CONTACT_MANAGER_HPP

#include <vector>
#include <memory>
#include <entt/entt.hpp>
#include "nbody/systems/rigid_body_collision/collision_data.hpp"
#include "nbody/math/vector_math.hpp"

namespace std {
    /**
     * @brief Hash function for entity pairs to enable contact persistence
     */
    template<>
    struct hash<std::pair<entt::entity, entt::entity>> {
        size_t operator()(const std::pair<entt::entity, entt::entity>& p) const {
            size_t h1 = std::hash<entt::entity>{}(p.first);
            size_t h2 = std::hash<entt::entity>{}(p.second);
            return h1 ^ (h2 << 1);
        }
    };
}

namespace RigidBodyCollision {

/**
 * @brief Represents a single contact point for the physics solver
 *
 * Contains geometric contact information and accumulated impulses from
 * previous frames for warm-starting the constraint solver.
 */
struct ContactRef {
    entt::entity a;              ///< First entity in contact
    entt::entity b;              ///< Second entity in contact
    Vector normal;               ///< Contact normal pointing from A to B
    Vector contactPoint;         ///< World space point of contact
    double penetration;          ///< Penetration depth
    double normalImpulseAccum;   ///< Accumulated normal impulse for warm-starting
    double tangentImpulseAccum;  ///< Accumulated friction impulse for warm-starting
};

/**
 * @brief Manages persistent contact information between physics frames
 *
 * The ContactManager bridges collision detection and constraint solving by:
 * - Storing contact points between frames
 * - Merging new contacts with existing ones
 * - Maintaining warm-start impulse data
 * - Cleaning up stale contacts
 */
class ContactManager {
public:
    ContactManager();
    ~ContactManager();

    /**
     * @brief Updates the contact set with new collision data
     * @param manifold New collision manifold from narrow-phase detection
     */
    void updateContacts(const CollisionManifold &manifold);

    /**
     * @brief Retrieves current contacts with warm-start data for the solver
     * @return Reference to the contact list
     */
    const std::vector<ContactRef> &getContactsForSolver();

    /**
     * @brief Stores solver results for next-frame warm-starting
     * @param results Updated contact data from the solver
     */
    void applySolverResults(const std::vector<ContactRef> &results);

    /**
     * @brief Removes contacts involving deleted entities
     * @param registry ECS registry for entity validation
     */
    void cleanupStaleContacts(entt::registry &registry);

    /**
     * @brief Retrieves the current collision manifold
     * @return Reference to the latest collision manifold
     */
    const CollisionManifold &getCurrentManifold() const;

private:
    class ContactData;  ///< PIMPL idiom for contact storage
    ContactData *m_data;
    std::vector<ContactRef> m_contactsCache;  ///< Working copy for solver
};

} // namespace RigidBodyCollision
#endif