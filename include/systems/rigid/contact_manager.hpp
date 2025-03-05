/**
 * @file contact_manager.hpp
 * @brief Contact persistence and warm-starting system for rigid body collision
 *
 * This module manages collision contacts between physics frames, enabling impulse
 * warm-starting for improved solver stability. It acts as a bridge between the
 * narrow-phase collision detection and the constraint solver, maintaining contact
 * history and accumulated impulses.
 */

#pragma once

#include <vector>
#include <memory>
#include <entt/entt.hpp>
#include "systems/rigid/collision_data.hpp"
#include "math/vector_math.hpp"

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
 * @brief Represents all contact points (and a shared normal) for a pair of entities
 *
 * A single "manifold" may contain multiple contact points if two polygons
 * are resting against each other, or if an object has multiple contacts
 * against a flat surface.
 */
struct ContactManifoldRef {
    entt::entity a;              ///< First entity
    entt::entity b;              ///< Second entity
    Vector normal;               ///< Primary contact normal
    std::vector<ContactRef> contacts; ///< All contact points sharing that normal
};

/**
 * @brief Manages persistent contact information between physics frames
 *
 * The ContactManager bridges collision detection and constraint solving by:
 * - Grouping multiple contacts for the same entity pair into manifolds
 * - Storing/managing warm-start impulse data
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
     * @brief Retrieves current manifolds (each may have multiple contact points)
     *        with warm-start data for the solver
     * @return Reference to the manifold list
     */
    std::vector<ContactManifoldRef> &getManifoldsForSolver();

    /**
     * @brief Stores solver results for next-frame warm-starting
     * @param manifolds Updated manifold data from the solver
     */
    void applySolverResults(const std::vector<ContactManifoldRef> &manifolds);

    /**
     * @brief Removes contacts involving deleted entities
     * @param registry ECS registry for entity validation
     */
    void cleanupStaleContacts(entt::registry &registry);

    /**
     * @brief Retrieves the current collision manifold
     * @return Reference to the latest collision manifold
     *
     * Note: This is the raw manifold from the narrowphase (with each collision
     * as a separate entry). We store it in case other systems need direct access.
     */
    const CollisionManifold &getCurrentManifold() const;

private:
    class ContactData;  ///< PIMPL idiom for contact storage
    ContactData *m_data;

    /**
     * @brief Caches built from the latest narrowphase collisions, but merged
     *        into manifolds for the solver.
     */
    std::vector<ContactManifoldRef> m_manifoldsCache;
};

} // namespace RigidBodyCollision
