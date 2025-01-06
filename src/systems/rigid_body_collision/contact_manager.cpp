/**
 * @file contact_manager.cpp
 * @brief Implementation of contact persistence and warm-starting system
 *
 * Implements the ContactManager class, providing contact persistence between
 * physics frames and warm-starting data for the constraint solver.
 */

#include "nbody/systems/rigid_body_collision/contact_manager.hpp"
#include <unordered_map>
#include <algorithm>
#include <utility>
#include <cmath>

namespace RigidBodyCollision {

/**
 * @brief Holds accumulated impulses for warm-starting and the last frame's normal.
 */
struct WarmStartData {
    double normalImpulseAccum  = 0.0;
    double tangentImpulseAccum = 0.0;
    Vector oldNormal{0.0, 0.0}; ///< Normal used in the previous frame for this contact
    bool hasOldNormal = false;  ///< Whether oldNormal has been set at least once
};

/**
 * @brief Helper function to always sort the entity pair so (smaller, larger).
 * This ensures consistent lookups in the map.
 */
static std::pair<entt::entity, entt::entity> sortEntityPair(entt::entity a, entt::entity b)
{
    return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
}

/**
 * @brief Private implementation of contact persistence storage
 */
class ContactManager::ContactData
{
public:
    CollisionManifold latestManifold;  ///< Most recent collision data

    /**
     * @brief Stores warm-start impulse accumulators keyed by entity pair.
     *        Key is always sorted so (A, B) has A < B.
     */
    std::unordered_map<std::pair<entt::entity, entt::entity>, WarmStartData> persistentContacts;

    ContactData() = default;
    ~ContactData() = default;
};

//------------------------------------------------------------------------------
// ContactManager Implementation
//------------------------------------------------------------------------------

ContactManager::ContactManager()
    : m_data(new ContactData())
{
    // no-op
}

ContactManager::~ContactManager()
{
    delete m_data;
    m_data = nullptr;
}

/**
 * @brief Merge or replace the old manifold with new collisions from narrowPhase.
 *        Also cleans up stale warm-start data (if a collision no longer exists).
 */
void ContactManager::updateContacts(const CollisionManifold &manifold)
{
    // 1) Copy the new manifold
    m_data->latestManifold = manifold;

    // 2) Build a temporary set of active pairs in this new manifold
    std::unordered_map<std::pair<entt::entity, entt::entity>, bool> activePairs;
    for (auto &col : manifold.collisions) {
        auto sortedPair = sortEntityPair(col.a, col.b);
        activePairs[sortedPair] = true;
    }

    // 3) Remove stale entries from persistentContacts
    //    (collisions that no longer appear in the new manifold)
    for (auto it = m_data->persistentContacts.begin(); it != m_data->persistentContacts.end(); )
    {
        if (activePairs.find(it->first) == activePairs.end()) {
            it = m_data->persistentContacts.erase(it);
        } else {
            ++it;
        }
    }
}

/**
 * @brief Return the collisions from the last narrow phase.
 */
const CollisionManifold &ContactManager::getCurrentManifold() const
{
    return m_data->latestManifold;
}

/**
 * @brief Return the list of references to each contact for the solver,
 *        initializing them with previously accumulated impulses for warm-start,
 *        and resetting if the normal changed drastically since last frame.
 */
std::vector<ContactRef> &ContactManager::getContactsForSolver()
{
    // Clear the old cache first
    m_contactsCache.clear();

    // For each collision, see if we have stored impulses
    for (auto &col : m_data->latestManifold.collisions) {
        ContactRef c;
        c.a = col.a;
        c.b = col.b;
        c.normal = col.normal;
        c.contactPoint = col.contactPoint;
        c.penetration = col.penetration;

        auto sortedPair = sortEntityPair(c.a, c.b);
        auto it = m_data->persistentContacts.find(sortedPair);

        if (it != m_data->persistentContacts.end()) {
            WarmStartData &wsd = it->second;

            // Check if the normal drastically changed or flipped
            // We'll measure the dot product with the old normal if we have it
            if (wsd.hasOldNormal) {
                double dot = c.normal.dotProduct(wsd.oldNormal);
                // For example, if dot < some threshold, assume they've diverged
                // We can pick 0.95 or 0.98. If dot < 0, it means nearly opposite directions.
                const double threshold = 0.95;
                if (dot < threshold) {
                    // The normal changed significantly, so reset impulses
                    wsd.normalImpulseAccum = 0.0;
                    wsd.tangentImpulseAccum = 0.0;
                }
            }

            // Use the stored impulses after possible reset
            c.normalImpulseAccum  = wsd.normalImpulseAccum;
            c.tangentImpulseAccum = wsd.tangentImpulseAccum;
        } else {
            // No warm-start data yet
            c.normalImpulseAccum  = 0.0;
            c.tangentImpulseAccum = 0.0;
        }

        m_contactsCache.push_back(c);
    }

    return m_contactsCache;
}

/**
 * @brief Let the solver's results be recorded for next-frame warm-starting.
 */
void ContactManager::applySolverResults(const std::vector<ContactRef> &results)
{
    for (auto &c : results) {
        auto sortedPair = sortEntityPair(c.a, c.b);
        WarmStartData &wsd = m_data->persistentContacts[sortedPair];

        wsd.normalImpulseAccum  = c.normalImpulseAccum;
        wsd.tangentImpulseAccum = c.tangentImpulseAccum;
        wsd.oldNormal           = c.normal; // store the new normal
        wsd.hasOldNormal        = true;
    }
}

/**
 * @brief Cleanup stale contacts if entities have disappeared, etc.
 *        Also remove from our persistent store if the ECS says they're invalid.
 */
void ContactManager::cleanupStaleContacts(entt::registry &registry)
{
    for (auto it = m_data->persistentContacts.begin(); it != m_data->persistentContacts.end(); )
    {
        auto eA = it->first.first;
        auto eB = it->first.second;
        if (!registry.valid(eA) || !registry.valid(eB)) {
            it = m_data->persistentContacts.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace RigidBodyCollision