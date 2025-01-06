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

namespace RigidBodyCollision {

/**
 * @brief Private implementation of contact persistence storage
 *
 * Currently maintains only the latest manifold, but designed to be
 * extended with persistent contact mapping and warm-start data.
 */
class ContactManager::ContactData
{
public:
    CollisionManifold latestManifold;  ///< Most recent collision data

    // Future extension point:
    // std::unordered_map<uint64_t, WarmStartData> persistentContacts;

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
 * @brief Merge or replace the old manifold with the new collisions from narrowPhase.
 *        For now, we just store the new data directly and discard the old.
 */
void ContactManager::updateContacts(const CollisionManifold &manifold)
{
    m_data->latestManifold = manifold;
}

/**
 * @brief Return the collisions from the last narrow phase.
 *        Currently a direct pass-through with no merging or impulse accumulation.
 */
const CollisionManifold &ContactManager::getCurrentManifold() const
{
    return m_data->latestManifold;
}

/**
 * @brief Return the list of references to each contact for the solver, 
 *        including any previously accumulated impulses (if we had them).
 */
const std::vector<ContactRef> &ContactManager::getContactsForSolver()
{
    // Clear the old cache first
    m_contactsCache.clear();

    // Translate each CollisionInfo => ContactRef
    // (In a more advanced version, you would use your map to retrieve 
    //  old impulses for each contact.)
    for (auto &col : m_data->latestManifold.collisions) {
        ContactRef c;
        c.a = col.a;
        c.b = col.b;
        c.normal = col.normal;
        c.contactPoint = col.contactPoint;
        c.penetration = col.penetration;

        // Warm-start impulses are not supported yet, so zero them
        c.normalImpulseAccum  = 0.0;
        c.tangentImpulseAccum = 0.0;

        m_contactsCache.push_back(c);
    }

    return m_contactsCache;
}

/**
 * @brief Let the solver's results be recorded. 
 *        (For now, we do nothing because we haven't implemented persistent impulses.)
 */
void ContactManager::applySolverResults(const std::vector<ContactRef> &results)
{
    // If we had a persistent store, we would copy out the impulses 
    // back into the appropriate data structures keyed by (entityA, entityB).
    (void)results; // do nothing
}

/**
 * @brief Cleanup stale contacts if entities have disappeared, etc.
 *        Currently a no-op.
 */
void ContactManager::cleanupStaleContacts(entt::registry &registry)
{
    // In a more advanced version, you would scan your stored 
    // contact pairs and remove entries whose entities are no longer valid.
    (void)registry; // no-op
}

} // namespace RigidBodyCollision