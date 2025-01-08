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

#define ENABLE_CONTACT_MANAGER_DEBUG 0
#define DEBUG(x) do { if (ENABLE_CONTACT_MANAGER_DEBUG) { std::cout << x; } } while(0)

namespace RigidBodyCollision {

/**
 * @brief Holds accumulated impulses for warm-starting and the last frame's contact normal.
 *
 * We store multiple contact points under a single manifold reference if
 * they share the same approximate normal. We identify that normal by
 * rounding or dot-product checks.
 */
struct ContactKey {
    std::pair<entt::entity, entt::entity> sortedPair;
    // We'll store a discrete "approximate" normal to combine multiple contacts.
    // For simplicity, we store the normal as-is, but in a real engine, you'd often
    // unify normals that differ by only a small angle. We'll keep it simpler here.
    Vector normal;

    // For hashing and equality, we'll only consider the pair plus the sign of normal
    // (or we can store exact normal). Minimal version below:
    bool operator==(const ContactKey &o) const {
        return (sortedPair == o.sortedPair) &&
               std::fabs(normal.x - o.normal.x) < 1e-7 &&
               std::fabs(normal.y - o.normal.y) < 1e-7;
    }
};

struct ContactKeyHash {
    size_t operator()(const ContactKey &ck) const {
        // We do a quick combination of the sortedPair hash + normal
        // For the normal, just fold in bits from the floats, ignoring small diffs
        // This is a simplistic approach but enough for an example
        auto pairHash = std::hash<std::pair<entt::entity, entt::entity>>{}(ck.sortedPair);
        // Combine normal bits
        auto nx = std::hash<long long>()(static_cast<long long>(ck.normal.x * 1000000.0));
        auto ny = std::hash<long long>()(static_cast<long long>(ck.normal.y * 1000000.0));
        // XOR them
        return pairHash ^ (nx + 0x9e3779b97f4a7c15ULL + (ny << 6) + (ny >> 2));
    }
};

/**
 * @brief Stores data needed for warm-start across multiple contact points
 *
 * For each manifold (unique pair+normal), we track a list of contact impulses
 * keyed by the contactPoint's position.
 */
struct ContactPointData {
    Vector contactPoint;
    double normalImpulseAccum  = 0.0;
    double tangentImpulseAccum = 0.0;
};

struct WarmStartManifoldData {
    // For quick lookup of existing contact points by position
    std::vector<ContactPointData> contactPoints;
    Vector oldNormal{0.0, 0.0}; ///< Normal used in the previous frame for this manifold
    bool hasOldNormal = false; 
};

/**
 * @brief Private storage for contact manager
 */
class ContactManager::ContactData
{
public:
    CollisionManifold latestManifold;  ///< Raw from narrowphase

    /**
     * @brief Map from (entity pair + approximate normal) to stored manifold data
     */
    std::unordered_map<ContactKey, WarmStartManifoldData, ContactKeyHash> persistentManifolds;

    ContactData() = default;
    ~ContactData() = default;
};

/**
 * @brief Helper function to always sort the entity pair so (smaller, larger).
 */
static std::pair<entt::entity, entt::entity> sortEntityPair(entt::entity a, entt::entity b)
{
    return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
}

//------------------------------------------------------------------------------
// ContactManager Implementation
//------------------------------------------------------------------------------

ContactManager::ContactManager()
    : m_data(new ContactData())
{
}

ContactManager::~ContactManager()
{
    delete m_data;
    m_data = nullptr;
}

/**
 * @brief Merge or replace old contacts with new collisions from the narrowPhase.
 *        Also cleans up stale manifold data (if a collision no longer exists).
 */
void ContactManager::updateContacts(const CollisionManifold &manifold)
{
    // 1) Copy the new manifold so others can use it if needed
    m_data->latestManifold = manifold;

    // 2) Build a temporary set of active keys from the new collisions
    std::unordered_map<ContactKey, bool, ContactKeyHash> activeKeys;

    for (auto &col : manifold.collisions) {
        // Sort the pair
        ContactKey key;
        key.sortedPair = sortEntityPair(col.a, col.b);
        key.normal = col.normal; // store exactly as given

        activeKeys[key] = true;
    }

    // 3) Remove stale manifold entries if they no longer appear
    for (auto it = m_data->persistentManifolds.begin(); it != m_data->persistentManifolds.end(); )
    {
        if (activeKeys.find(it->first) == activeKeys.end()) {
            it = m_data->persistentManifolds.erase(it);
        } else {
            ++it;
        }
    }
}

/**
 * @brief Return the raw manifold from narrowphase (useful for position solver).
 */
const CollisionManifold &ContactManager::getCurrentManifold() const
{
    return m_data->latestManifold;
}

/**
 * @brief Retrieve manifolds for the solver: group collisions by (pair+normal).
 *
 * We will fill `m_manifoldsCache` with one manifold per (pair+normal).
 * Each manifold can have multiple contact points if the narrowphase
 * returned multiple collisions with the same normal for that pair.
 *
 * We also attach warm-start impulses if they're in our persistent data.
 */
std::vector<ContactManifoldRef> &ContactManager::getManifoldsForSolver()
{
    m_manifoldsCache.clear();

    // Temporary structure keyed by ContactKey => vector of collisions
    std::unordered_map<ContactKey, std::vector<CollisionInfo>, ContactKeyHash> collisionsByKey;

    // Group all narrowphase collisions that share the same pair+normal
    for (auto &col : m_data->latestManifold.collisions) {
        ContactKey key;
        key.sortedPair = sortEntityPair(col.a, col.b);
        key.normal = col.normal;
        collisionsByKey[key].push_back(col);
    }

    // Now build manifold references, attaching warm-start data
    for (auto &kv : collisionsByKey) {
        const auto &key = kv.first;
        const auto &cols = kv.second;

        ContactManifoldRef manifoldRef;
        // Reconstruct who is "a" and who is "b" (the pair is always sorted, but
        // we keep the normal direction from the narrowphase)
        manifoldRef.a = key.sortedPair.first;
        manifoldRef.b = key.sortedPair.second;
        manifoldRef.normal = key.normal;
        
        // Retrieve or create a WarmStartManifoldData for this key
        auto wIt = m_data->persistentManifolds.find(key);
        if (wIt == m_data->persistentManifolds.end()) {
            // Not found, create blank
            WarmStartManifoldData blank;
            m_data->persistentManifolds[key] = blank;
            wIt = m_data->persistentManifolds.find(key);
        }
        auto &manifoldData = wIt->second;

        // If the normal changed drastically from old frame, we can reset
        if (manifoldData.hasOldNormal) {
            double dot = manifoldRef.normal.dotProduct(manifoldData.oldNormal);
            const double threshold = 0.95;
            if (dot < threshold) {
                // reset
                manifoldData.contactPoints.clear();
            }
        }

        // Build each contact point
        for (auto &cinfo : cols) {
            ContactRef c;
            c.a = cinfo.a;
            c.b = cinfo.b;
            c.normal = cinfo.normal;
            c.contactPoint = cinfo.contactPoint;
            c.penetration = cinfo.penetration;

            // Check if we have stored impulses from a prior frame
            bool foundPrev = false;
            for (auto &cpd : manifoldData.contactPoints) {
                // If same contact point (by position) or close enough, reuse accum
                double dx = cinfo.contactPoint.x - cpd.contactPoint.x;
                double dy = cinfo.contactPoint.y - cpd.contactPoint.y;
                double dist2 = dx*dx + dy*dy;
                if (dist2 < 1e-6) { 
                    // We'll use the previously accumulated impulses
                    c.normalImpulseAccum  = cpd.normalImpulseAccum;
                    c.tangentImpulseAccum = cpd.tangentImpulseAccum;
                    foundPrev = true;
                    break;
                }
            }
            // Not found => set zero accum
            if (!foundPrev) {
                c.normalImpulseAccum = 0.0;
                c.tangentImpulseAccum = 0.0;
            }

            manifoldRef.contacts.push_back(c);
        }

        m_manifoldsCache.push_back(manifoldRef);
    }

    return m_manifoldsCache;
}

/**
 * @brief Let the solver's results be recorded for next-frame warm-starting.
 */
void ContactManager::applySolverResults(const std::vector<ContactManifoldRef> &manifolds)
{
    for (auto &mref : manifolds) {
        // Build a key from the manifold
        ContactKey key;
        key.sortedPair = sortEntityPair(mref.a, mref.b);
        key.normal = mref.normal;

        auto &wsd = m_data->persistentManifolds[key];
        // Clear out the old data
        wsd.contactPoints.clear();

        // Store all contact points with updated impulses
        for (auto &c : mref.contacts) {
            ContactPointData cpd;
            cpd.contactPoint = c.contactPoint;
            cpd.normalImpulseAccum = c.normalImpulseAccum;
            cpd.tangentImpulseAccum = c.tangentImpulseAccum;

            wsd.contactPoints.push_back(cpd);
        }

        // Remember the normal for next frame
        wsd.oldNormal = mref.normal;
        wsd.hasOldNormal = true;
    }
}

/**
 * @brief Cleanup stale contacts if entities have disappeared, etc.
 *        Also remove from our persistent store if the ECS says they're invalid.
 */
void ContactManager::cleanupStaleContacts(entt::registry &registry)
{
    for (auto it = m_data->persistentManifolds.begin(); it != m_data->persistentManifolds.end(); )
    {
        auto eA = it->first.sortedPair.first;
        auto eB = it->first.sortedPair.second;
        if (!registry.valid(eA) || !registry.valid(eB)) {
            it = m_data->persistentManifolds.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace RigidBodyCollision