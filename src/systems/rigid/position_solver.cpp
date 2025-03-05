/**
 * @file position_solver.cpp
 * @brief Position-based penetration correction for rigid body collisions
 *
 * Resolves remaining penetration between colliding bodies after the velocity solver
 * by directly adjusting positions. Uses a Baumgarte-style position correction scheme
 * that maintains relative body positions while eliminating overlap.
 * 
 * Key features:
 * - Batches collision data into arrays for efficient processing
 * - Handles both linear and angular corrections
 * - Supports static and dynamic bodies
 * - Preserves center of mass for multi-body contacts
 */

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <vector>

#include <entt/entt.hpp>

#include "entities/entity_components.hpp"
#include "core/profile.hpp"
#include "math/vector_math.hpp"
#include "systems/rigid/position_solver.hpp"
#include "systems/rigid/collision_data.hpp"

namespace RigidBodyCollision {

/**
 * @brief Minimal data needed for position correction of a single contact point
 */
struct PositionContact {
    int indexA;      ///< Index into body arrays for first body
    int indexB;      ///< Index into body arrays for second body
    Vector contactPoint;
    Vector normal;   ///< Points from A to B
    double penetration;
};

/**
 * @brief Per-body data for position solver
 */
struct BodyData {
    entt::entity e;
    bool valid;      ///< Skip if false (missing required components)
    bool canRotate;  ///< Has finite inertia
    bool isSolid;    ///< At least one body must be solid to solve

    double invMass;
    double invI;

    double x, y;     ///< Current position (updated during solve)
    double angle;    ///< Current angle (updated during solve)
};

/**
 * @brief Builds arrays of body data and position contacts from collision manifold
 * 
 * @param registry ECS registry containing body components
 * @param manifold Collision data from previous frame
 * @param[out] bodies Array of body data for efficient solving
 * @param[out] contacts Array of position contacts to resolve
 */
static void gatherPositionData(
    entt::registry &registry,
    const CollisionManifold &manifold,
    std::vector<BodyData> &bodies,
    std::vector<PositionContact> &contacts)
{
    bodies.clear();
    contacts.clear();
    bodies.reserve(manifold.collisions.size() * 2); // upper bound guess
    contacts.reserve(manifold.collisions.size());

    // We'll map entt::entity -> index in bodies[]
    std::unordered_map<entt::entity, int> indexMap;

    // A small lambda to add a body to our array if not present
    auto getBodyIndex = [&](entt::entity e) -> int {
        auto it = indexMap.find(e);
        if (it != indexMap.end()) {
            return it->second;
        }
        // Otherwise, create a new BodyData
        BodyData bd;
        bd.e = e;
        bd.valid = false;
        bd.canRotate = false;
        bd.isSolid = false;

        // Next index
        int const idx = static_cast<int>(bodies.size());
        bodies.push_back(bd);
        indexMap[e] = idx;
        return idx;
    };

    // 1) Create a body slot for every entity that appears in collisions
    //    We also skip collisions if both are non-solid.
    for (const auto &c : manifold.collisions) {
        if (!registry.valid(c.a) || !registry.valid(c.b)) {
            continue; // skip
        }
        // Check phases to see if at least one is solid
        // If neither is solid, skip contact
        bool aSolid = false;
        bool bSolid = false;
        if (registry.all_of<Components::ParticlePhase>(c.a)) {
            aSolid = (registry.get<Components::ParticlePhase>(c.a).phase == Components::Phase::Solid);
        }
        if (registry.all_of<Components::ParticlePhase>(c.b)) {
            bSolid = (registry.get<Components::ParticlePhase>(c.b).phase == Components::Phase::Solid);
        }
        if (!aSolid && !bSolid) {
            continue; 
        }
        // We keep it
        int const iA = getBodyIndex(c.a);
        int const iB = getBodyIndex(c.b);

        PositionContact pc;
        pc.indexA = iA;
        pc.indexB = iB;
        pc.contactPoint = c.contactPoint;
        pc.normal = c.normal;
        pc.penetration = c.penetration;
        contacts.push_back(pc);
    }
}

/**
 * @brief Loads mass, inertia and position data from ECS into body array
 */
static void loadBodyData(entt::registry &registry, std::vector<BodyData> &bodies)
{
    for (auto &bd : bodies) {
        entt::entity const e = bd.e;
        if (!registry.valid(e)) {
            bd.valid = false;
            continue;
        }
        // Must have position & mass to be relevant
        if (!registry.all_of<Components::Position, Components::Mass>(e)) {
            bd.valid = false;
            continue;
        }
        bd.valid = true;

        // Load position
        auto pos = registry.get<Components::Position>(e);
        bd.x = pos.x;
        bd.y = pos.y;

        // Load mass
        double const m = registry.get<Components::Mass>(e).value;
        bd.invMass = (m > 1e29) ? 0.0 : (1.0 / m);

        // Load angle if present
        bd.angle = 0.0;
        bd.canRotate = false;
        bd.invI = 0.0;
        if (registry.any_of<Components::AngularPosition>(e)) {
            bd.angle = registry.get<Components::AngularPosition>(e).angle;
        }
        if (registry.any_of<Components::Inertia>(e)) {
            double const i = registry.get<Components::Inertia>(e).I;
            if (i < 1e29 && i > 1e-12) {
                bd.canRotate = true;
                bd.invI = (1.0 / i);
            }
        }
        // Check if it's "solid"
        bd.isSolid = false;
        if (registry.all_of<Components::ParticlePhase>(e)) {
            auto ph = registry.get<Components::ParticlePhase>(e).phase;
            bd.isSolid = (ph == Components::Phase::Solid);
        }
    }
}

/**
 * @brief Writes final positions and angles back to ECS
 */
static void storeBodyData(entt::registry &registry, const std::vector<BodyData> &bodies)
{
    for (const auto &bd : bodies) {
        if (!bd.valid) {
            continue;
        }
        // Write position
        Components::Position newPos;
        newPos.x = bd.x;
        newPos.y = bd.y;
        registry.replace<Components::Position>(bd.e, newPos);

        // Write angle if any
        if (bd.canRotate && registry.any_of<Components::AngularPosition>(bd.e)) {
            auto &ap = registry.get<Components::AngularPosition>(bd.e);
            ap.angle = bd.angle;
            registry.replace<Components::AngularPosition>(bd.e, ap);
        }
    }
}

/**
 * @brief Performs one iteration of position correction over all contacts
 * 
 * Uses a Baumgarte-style position correction scheme that maintains relative body
 * positions while eliminating penetration. The correction is scaled by baumgarte
 * to avoid overshooting.
 */
static void solvePositionContactsOnce(
    std::vector<BodyData> &bodies,
    const std::vector<PositionContact> &contacts,
    double baumgarte,
    double slop)
{
    for (const auto &c : contacts) {
        int const iA = c.indexA;
        int const iB = c.indexB;
        if (iA < 0 || iB < 0) {
            continue;
        }
        BodyData &a = bodies[iA];
        BodyData &b = bodies[iB];

        // If either is invalid (missing mass or position), skip
        if (!a.valid || !b.valid) {
            continue;
        }
        // If neither is solid, skip
        if (!a.isSolid && !b.isSolid) {
            continue;
        }

        double const pen = c.penetration - slop;
        if (pen <= 0.0) {
            continue;
        }

        // normal from A to B
        Vector const n = c.normal.normalized();
        double const corr = baumgarte * pen;

        double const invMA = a.invMass;
        double const invMB = b.invMass;
        double const invIA = a.invI;
        double const invIB = b.invI;

        // Compute lever arms
        double const rxA = c.contactPoint.x - a.x;
        double const ryA = c.contactPoint.y - a.y;
        double const rxB = c.contactPoint.x - b.x;
        double const ryB = c.contactPoint.y - b.y;
        Vector const rA(rxA, ryA);
        Vector const rB(rxB, ryB);

        // Effective mass denominator = (invM_A + invM_B) + (rA x n)^2 * invI_A + ...
        double const rACrossN = rA.cross(n);
        double const rBCrossN = rB.cross(n);
        double const denom = invMA + invMB
                     + (rACrossN*rACrossN)*invIA
                     + (rBCrossN*rBCrossN)*invIB;
        if (denom < 1e-12) {
            continue;
        }
        double const scalar = corr / denom;
        double const dx = n.x * scalar;
        double const dy = n.y * scalar;

        // Move A opposite the normal
        a.x -= dx * invMA;
        a.y -= dy * invMA;
        if (a.canRotate) {
            double const rotA = rACrossN * scalar * invIA;
            a.angle -= rotA;
        }

        // Move B along the normal
        b.x += dx * invMB;
        b.y += dy * invMB;
        if (b.canRotate) {
            double const rotB = rBCrossN * scalar * invIB;
            b.angle += rotB;
        }
    }
}

/**
 * @brief Resolves penetration between colliding bodies by adjusting positions
 * 
 * @param registry ECS registry containing body components
 * @param manifold Collision data from previous frame
 * @param config Position solver configuration
 */
void PositionSolver::positionalSolver(
    entt::registry &registry, 
    const CollisionManifold &manifold,
    const PositionSolverConfig &config)
{
    PROFILE_SCOPE("PositionSolver");

    // Gather collisions & create arrays
    std::vector<BodyData> bodies;
    std::vector<PositionContact> contacts;
    gatherPositionData(registry, manifold, bodies, contacts);

    if (contacts.empty()) {
        return; // nothing to do
    }

    // Load body data from ECS once
    loadBodyData(registry, bodies);

    // Perform a few solver iterations
    for (int i = 0; i < config.iterations; i++) {
        solvePositionContactsOnce(bodies, contacts, config.baumgarte, config.slop);
    }

    // Store results back in ECS
    storeBodyData(registry, bodies);
}

} // namespace RigidBodyCollision