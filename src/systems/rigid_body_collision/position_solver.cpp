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

#include "nbody/systems/rigid_body_collision/position_solver.hpp"
#include "nbody/systems/rigid_body_collision/collision_data.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/profile.hpp"
#include "nbody/math/vector_math.hpp"

#define ENABLE_POSITION_SOLVER_DEBUG 0

namespace RigidBodyCollision {

// If you'd like to experiment with fewer or more passes
static constexpr int POS_SOLVER_ITERATIONS = 3;
static constexpr double BAUMGARTE_FACTOR = 0.2; 
static constexpr double PENETRATION_SLOP = 0.001;

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
        int idx = static_cast<int>(bodies.size());
        bodies.push_back(bd);
        indexMap[e] = idx;
        return idx;
    };

    // 1) Create a body slot for every entity that appears in collisions
    //    We also skip collisions if both are non-solid.
    for (auto &c : manifold.collisions) {
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
        int iA = getBodyIndex(c.a);
        int iB = getBodyIndex(c.b);

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
        entt::entity e = bd.e;
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
        double m = registry.get<Components::Mass>(e).value;
        bd.invMass = (m > 1e29) ? 0.0 : (1.0 / m);

        // Load angle if present
        bd.angle = 0.0;
        bd.canRotate = false;
        bd.invI = 0.0;
        if (registry.any_of<Components::AngularPosition>(e)) {
            bd.angle = registry.get<Components::AngularPosition>(e).angle;
        }
        if (registry.any_of<Components::Inertia>(e)) {
            double I = registry.get<Components::Inertia>(e).I;
            if (I < 1e29 && I > 1e-12) {
                bd.canRotate = true;
                bd.invI = (1.0 / I);
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
    for (auto &bd : bodies) {
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
 * positions while eliminating penetration. The correction is scaled by BAUMGARTE_FACTOR
 * to avoid overshooting.
 */
static void solvePositionContactsOnce(std::vector<BodyData> &bodies,
                                    const std::vector<PositionContact> &contacts)
{
    for (auto &c : contacts) {
        int iA = c.indexA;
        int iB = c.indexB;
        if (iA < 0 || iB < 0) {
            continue;
        }
        BodyData &A = bodies[iA];
        BodyData &B = bodies[iB];

        // If either is invalid (missing mass or position), skip
        if (!A.valid || !B.valid) {
            continue;
        }
        // If neither is solid, skip
        if (!A.isSolid && !B.isSolid) {
            continue;
        }

        double pen = c.penetration - PENETRATION_SLOP;
        if (pen <= 0.0) {
            continue;
        }

        // normal from A to B
        Vector n = c.normal.normalized();
        double corr = BAUMGARTE_FACTOR * pen;

        double invM_A = A.invMass;
        double invM_B = B.invMass;
        double invI_A = A.invI;
        double invI_B = B.invI;

        // Compute lever arms
        double rxA = c.contactPoint.x - A.x;
        double ryA = c.contactPoint.y - A.y;
        double rxB = c.contactPoint.x - B.x;
        double ryB = c.contactPoint.y - B.y;
        Vector rA(rxA, ryA);
        Vector rB(rxB, ryB);

        // Effective mass denominator = (invM_A + invM_B) + (rA x n)^2 * invI_A + ...
        double rA_cross_n = rA.cross(n);
        double rB_cross_n = rB.cross(n);
        double denom = invM_A + invM_B
                     + (rA_cross_n*rA_cross_n)*invI_A
                     + (rB_cross_n*rB_cross_n)*invI_B;
        if (denom < 1e-12) {
            continue;
        }
        double scalar = corr / denom;
        double dx = n.x * scalar;
        double dy = n.y * scalar;

        // Move A opposite the normal
        A.x -= dx * invM_A;
        A.y -= dy * invM_A;
        if (A.canRotate) {
            double rotA = rA_cross_n * scalar * invI_A;
            A.angle -= rotA;
        }

        // Move B along the normal
        B.x += dx * invM_B;
        B.y += dy * invM_B;
        if (B.canRotate) {
            double rotB = rB_cross_n * scalar * invI_B;
            B.angle += rotB;
        }
    }
}

/**
 * @brief Resolves penetration between colliding bodies by adjusting positions
 * 
 * @param registry ECS registry containing body components
 * @param manifold Collision data from previous frame
 */
void PositionSolver::positionalSolver(entt::registry &registry, const CollisionManifold &manifold)
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
    for (int i = 0; i < POS_SOLVER_ITERATIONS; i++) {
        solvePositionContactsOnce(bodies, contacts);
    }

    // Store results back in ECS
    storeBodyData(registry, bodies);
}

} // namespace RigidBodyCollision