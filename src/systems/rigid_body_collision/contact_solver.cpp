/**
 * @file contact_solver.cpp
 * @brief Linear Complementarity Problem (LCP) solver for rigid body contact resolution
 *
 * This solver implements a global approach to contact resolution by:
 * 1. Building a single LCP system combining all contact constraints
 * 2. Solving normal and friction constraints simultaneously using Projected Gauss-Seidel
 * 3. Applying the resulting impulses to update body velocities
 * 
 * Key features:
 * - Handles both normal and friction constraints
 * - Supports warm starting from previous frame's impulses
 * - Efficiently processes stacked and interlinked contacts
 * - Uses a sparse velocity representation for dynamic bodies only
 */

#include "nbody/systems/rigid_body_collision/contact_solver.hpp"
#include <unordered_map>
#include <cmath>
#include <iostream>

#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/core/profile.hpp"
#include "nbody/systems/rigid_body_collision/contact_manager.hpp"

#define ENABLE_CONTACT_SOLVER_DEBUG 0
static bool g_debugFilter = true;

#define DEBUG_LOG(x) \
    do { if (ENABLE_CONTACT_SOLVER_DEBUG && g_debugFilter) { std::cout << x << std::endl; } } while(0)

namespace RigidBodyCollision
{

/**
 * @brief Maps a rigid body to its degrees of freedom in the global velocity vector
 *
 * Each dynamic body gets 3 consecutive slots in the velocity vector for (vx, vy, omega).
 * Static bodies are marked with index = -1 to skip velocity updates.
 */
struct BodyDOF
{
    bool isDynamic = false;  ///< false for infinite mass or non-rotating bodies
    int index = -1;         ///< base index in global velocity array, or -1 if static
};

/**
 * @brief Check if the entity has infinite mass or is otherwise immovable
 */
static bool isInfiniteMass(const entt::registry &registry, entt::entity e)
{
    if (!registry.valid(e)) return false;
    if (!registry.all_of<Components::Mass>(e)) return false;
    const auto &m = registry.get<Components::Mass>(e);
    return (m.value > 1e29);
}

static bool canRotate(const entt::registry &registry, entt::entity e)
{
    // Must have angular velocity + inertia
    return registry.all_of<Components::AngularVelocity, Components::Inertia>(e)
        && registry.get<Components::Inertia>(e).I > 1e-12;
}

/**
 * @brief Build a global index mapping for all dynamic bodies in the system
 *        so we can store velocities in a single vector [vx, vy, w, vx, vy, w, ...].
 *
 * @return A map: entt::entity -> BodyDOF (with each dynamic body's base index).
 *         If body is static (infinite mass), or has no rotation, the solver sets them
 *         as dof=-1 or partial dof usage accordingly.
 */
static std::unordered_map<entt::entity, BodyDOF> buildBodyDOFTable(
    entt::registry &registry,
    const std::vector<ContactManifoldRef> &manifolds)
{
    // We only care about bodies that actually appear in collisions
    std::unordered_map<entt::entity, BodyDOF> table;

    // Gather collision participants
    for (auto &mref : manifolds) {
        for (auto &c : mref.contacts) {
            table[c.a] = BodyDOF();
            table[c.b] = BodyDOF();
        }
    }

    // Now assign an index for each dynamic body
    // each dynamic body has 3 dofs in 2D: vx, vy, w. (If infinite mass, dof = -1.)
    int currentIndex = 0;
    for (auto &kv : table) {
        entt::entity e = kv.first;
        auto &dof      = kv.second;

        double mass  = registry.get<Components::Mass>(e).value;
        bool dynamic = (mass < 1e29);

        if (!dynamic) {
            // Static or infinite mass => dof index stays -1
            dof.isDynamic = false;
            continue;
        }
        dof.isDynamic = true;

        // We place (vx, vy, w) in the global velocity array
        dof.index = currentIndex;   // This is the base index for this entity
        currentIndex += 3;          // 3 dofs in 2D
    }

    return table;
}

/**
 * @brief Represents a single constraint row in the LCP system
 *
 * Each contact generates two constraint rows:
 * - A normal constraint ensuring non-penetration (λ ≥ 0)
 * - A friction constraint bounded by ±µN
 */
struct ConstraintRow
{
    // Indices
    entt::entity a;   ///< Body A
    entt::entity b;   ///< Body B

    // The direction of this constraint (unit normal or friction tangent)
    Vector dir;       ///< Constraint direction in world space

    // Contact point relative positions
    Vector rA;        ///< contactPoint - posA
    Vector rB;        ///< contactPoint - posB

    // Effective mass: row i => K = J M^-1 J^T
    // We'll compute it on the fly
    double effMass = 0.0;

    // RHS offset (b) in the LCP: b_i = -(J v + ...)
    double rhs = 0.0;

    // Lower and upper bounds for impulse (e.g. normal >= 0, friction can be +/- frictionLimit).
    // For normal constraints: lo = 0, hi = +∞ (or some big number).
    // For friction constraints: lo = -fLimit, hi = +fLimit.
    double lo = 0.0;
    double hi = 0.0;

    // Accumulated impulse (for warm-start)
    double lambda = 0.0;
};

/**
 * @brief Pairs a normal and friction constraint for a single contact point
 *
 * The friction constraint's bounds are updated during solving based on
 * the normal constraint's impulse magnitude (λ_n) and friction coefficient.
 */
struct ContactRows
{
    // The normal constraint
    ConstraintRow normal;

    // The friction constraint
    ConstraintRow friction;

    // We keep a pointer back to the normal row's impulse so friction can see it
    // in real-time if we do iterative updates. This is typical in the PGS approach.
};

// -----------------------------------------------------------------------------
// Build the constraint rows (normal+friction) for each contact in each manifold.
// -----------------------------------------------------------------------------
static std::vector<ContactRows> buildConstraintRows(
    entt::registry &registry,
    const std::vector<ContactManifoldRef> &manifolds,
    double frictionCoeff)
{
    std::vector<ContactRows> allRows;
    allRows.reserve(manifolds.size() * 4); // guess

    for (auto &mref : manifolds) {
        for (auto &c : mref.contacts) {
            // Normal row
            ConstraintRow normalRow;
            normalRow.a = c.a;
            normalRow.b = c.b;
            normalRow.dir = c.normal.normalized();

            // Positions
            auto posA = registry.get<Components::Position>(c.a);
            auto posB = registry.get<Components::Position>(c.b);
            normalRow.rA = Vector(c.contactPoint.x - posA.x, c.contactPoint.y - posA.y);
            normalRow.rB = Vector(c.contactPoint.x - posB.x, c.contactPoint.y - posB.y);

            // For normal constraints: λ ≥ 0
            normalRow.lo = 0.0;
            normalRow.hi = 1e20; // large

            // We'll store warm-start impulse in normalRow.lambda
            normalRow.lambda = c.normalImpulseAccum;

            // We define a "restitution" or "velocity bias" if we want bouncing. 
            // For now, we set it to 0 or a small negative for slight position correction. 
            // We'll skip restitution in this example. 
            normalRow.rhs = 0.0; 

            // friction row
            ConstraintRow frictionRow;
            frictionRow.a = c.a;
            frictionRow.b = c.b;

            // The friction direction is tangent to the normal => rotate normal by +90° or -90° 
            frictionRow.dir = Vector(-normalRow.dir.y, normalRow.dir.x);

            frictionRow.rA = normalRow.rA;
            frictionRow.rB = normalRow.rB;

            // friction can be negative or positive => lo = -∞, hi = +∞, but we'll clamp to ±(µ * normalImpulse)
            frictionRow.lo = -1e20;
            frictionRow.hi = +1e20;

            frictionRow.lambda = c.tangentImpulseAccum;
            frictionRow.rhs = 0.0; // no bounce in tangential

            ContactRows rows;
            rows.normal   = normalRow;
            rows.friction = frictionRow;
            allRows.push_back(rows);
        }
    }
    return allRows;
}

// -----------------------------------------------------------------------------
// Compute the effective mass for a single constraint row. 
// "effMass = 1 / (J * M^-1 * J^T)"
// Where J is the row's Jacobian, M^-1 is diagonal block for the two bodies in question.
// In 2D, we do a small form for each row: see typical Box2D derivations.
// -----------------------------------------------------------------------------
static double computeEffectiveMass(
    const ConstraintRow &row,
    const std::unordered_map<entt::entity, BodyDOF> &dofTable,
    const std::vector<double> &invMass,
    const std::vector<double> &invInertia)
{
    // If body is infinite mass, dofTable[x].index < 0 => that body's invMass/invInertia = 0.
    double invM_A = 0.0, invM_B = 0.0, invI_A = 0.0, invI_B = 0.0;

    const auto &dA = dofTable.at(row.a);
    if (dA.isDynamic) {
        int idxA = dA.index / 3; // the "body slot"
        invM_A = invMass[idxA];
        invI_A = invInertia[idxA];
    }
    const auto &dB = dofTable.at(row.b);
    if (dB.isDynamic) {
        int idxB = dB.index / 3;
        invM_B = invMass[idxB];
        invI_B = invInertia[idxB];
    }

    // J = [ -dir, -(rA x dir) , dir, (rB x dir) ]
    // K = J * M^-1 * J^T => scalar in 2D
    // K = (invM_A + invM_B) + (rA x dir)^2 * invI_A + (rB x dir)^2 * invI_B
    double rA_cross_n = row.rA.cross(row.dir);
    double rB_cross_n = row.rB.cross(row.dir);

    double k = invM_A + invM_B
             + (rA_cross_n * rA_cross_n)*invI_A
             + (rB_cross_n * rB_cross_n)*invI_B;

    if (k < 1e-12) return 0.0;
    return 1.0 / k;
}

// -----------------------------------------------------------------------------
// Compute the velocity along the constraint direction for the row, i.e. J·v.
// We'll look up the velocity in our big array and do a standard 2D rigid body velocity 
// transformation at the contact point: v_contact = v_lin + cross(omega, r).
// Then dot with row.dir.
// -----------------------------------------------------------------------------
static double computeRelativeVelocity(
    const ConstraintRow &row,
    const std::unordered_map<entt::entity, BodyDOF> &dofTable,
    const std::vector<double> &v) // big velocity vector
{
    auto &dA = dofTable.at(row.a);
    auto &dB = dofTable.at(row.b);

    double vxA=0.0, vyA=0.0, wA=0.0;
    if (dA.isDynamic) {
        int base = dA.index;
        vxA = v[base + 0];
        vyA = v[base + 1];
        wA  = v[base + 2];
    }
    double vxB=0.0, vyB=0.0, wB=0.0;
    if (dB.isDynamic) {
        int base = dB.index;
        vxB = v[base + 0];
        vyB = v[base + 1];
        wB  = v[base + 2];
    }

    // Velocity at contact for A: vA_contact = (vxA, vyA) + cross(wA, rA).
    // cross(w, r) in 2D => ( -w * r.y, w * r.x ).
    Vector vA_contact(vxA - wA*row.rA.y, vyA + wA*row.rA.x);
    Vector vB_contact(vxB - wB*row.rB.y, vyB + wB*row.rB.x);

    Vector rel = vB_contact - vA_contact;
    return rel.dotProduct(row.dir);
}

/**
 * @brief Applies an impulse to each body's velocity in the global velocity vector
 *
 * This function updates the velocity vector v by applying the impulse Δλ * dir to each body.
 * It handles both dynamic and static bodies, updating their velocities accordingly.
 */
static void applyImpulse(
    const ConstraintRow &row,
    double dLambda,
    std::vector<double> &v,
    const std::unordered_map<entt::entity, BodyDOF> &dofTable,
    const std::vector<double> &invMass,
    const std::vector<double> &invInertia)
{
    if (std::fabs(dLambda) < 1e-15) return;

    const auto &dA = dofTable.at(row.a);
    const auto &dB = dofTable.at(row.b);

    int iA = dA.isDynamic ? (dA.index/3) : -1;
    int iB = dB.isDynamic ? (dB.index/3) : -1;

    double imA = (iA >= 0) ? invMass[iA] : 0.0;
    double imB = (iB >= 0) ? invMass[iB] : 0.0;
    double iiA = (iA >= 0) ? invInertia[iA] : 0.0;
    double iiB = (iB >= 0) ? invInertia[iB] : 0.0;

    // apply to A:  vA += -dir * (dLambda * imA)
    // angularA   += - (rA cross dir) * dLambda * iiA
    if (dA.isDynamic) {
        int baseA = dA.index;
        v[baseA+0] -= row.dir.x * (dLambda * imA);
        v[baseA+1] -= row.dir.y * (dLambda * imA);

        double crossA = row.rA.cross(row.dir);
        v[baseA+2]   -= crossA * dLambda * iiA;
    }

    // apply to B:  vB += +dir * (dLambda * imB)
    if (dB.isDynamic) {
        int baseB = dB.index;
        v[baseB+0] += row.dir.x * (dLambda * imB);
        v[baseB+1] += row.dir.y * (dLambda * imB);

        double crossB = row.rB.cross(row.dir);
        v[baseB+2]   += crossB * dLambda * iiB;
    }
}


/**
 * @brief Solves the Linear Complementarity Problem using Projected Gauss-Seidel
 *
 * This function iterates over each constraint row, computes the necessary incremental
 * impulse, clamps it to [lo, hi], and updates velocities.
 */
static void solveLCP_PGS(
    std::vector<ContactRows> &contactRows,
    std::vector<double> &v, // global velocity array
    const std::unordered_map<entt::entity, BodyDOF> &dofTable,
    const std::vector<double> &invMass,
    const std::vector<double> &invInertia,
    double frictionCoeff,
    int iterations = 10)
{
    // Precompute each row's effective mass
    for (auto &cr : contactRows) {
        cr.normal.effMass   = computeEffectiveMass(cr.normal, dofTable, invMass, invInertia);
        cr.friction.effMass = computeEffectiveMass(cr.friction, dofTable, invMass, invInertia);
    }

    for (int iter=0; iter<iterations; ++iter)
    {
        // For each contact's normal + friction rows
        for (auto &cr : contactRows) {
            // 1) Solve normal row
            {
                auto &row = cr.normal;
                double vn  = computeRelativeVelocity(row, dofTable, v);
                double lambdaOld = row.lambda;

                // The "constraint violation" is (vn + row.rhs). We want it >= 0 if row.lambda=0, etc.
                // Here: dλ = -effMass*(vn + b)
                double dLambda = - row.effMass * (vn + row.rhs);

                // Accumulate
                double newLambda = lambdaOld + dLambda;
                // clamp to [lo, hi]
                if (newLambda < row.lo) newLambda = row.lo;
                if (newLambda > row.hi) newLambda = row.hi;

                dLambda = newLambda - lambdaOld;
                row.lambda = newLambda;

                // Apply the impulse
                applyImpulse(row, dLambda, v, dofTable, invMass, invInertia);
            }

            // 2) Solve friction row
            {
                auto &frow = cr.friction;
                double vt = computeRelativeVelocity(frow, dofTable, v);
                double lambdaOld = frow.lambda;

                // Maximum friction = µ * normalImpulse
                double maxFriction = frictionCoeff * cr.normal.lambda;
                frow.lo = -maxFriction;
                frow.hi = +maxFriction;

                double dLambda = - frow.effMass * (vt + frow.rhs);
                double newLambda = lambdaOld + dLambda;
                if (newLambda < frow.lo) newLambda = frow.lo;
                if (newLambda > frow.hi) newLambda = frow.hi;

                dLambda = newLambda - lambdaOld;
                frow.lambda = newLambda;

                applyImpulse(frow, dLambda, v, dofTable, invMass, invInertia);
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Main routine: build the velocity array, build constraints, solve them, 
// then write velocities back to the ECS and store impulses in the contact manager.
// -----------------------------------------------------------------------------
void ContactSolver::solveContactConstraints(entt::registry &registry, ContactManager &manager)
{
    PROFILE_SCOPE("ContactSolver (LCP)");

    // Retrieve manifolds from manager
    auto &manifolds = manager.getManifoldsForSolver();
    if (manifolds.empty()) {
        return; // no collisions
    }

    // Build a map of "body -> DOF index"
    auto dofTable = buildBodyDOFTable(registry, manifolds);

    // Prepare a big velocity vector v, plus arrays for invMass & invInertia
    //    We'll store them "by body" not by dof-component, so invMass[i], invInertia[i].
    //    The dof index for body i is (3*i).
    //    We'll do a small pass to gather each dynamic body in dofTable in a stable order.
    //    For direct indexing, we store "entity -> i".
    std::vector<entt::entity> bodyList; 
    bodyList.reserve(dofTable.size());
    {
        // gather dynamic bodies in a stable vector
        for (auto &kv : dofTable) {
            if (kv.second.isDynamic) {
                bodyList.push_back(kv.first);
            }
        }
    }
    int bodyCount = static_cast<int>(bodyList.size());
    std::vector<double> vGlobal(bodyCount * 3, 0.0);
    std::vector<double> invMass(bodyCount, 0.0);
    std::vector<double> invInertia(bodyCount, 0.0);

    // Fill them from ECS
    for (int i = 0; i < bodyCount; i++) {
        entt::entity e = bodyList[i];
        double m = registry.get<Components::Mass>(e).value;
        double im = (m > 1e29) ? 0.0 : (1.0/m);
        double I  = 0.0;
        double iI = 0.0;
        if (canRotate(registry, e)) {
            I = registry.get<Components::Inertia>(e).I;
            if (I < 1e29 && I > 1e-12) {
                iI = 1.0 / I;
            }
        }
        invMass[i]    = im;
        invInertia[i] = iI;

        // Read current velocities from ECS
        auto &vel = registry.get<Components::Velocity>(e);
        double vx = vel.x;
        double vy = vel.y;
        double w  = 0.0;
        if (canRotate(registry, e)) {
            w = registry.get<Components::AngularVelocity>(e).omega;
        }
        // Place in vGlobal
        vGlobal[i*3 + 0] = vx;
        vGlobal[i*3 + 1] = vy;
        vGlobal[i*3 + 2] = w;
    }

    // 4) Build constraints (normal + friction) for all contacts
    static const double GlobalFrictionCoeff = 0.5; // Example value
    auto contactRows = buildConstraintRows(registry, manifolds, GlobalFrictionCoeff);

    // 5) Solve LCP
    solveLCP_PGS(contactRows, vGlobal, dofTable, invMass, invInertia, GlobalFrictionCoeff, 10);

    // 6) Write velocities back to ECS
    for (int i = 0; i < bodyCount; i++) {
        entt::entity e = bodyList[i];
        auto vel = registry.get<Components::Velocity>(e);
        vel.x = vGlobal[i*3 + 0];
        vel.y = vGlobal[i*3 + 1];
        registry.replace<Components::Velocity>(e, vel);

        if (canRotate(registry, e)) {
            auto angVel = registry.get<Components::AngularVelocity>(e);
            angVel.omega = vGlobal[i*3 + 2];
            registry.replace<Components::AngularVelocity>(e, angVel);
        }
    }

    // Store the final impulses back into the contact manager for warm-starting
    //    In each ContactRows, normal.lambda => c.normalImpulseAccum, friction.lambda => c.tangentImpulseAccum
    //    The manager's final step is applySolverResults(...), so we re-gather everything in a new
    //    manifold-like structure. However, we already have "manifolds" from the manager, so we just
    //    update them in place.
    //    We must match them 1:1 with contactRows, in the same order we built them. That was an
    //    iteration over manifolds => for each contact => build ContactRows. So we do the same iteration:

    {
        int rowIndex = 0;
        for (auto &mref : manifolds) {
            for (auto &c : mref.contacts) {
                auto &rows = contactRows[rowIndex++];
                c.normalImpulseAccum  = rows.normal.lambda;
                c.tangentImpulseAccum = rows.friction.lambda;
            }
        }
    }

    // 8) Let the manager officially store those impulses
    manager.applySolverResults(manifolds);

    DEBUG_LOG("LCP Contact Solver complete.");
}

} // namespace RigidBodyCollision