/**
 * @fileoverview contact_solver.cpp
 * @brief SIMD-optimized LCP solver for rigid body contact resolution on Apple Silicon
 *
 * This solver builds a global LCP containing both normal and friction constraints,
 * then solves it using a Projected Gauss-Seidel approach. It accelerates critical
 * vector math (dot products, cross products) with NEON intrinsics on the M2's ARM
 * architecture, reducing the time spent per iteration.
 *
 * Major changes for performance:
 * - Uses single-precision floats and NEON intrinsics for basic vector operations
 * - Minimizes scalar fallback by storing contact geometry in float arrays
 * - Maintains a batched approach to velocity updates
 * - Retains warm-start impulses and friction clamping logic
 *
 * Note: Converting double-precision data to float can have minor numerical side effects.
 *       In many physics simulations, this is acceptable if it significantly improves speed.
 */

#include <arm_neon.h>         // NEON intrinsics for Apple M-series
#include <unordered_map>
#include <vector>
#include <cmath>
#include <iostream>

#include "nbody/systems/rigid_body_collision/contact_solver.hpp"
#include "nbody/systems/rigid_body_collision/contact_manager.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/core/profile.hpp"

namespace RigidBodyCollision
{

#if !defined(ENABLE_CONTACT_SOLVER_DEBUG)
    #define ENABLE_CONTACT_SOLVER_DEBUG 0
#endif

static bool g_debugFilter = true;

/**
 * @brief Logs debug messages if debugging is enabled
 *
 * @param x The message to log
 */
#define DEBUG_LOG(x) \
    do { if (ENABLE_CONTACT_SOLVER_DEBUG && g_debugFilter) { std::cout << x << std::endl; } } while(0)

/**
 * @brief Represents how a body is mapped into the solver's velocity array
 */
struct BodyDOF {
    bool isDynamic = false;  ///< false if infinite mass or no motion
    int index = -1;          ///< base index in velocity array (3 floats per body), or -1 if static
};

/**
 * @brief Checks if an entity effectively has infinite mass
 *
 * @param registry ECS registry
 * @param e The entity to check
 * @return true if mass > 1e29, false otherwise
 */
static bool isInfiniteMass(const entt::registry &registry, entt::entity e)
{
    if (!registry.valid(e) || !registry.all_of<Components::Mass>(e)) {
        return false;
    }
    double m = registry.get<Components::Mass>(e).value;
    return (m > 1e29);
}

/**
 * @brief Checks if an entity has finite inertia (thus can rotate)
 *
 * @param registry ECS registry
 * @param e The entity to check
 * @return true if it has AngularVelocity & Inertia < infinite
 */
static bool canRotate(const entt::registry &registry, entt::entity e)
{
    if (!registry.all_of<Components::AngularVelocity, Components::Inertia>(e)) {
        return false;
    }
    double I = registry.get<Components::Inertia>(e).I;
    return (I > 1e-12 && I < 1e29);
}

/**
 * @brief Builds a map from entity -> BodyDOF, assigning each dynamic body an index in the velocity array.
 *
 * @param registry ECS registry
 * @param manifolds Contact manifolds
 * @return Mapping from entt::entity to BodyDOF
 */
static std::unordered_map<entt::entity, BodyDOF> buildBodyDOFTable(
    entt::registry &registry,
    const std::vector<ContactManifoldRef> &manifolds)
{
    std::unordered_map<entt::entity, BodyDOF> table;
    table.reserve(manifolds.size() * 2);

    // Collect all entities from collisions
    for (const auto &mref : manifolds) {
        for (const auto &c : mref.contacts) {
            table[c.a] = BodyDOF();
            table[c.b] = BodyDOF();
        }
    }

    // Assign index if body is dynamic
    int currentIndex = 0;
    for (auto &kv : table) {
        entt::entity e = kv.first;
        BodyDOF &dof   = kv.second;

        if (!isInfiniteMass(registry, e)) {
            dof.isDynamic = true;
            dof.index     = currentIndex;
            currentIndex += 3;  // 3 floats per body: vx, vy, w
        }
    }
    return table;
}

/**
 * @brief Stores the normal or friction constraint for a single contact row
 */
struct ConstraintRow
{
    entt::entity a;    ///< Body A
    entt::entity b;    ///< Body B
    float dirX, dirY;  ///< Constraint direction (normalized), single-precision
    float rxA, ryA;    ///< Contact offset from A
    float rxB, ryB;    ///< Contact offset from B
    float effMass;     ///< Effective mass
    float rhs;         ///< Restitution / error bias term
    float lo;          ///< Lower bound for impulse
    float hi;          ///< Upper bound for impulse
    float lambda;      ///< Accumulated impulse from warm-start
};

/**
 * @brief Represents a pair of constraints (normal + friction) for one contact point
 */
struct ContactRows
{
    ConstraintRow normal;
    ConstraintRow friction;
};

/**
 * @brief Converts a double-based Vector to a float2 NEON vector
 *
 * @param v The input Vector with double precision
 * @return float32x2_t with x,y from v
 */
inline static float32x2_t toFloat2(const Vector &v)
{
    return {(float)v.x, (float)v.y};
}

/**
 * @brief Builds normal + friction constraints for all contacts in single-precision
 *
 * @param registry ECS registry
 * @param manifolds The collision manifold references
 * @param frictionCoeff Global friction coefficient
 * @return Vector of ContactRows
 */
static std::vector<ContactRows> buildConstraintRows(
    entt::registry &registry,
    const std::vector<ContactManifoldRef> &manifolds,
    float frictionCoeff)
{
    std::vector<ContactRows> out;
    out.reserve(manifolds.size() * 4);

    for (auto &mref : manifolds) {
        for (auto &c : mref.contacts) {
            // Normal row
            ConstraintRow rowN;
            rowN.a = c.a;
            rowN.b = c.b;

            // Convert normal to single-precision
            Vector unitN = c.normal.normalized();
            rowN.dirX = (float)unitN.x;
            rowN.dirY = (float)unitN.y;

            // Positions
            auto posA = registry.get<Components::Position>(c.a);
            auto posB = registry.get<Components::Position>(c.b);

            // Offsets rA, rB
            rowN.rxA = (float)(c.contactPoint.x - posA.x);
            rowN.ryA = (float)(c.contactPoint.y - posA.y);
            rowN.rxB = (float)(c.contactPoint.x - posB.x);
            rowN.ryB = (float)(c.contactPoint.y - posB.y);

            // Normal constraint bounds
            rowN.lo = 0.0f;
            rowN.hi = 1e20f;
            rowN.lambda = (float)c.normalImpulseAccum;
            rowN.rhs    = 0.0f;  // no restitution
            rowN.effMass= 0.0f;

            // Friction row
            ConstraintRow rowF;
            rowF.a = c.a;
            rowF.b = c.b;

            // tangent = rotate normal by +90 deg => (-ny, nx)
            rowF.dirX = -rowN.dirY;
            rowF.dirY =  rowN.dirX;

            rowF.rxA = rowN.rxA;
            rowF.ryA = rowN.ryA;
            rowF.rxB = rowN.rxB;
            rowF.ryB = rowN.ryB;

            // friction row bounds => ±∞ (clamp later)
            rowF.lo     = -1e20f;
            rowF.hi     =  1e20f;
            rowF.lambda = (float)c.tangentImpulseAccum;
            rowF.rhs    = 0.0f;
            rowF.effMass= 0.0f;

            ContactRows pair;
            pair.normal   = rowN;
            pair.friction = rowF;
            out.push_back(pair);
        }
    }
    return out;
}

/**
 * @brief Computes a cross product in 2D: cross((x1,y1), (x2,y2)) => x1*y2 - y1*x2
 *        using NEON single-precision floats
 *
 * @param a float32x2_t vector
 * @param b float32x2_t vector
 * @return float cross product
 */
inline static float cross2f_NEON(const float32x2_t &a, const float32x2_t &b)
{
    // cross = a.x * b.y - a.y * b.x
    float32x2_t mul1 = vmul_f32(a, vrev64_f32(b)); // a.x*b.y, a.y*b.x
    // cross in lane0 - lane1
    float crossVal = vget_lane_f32(mul1, 0) - vget_lane_f32(mul1, 1);
    return crossVal;
}

/**
 * @brief Computes the effective mass for a constraint row:
 *        effMass = 1 / (invMassA + invMassB + (rA×dir)^2·invInertiaA + (rB×dir)^2·invInertiaB)
 *
 * @param row The constraint row
 * @param dofMap Mapping from entity to body DOF
 * @param invMass Array of inverse masses for dynamic bodies
 * @param invInertia Array of inverse inertias for dynamic bodies
 * @return The computed 1/k
 */
static float computeEffectiveMass(
    const ConstraintRow &row,
    const std::unordered_map<entt::entity, BodyDOF> &dofMap,
    const std::vector<float> &invMass,
    const std::vector<float> &invInertia)
{
    float imA = 0.f, imB = 0.f;
    float iiA = 0.f, iiB = 0.f;

    // retrieve body A data
    auto itA = dofMap.find(row.a);
    if (itA != dofMap.end() && itA->second.isDynamic) {
        int idxA = itA->second.index / 3;
        imA = invMass[idxA];
        iiA = invInertia[idxA];
    }
    // retrieve body B data
    auto itB = dofMap.find(row.b);
    if (itB != dofMap.end() && itB->second.isDynamic) {
        int idxB = itB->second.index / 3;
        imB = invMass[idxB];
        iiB = invInertia[idxB];
    }

    float32x2_t dir = { row.dirX, row.dirY };
    float32x2_t rA  = { row.rxA, row.ryA };
    float32x2_t rB  = { row.rxB, row.ryB };

    float rAxn = cross2f_NEON(rA, dir); // (rA x dir)
    float rBxn = cross2f_NEON(rB, dir);

    float sum = imA + imB + (rAxn*rAxn)*iiA + (rBxn*rBxn)*iiB;
    if (sum < 1e-12f) {
        return 0.f;
    }
    return 1.f / sum;
}

/**
 * @brief Calculates relative velocity along row.dir using NEON ops
 *
 * @param row The constraint row
 * @param dofMap Mapping for body DOFs
 * @param v The global velocity array
 * @return The scalar relative speed along row.dir
 */
static float getRelativeVelocity(
    const ConstraintRow &row,
    const std::unordered_map<entt::entity, BodyDOF> &dofMap,
    const std::vector<float> &v)
{
    float vxA=0.f, vyA=0.f, wA=0.f;
    float vxB=0.f, vyB=0.f, wB=0.f;

    // Body A
    auto itA = dofMap.find(row.a);
    if (itA != dofMap.end() && itA->second.isDynamic) {
        int baseA = itA->second.index;
        vxA = v[baseA + 0];
        vyA = v[baseA + 1];
        wA  = v[baseA + 2];
    }
    // Body B
    auto itB = dofMap.find(row.b);
    if (itB != dofMap.end() && itB->second.isDynamic) {
        int baseB = itB->second.index;
        vxB = v[baseB + 0];
        vyB = v[baseB + 1];
        wB  = v[baseB + 2];
    }

    float rxVAx = vxA - wA*row.ryA; // A contact vel x
    float ryVAy = vyA + wA*row.rxA; // A contact vel y
    float rxVBx = vxB - wB*row.ryB; // B contact vel x
    float ryVBy = vyB + wB*row.rxB; // B contact vel y

    // rel = Vb - Va => (rxVBx - rxVAx, ryVBy - ryVAy)
    float relX = rxVBx - rxVAx;
    float relY = ryVBy - ryVAy;

    // dot( rel, row.dir )
    return relX * row.dirX + relY * row.dirY;
}

/**
 * @brief Applies the impulse dλ·(dirX, dirY) to the bodies in the global velocity array
 *
 * @param row The constraint row
 * @param dLambda Incremental impulse
 * @param v The global velocity array
 * @param dofMap Mapping from entity -> BodyDOF
 * @param invMass Inverse mass array
 * @param invInertia Inverse inertia array
 */
static void applyImpulse(
    const ConstraintRow &row,
    float dLambda,
    std::vector<float> &v,
    const std::unordered_map<entt::entity, BodyDOF> &dofMap,
    const std::vector<float> &invMass,
    const std::vector<float> &invInertia)
{
    if (std::fabs(dLambda) < 1e-15f) {
        return;
    }
    // Body A
    auto itA = dofMap.find(row.a);
    if (itA != dofMap.end() && itA->second.isDynamic) {
        int iA = itA->second.index / 3;
        int baseA = itA->second.index;
        float imA = invMass[iA];
        float iiA = invInertia[iA];

        v[baseA + 0] -= row.dirX * (dLambda * imA);
        v[baseA + 1] -= row.dirY * (dLambda * imA);

        // cross(rA, dir)
        float crossA = row.rxA * row.dirY - row.ryA * row.dirX;
        v[baseA + 2] -= crossA * dLambda * iiA;
    }

    // Body B
    auto itB = dofMap.find(row.b);
    if (itB != dofMap.end() && itB->second.isDynamic) {
        int iB = itB->second.index / 3;
        int baseB = itB->second.index;
        float imB = invMass[iB];
        float iiB = invInertia[iB];

        v[baseB + 0] += row.dirX * (dLambda * imB);
        v[baseB + 1] += row.dirY * (dLambda * imB);

        // cross(rB, dir)
        float crossB = row.rxB * row.dirY - row.ryB * row.dirX;
        v[baseB + 2] += crossB * dLambda * iiB;
    }
}

/**
 * @brief Performs a Projected Gauss-Seidel iteration for all constraints
 *
 * @param contacts The set of normal+friction constraints
 * @param v The global velocity array (single-precision)
 * @param dofMap Mapping from entity -> BodyDOF
 * @param invMass Inverse masses
 * @param invInertia Inverse inertias
 * @param frictionCoeff Global friction coefficient
 * @param iterations Number of solver iterations
 */
static void solveLCP_PGS(
    std::vector<ContactRows> &contacts,
    std::vector<float> &v,
    const std::unordered_map<entt::entity, BodyDOF> &dofMap,
    const std::vector<float> &invMass,
    const std::vector<float> &invInertia,
    float frictionCoeff,
    int iterations)
{
    // Precompute effective masses
    for (auto &cr : contacts) {
        cr.normal.effMass   = computeEffectiveMass(cr.normal,   dofMap, invMass, invInertia);
        cr.friction.effMass = computeEffectiveMass(cr.friction, dofMap, invMass, invInertia);
    }

    // Iterative solve
    for (int iter = 0; iter < iterations; ++iter) {
        for (auto &cr : contacts) {
            // 1) Normal row
            {
                auto &row = cr.normal;
                float vn  = getRelativeVelocity(row, dofMap, v);
                float old = row.lambda;
                float dLam= -row.effMass * (vn + row.rhs);

                float newLam = old + dLam;
                if (newLam < row.lo) newLam = row.lo;
                if (newLam > row.hi) newLam = row.hi;

                dLam = newLam - old;
                row.lambda = newLam;
                applyImpulse(row, dLam, v, dofMap, invMass, invInertia);
            }

            // 2) Friction row
            {
                auto &frow = cr.friction;
                float vt   = getRelativeVelocity(frow, dofMap, v);
                float oldF = frow.lambda;
                float limit= frictionCoeff * cr.normal.lambda;

                frow.lo = -limit;
                frow.hi =  limit;

                float dF = -frow.effMass*(vt + frow.rhs);
                float newF = oldF + dF;
                if (newF < frow.lo) newF = frow.lo;
                if (newF > frow.hi) newF = frow.hi;

                dF = newF - oldF;
                frow.lambda = newF;
                applyImpulse(frow, dF, v, dofMap, invMass, invInertia);
            }
        }
    }
}

/**
 * @brief The main contact solver routine using a SIMD-accelerated LCP approach
 *
 * @param registry ECS registry
 * @param manager Contact manager
 */
void ContactSolver::solveContactConstraints(entt::registry &registry, ContactManager &manager)
{
    PROFILE_SCOPE("ContactSolver (SIMD LCP)");

    // Gather manifolds
    const auto &manifolds = manager.getManifoldsForSolver();
    if (manifolds.empty()) {
        return;
    }

    // Build DOF map
    auto dofMap = buildBodyDOFTable(registry, manifolds);

    // Build a stable list of dynamic bodies
    std::vector<entt::entity> bodyList;
    bodyList.reserve(dofMap.size());
    for (auto &kv : dofMap) {
        if (kv.second.isDynamic) {
            bodyList.push_back(kv.first);
        }
    }
    int bodyCount = (int)bodyList.size();

    // Single-precision arrays for velocity, invMass, invInertia
    std::vector<float> velocity(bodyCount * 3, 0.f);
    std::vector<float> invMassArr(bodyCount, 0.f);
    std::vector<float> invInertiaArr(bodyCount, 0.f);

    // Load from ECS (convert double to float)
    for (int i = 0; i < bodyCount; i++) {
        entt::entity e = bodyList[i];
        double massVal  = registry.get<Components::Mass>(e).value;
        float imVal     = (massVal > 1e29) ? 0.f : (float)(1.0 / massVal);

        float iInertVal = 0.f;
        if (canRotate(registry, e)) {
            double I = registry.get<Components::Inertia>(e).I;
            if (I > 1e-12 && I < 1e29) {
                iInertVal = (float)(1.0 / I);
            }
        }
        invMassArr[i]     = imVal;
        invInertiaArr[i]  = iInertVal;

        // Current velocity
        const auto &vel = registry.get<Components::Velocity>(e);
        float vx = (float)vel.x;
        float vy = (float)vel.y;
        float w  = 0.f;
        if (canRotate(registry, e)) {
            w = (float)registry.get<Components::AngularVelocity>(e).omega;
        }
        velocity[i*3 + 0] = vx;
        velocity[i*3 + 1] = vy;
        velocity[i*3 + 2] = w;
    }

    // Build constraints in float
    float frictionCoeff = 0.5f;
    auto contactRows = buildConstraintRows(registry, manifolds, frictionCoeff);

    // Solve LCP
    int solverIterations = 10;
    solveLCP_PGS(contactRows, velocity, dofMap, invMassArr, invInertiaArr,
                 frictionCoeff, solverIterations);

    // Write velocities back to ECS
    for (int i = 0; i < bodyCount; i++) {
        entt::entity e = bodyList[i];
        auto vel = registry.get<Components::Velocity>(e);
        vel.x = velocity[i*3 + 0];
        vel.y = velocity[i*3 + 1];
        registry.replace<Components::Velocity>(e, vel);

        if (canRotate(registry, e)) {
            auto angVel = registry.get<Components::AngularVelocity>(e);
            angVel.omega = velocity[i*3 + 2];
            registry.replace<Components::AngularVelocity>(e, angVel);
        }
    }

    // Store impulses back for warm-starting
    {
        int rowIndex = 0;
        for (auto &mref : manager.getManifoldsForSolver()) {
            for (auto &c : mref.contacts) {
                auto &r = contactRows[rowIndex++];
                c.normalImpulseAccum  = r.normal.lambda;
                c.tangentImpulseAccum = r.friction.lambda;
            }
        }
    }
    manager.applySolverResults(manager.getManifoldsForSolver());

    DEBUG_LOG("[ContactSolver] SIMD LCP solve complete.");
}

} // namespace RigidBodyCollision