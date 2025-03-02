/**
 * @fileoverview contact_solver.cpp
 * @brief SIMD-optimized LCP solver for rigid body contact resolution on Apple Silicon
 *
 * This solver builds a global LCP containing both normal and friction constraints,
 * then solves it using a Projected Gauss-Seidel approach. It accelerates critical
 * vector math (dot products, cross products) with NEON intrinsics on the M2's ARM
 * architecture, reducing the time spent per iteration.
 */

#include <arm_neon.h>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "nbody/systems/rigid/contact_solver.hpp"
#include "nbody/systems/rigid/contact_manager.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/core/profile.hpp"

namespace RigidBodyCollision
{

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
    if (!registry.valid(e) || !registry.all_of<Components::Mass>(e)) { return false; }
    double const m = registry.get<Components::Mass>(e).value;
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
    if (!registry.all_of<Components::AngularVelocity, Components::Inertia>(e)) { return false; }
    double const i = registry.get<Components::Inertia>(e).I;
    return (i > 1e-12 && i < 1e29);
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
        entt::entity const e = kv.first;
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
 * @brief Builds normal + friction constraints for all contacts in single-precision
 *
 * @param registry ECS registry
 * @param manifolds The collision manifold references
 * @return Vector of ContactRows
 */
static std::vector<ContactRows> buildConstraintRows(
    entt::registry &registry,
    const std::vector<ContactManifoldRef> &manifolds)
{
    std::vector<ContactRows> out;
    out.reserve(manifolds.size() * 4);

    for (const auto &mref : manifolds) {
        for (const auto &c : mref.contacts) {
            // Normal row
            ConstraintRow rowN;
            rowN.a = c.a;
            rowN.b = c.b;

            // Convert normal to single-precision
            Vector const unitN = c.normal.normalized();
            rowN.dirX = static_cast<float>(unitN.x);
            rowN.dirY = static_cast<float>(unitN.y);

            // Positions
            auto posA = registry.get<Components::Position>(c.a);
            auto posB = registry.get<Components::Position>(c.b);

            // Offsets rA, rB
            rowN.rxA = static_cast<float>(c.contactPoint.x - posA.x);
            rowN.ryA = static_cast<float>(c.contactPoint.y - posA.y);
            rowN.rxB = static_cast<float>(c.contactPoint.x - posB.x);
            rowN.ryB = static_cast<float>(c.contactPoint.y - posB.y);

            // Normal constraint bounds
            rowN.lo = 0.0F;
            rowN.hi = 1e20F;
            rowN.lambda = static_cast<float>(c.normalImpulseAccum);
            rowN.rhs    = 0.0F;  // no restitution
            rowN.effMass= 0.0F;

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
            rowF.lo     = -1e20F;
            rowF.hi     =  1e20F;
            rowF.lambda = static_cast<float>(c.tangentImpulseAccum);
            rowF.rhs    = 0.0F;
            rowF.effMass= 0.0F;

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
inline static float cross2fNeon(const float32x2_t &a, const float32x2_t &b)
{
    // cross = a.x * b.y - a.y * b.x
    float32x2_t const mul1 = vmul_f32(a, vrev64_f32(b)); // a.x*b.y, a.y*b.x
    // cross in lane0 - lane1
    float const crossVal = vget_lane_f32(mul1, 0) - vget_lane_f32(mul1, 1);
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
    float imA = 0.f;
    float imB = 0.f;
    float iiA = 0.f;
    float iiB = 0.f;

    // retrieve body A data
    auto itA = dofMap.find(row.a);
    if (itA != dofMap.end() && itA->second.isDynamic) {
        int const idxA = itA->second.index / 3;
        imA = invMass[idxA];
        iiA = invInertia[idxA];
    }
    // retrieve body B data
    auto itB = dofMap.find(row.b);
    if (itB != dofMap.end() && itB->second.isDynamic) {
        int const idxB = itB->second.index / 3;
        imB = invMass[idxB];
        iiB = invInertia[idxB];
    }

    float32x2_t const dir = { row.dirX, row.dirY };
    float32x2_t const rA  = { row.rxA, row.ryA };
    float32x2_t const rB  = { row.rxB, row.ryB };

    float const rAxn = cross2fNeon(rA, dir); // (rA x dir)
    float const rBxn = cross2fNeon(rB, dir);

    float const sum = imA + imB + (rAxn*rAxn)*iiA + (rBxn*rBxn)*iiB;
    if (sum < 1e-12F) {
        return 0.F;
    }
    return 1.F / sum;
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
    float vxA=0.f;
    float vyA=0.f;
    float wA=0.f;
    float vxB=0.f;
    float vyB=0.f;
    float wB=0.f;

    // Body A
    auto itA = dofMap.find(row.a);
    if (itA != dofMap.end() && itA->second.isDynamic) {
        int const baseA = itA->second.index;
        vxA = v[baseA + 0];
        vyA = v[baseA + 1];
        wA  = v[baseA + 2];
    }
    // Body B
    auto itB = dofMap.find(row.b);
    if (itB != dofMap.end() && itB->second.isDynamic) {
        int const baseB = itB->second.index;
        vxB = v[baseB + 0];
        vyB = v[baseB + 1];
        wB  = v[baseB + 2];
    }

    float const rxVAx = vxA - wA*row.ryA; // A contact vel x
    float const ryVAy = vyA + wA*row.rxA; // A contact vel y
    float const rxVBx = vxB - wB*row.ryB; // B contact vel x
    float const ryVBy = vyB + wB*row.rxB; // B contact vel y

    // rel = Vb - Va => (rxVBx - rxVAx, ryVBy - ryVAy)
    float const relX = rxVBx - rxVAx;
    float const relY = ryVBy - ryVAy;

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
    if (std::fabs(dLambda) < 1e-15F) {
        return;
    }
    // Body A
    auto itA = dofMap.find(row.a);
    if (itA != dofMap.end() && itA->second.isDynamic) {
        int const iA = itA->second.index / 3;
        int const baseA = itA->second.index;
        float const imA = invMass[iA];
        float const iiA = invInertia[iA];

        v[baseA + 0] -= row.dirX * (dLambda * imA);
        v[baseA + 1] -= row.dirY * (dLambda * imA);

        // cross(rA, dir)
        float const crossA = row.rxA * row.dirY - row.ryA * row.dirX;
        v[baseA + 2] -= crossA * dLambda * iiA;
    }

    // Body B
    auto itB = dofMap.find(row.b);
    if (itB != dofMap.end() && itB->second.isDynamic) {
        int const iB = itB->second.index / 3;
        int const baseB = itB->second.index;
        float const imB = invMass[iB];
        float const iiB = invInertia[iB];

        v[baseB + 0] += row.dirX * (dLambda * imB);
        v[baseB + 1] += row.dirY * (dLambda * imB);

        // cross(rB, dir)
        float const crossB = row.rxB * row.dirY - row.ryB * row.dirX;
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
static void solveLcpPgs(
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
                float const vn  = getRelativeVelocity(row, dofMap, v);
                float const old = row.lambda;
                float dLam= -row.effMass * (vn + row.rhs);

                float newLam = old + dLam;
                if (newLam < row.lo) { newLam = row.lo;
}
                if (newLam > row.hi) { newLam = row.hi;
}

                dLam = newLam - old;
                row.lambda = newLam;
                applyImpulse(row, dLam, v, dofMap, invMass, invInertia);
            }

            // 2) Friction row
            {
                auto &frow = cr.friction;
                float const vt   = getRelativeVelocity(frow, dofMap, v);
                float const oldF = frow.lambda;
                float const limit= frictionCoeff * cr.normal.lambda;

                frow.lo = -limit;
                frow.hi =  limit;

                float dF = -frow.effMass*(vt + frow.rhs);
                float newF = oldF + dF;
                if (newF < frow.lo) { newF = frow.lo;
}
                if (newF > frow.hi) { newF = frow.hi;
}

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
 * @param config Contact solver configuration
 */
void ContactSolver::solveContactConstraints(
    entt::registry &registry, 
    ContactManager &manager,
    const ContactSolverConfig &config)
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
    int const bodyCount = static_cast<int>(bodyList.size());

    // Single-precision arrays for velocity, invMass, invInertia
    std::vector<float> velocity(static_cast<std::vector<float>::size_type>(bodyCount * 3), 0.F);
    std::vector<float> invMassArr(bodyCount, 0.F);
    std::vector<float> invInertiaArr(bodyCount, 0.F);

    // Load from ECS (convert double to float)
    for (int i = 0; i < bodyCount; i++) {
        entt::entity const e = bodyList[i];
        double const massVal  = registry.get<Components::Mass>(e).value;
        float const imVal     = (massVal > 1e29) ? 0.F : static_cast<float>(1.0 / massVal);

        float iInertVal = 0.F;
        if (canRotate(registry, e)) {
            double const i = registry.get<Components::Inertia>(e).I;
            if (i > 1e-12 && i < 1e29) {
                iInertVal = static_cast<float>(1.0 / i);
            }
        }
        invMassArr[i]     = imVal;
        invInertiaArr[i]  = iInertVal;

        // Current velocity
        const auto &vel = registry.get<Components::Velocity>(e);
        auto const vx = static_cast<float>(vel.x);
        auto const vy = static_cast<float>(vel.y);
        float w  = 0.F;
        if (canRotate(registry, e)) {
            w = static_cast<float>(registry.get<Components::AngularVelocity>(e).omega);
        }
        velocity[i*3 + 0] = vx;
        velocity[i*3 + 1] = vy;
        velocity[i*3 + 2] = w;
    }

    // Build constraints in float
    auto contactRows = buildConstraintRows(registry, manifolds);

    // Solve LCP using configuration values
    solveLcpPgs(contactRows, velocity, dofMap, invMassArr, invInertiaArr,
                config.frictionCoeff, config.iterations);

    // Write velocities back to ECS
    for (int i = 0; i < bodyCount; i++) {
        entt::entity const e = bodyList[i];
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
}

} // namespace RigidBodyCollision