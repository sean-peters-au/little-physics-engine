#include "nbody/systems/collision/narrow_phase_system.hpp"
#include "nbody/algo/gjk.hpp"
#include "nbody/algo/epa.hpp"
#include "nbody/math/polygon.hpp"
#include <cmath>

// Convert entt entities to ShapeData
static ShapeData extractShapeData(entt::registry &reg, entt::entity e) {
    ShapeData sd;
    sd.pos = reg.get<Components::Position>(e);
    sd.angle = 0.0;
    if (reg.any_of<Components::AngularPosition>(e)) {
        sd.angle = reg.get<Components::AngularPosition>(e).angle;
    }

    if (reg.any_of<CircleShape>(e)) {
        sd.isCircle = true;
        sd.radius = reg.get<CircleShape>(e).radius;
    } else {
        sd.isCircle = false;
        sd.radius = 0.0;
        sd.poly = reg.get<PolygonShape>(e);
    }
    return sd;
}

void Systems::NarrowPhaseSystem::update(entt::registry &registry, 
                                        const std::vector<CandidatePair> &candidatePairs,
                                        CollisionManifold &manifold) {
    manifold.clear();

    for (auto &pair : candidatePairs) {
        if (!registry.valid(pair.eA) || !registry.valid(pair.eB)) continue;

        auto sdA = extractShapeData(registry, pair.eA);
        auto sdB = extractShapeData(registry, pair.eB);

        Simplex simplex;
        if (GJKIntersect(sdA, sdB, simplex)) {
            auto epaRes = EPA(sdA, sdB, simplex);
            if (epaRes.has_value()) {
                EPAResult res = epaRes.value();
                // Compute a contact point approximation:
                // Move A by half penetration along -normal
                // Move B by half penetration along normal
                // Then midpoint
                Vector cA(sdA.pos.x, sdA.pos.y);
                Vector cB(sdB.pos.x, sdB.pos.y);
                Vector halfPen = res.normal*(res.penetration*0.5);
                cA = cA - halfPen;
                cB = cB + halfPen;
                Vector contact = (cA + cB)*0.5;

                CollisionInfo info;
                info.a = pair.eA;
                info.b = pair.eB;
                info.normal = res.normal;
                info.penetration = res.penetration;
                info.contactPoint = contact;
                manifold.collisions.push_back(info);
            }
        }
    }
}