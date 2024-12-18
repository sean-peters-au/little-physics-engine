#ifndef LIQUID_COLLISION_RESPONSE_SYSTEM_HPP
#define LIQUID_COLLISION_RESPONSE_SYSTEM_HPP

#include <entt/entt.hpp>
#include "nbody/systems/collision/collision_data.hpp"

namespace Systems {
    class LiquidCollisionResponseSystem {
    public:
        static void update(entt::registry &registry, CollisionManifold &manifold);
    };
}

#endif