#ifndef SOLID_COLLISION_RESPONSE_SYSTEM_HPP
#define SOLID_COLLISION_RESPONSE_SYSTEM_HPP

#include <entt/entt.hpp>
#include "nbody/systems/collision/collision_data.hpp"

namespace Systems {
    class SolidCollisionResponseSystem {
    public:
        static void update(entt::registry &registry, CollisionManifold &manifold);
    };
}

#endif