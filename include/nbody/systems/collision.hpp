#ifndef COLLISION_SYSTEM_H
#define COLLISION_SYSTEM_H

#include <entt/entt.hpp>
#include <vector>
#include "nbody/components/basic.hpp"
#include "nbody/core/constants.hpp"

namespace Systems {
    class CollisionSystem {
    public:
        // Update collisions and interactions among particles
        static void update(entt::registry& registry);
    };
}

#endif