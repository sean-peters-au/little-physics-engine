#ifndef COLLISION_SYSTEM_H
#define COLLISION_SYSTEM_H

#include <entt/entt.hpp>
#include <vector>
#include "components.h"
#include "simulator_constants.h"

namespace Systems {
    class CollisionSystem {
    public:
        // Update collisions and interactions among particles
        static void update(entt::registry& registry);
    };
}

#endif