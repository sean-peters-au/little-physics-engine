#ifndef BASIC_GRAVITY_SYSTEM_H
#define BASIC_GRAVITY_SYSTEM_H

#include <entt/entt.hpp>

namespace Systems {
    class BasicGravitySystem {
    public:
        // Apply a uniform gravity field (e.g., downward in y-direction)
        static void update(entt::registry& registry);
    };
}

#endif