#ifndef MOVEMENT_SYSTEM_HPP
#define MOVEMENT_SYSTEM_HPP

#include <entt/entt.hpp>

namespace Systems {
    class MovementSystem {
    public:
        static void update(entt::registry &registry);
    };
}

#endif