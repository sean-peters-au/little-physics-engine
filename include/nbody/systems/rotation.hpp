#ifndef ROTATION_SYSTEM_H
#define ROTATION_SYSTEM_H

#include <entt/entt.hpp>

namespace Systems {
    class RotationSystem {
    public:
        static void update(entt::registry &registry);
    };
}

#endif