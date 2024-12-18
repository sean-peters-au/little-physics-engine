#ifndef SLEEP_SYSTEM_HPP
#define SLEEP_SYSTEM_HPP

#include <entt/entt.hpp>

namespace Systems {
    class SleepSystem {
    public:
        static void update(entt::registry &registry);
    };
}

#endif