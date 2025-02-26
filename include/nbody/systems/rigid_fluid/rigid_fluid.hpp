#pragma once

#include <entt/entt.hpp>

namespace Systems {

class RigidFluidSystem {
public:
    static void update(entt::registry &registry);
};

} // namespace Systems
