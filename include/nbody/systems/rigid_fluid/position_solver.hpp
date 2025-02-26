#pragma once

#include <entt/entt.hpp>

namespace Systems {

class RigidFluidPositionSolver {
public:
    static void update(entt::registry &registry);
};

} // namespace Systems
