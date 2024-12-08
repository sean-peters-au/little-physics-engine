#ifndef ECS_SIMULATOR_H
#define ECS_SIMULATOR_H

#include <entt/entt.hpp>
#include "components.h"
#include "coordinates.h"

class ECSSimulator {
private:
    entt::registry registry;

public:
    ECSSimulator();
    explicit ECSSimulator(CoordinateSystem* coordSystem);
    
    void generateParticles();
    void init();
    void tick();
    
    // Accessor for the drawer
    const entt::registry& getRegistry() const { return registry; }
};

#endif