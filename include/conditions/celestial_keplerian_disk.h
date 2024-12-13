#ifndef ECS_SIMULATOR_H
#define ECS_SIMULATOR_H

#include <entt/entt.hpp>
#include "components.h"
#include "coordinates.h"

class ECSSimulator {
private:
    entt::registry registry;
    // Particle generation functions
    void createCentralBody();
    void createKeplerianDisk();
    
    // Physics helper functions
    double calculateKeplerianVelocity(double radius_meters, double central_mass) const;
    double calculateDiskHeight(double radius, double max_radius) const;
    double calculateDiskDensity(double radius, double max_radius) const;

public:
    ECSSimulator();
    explicit ECSSimulator(CoordinateSystem* coordSystem);
    
    void init();
    void tick();
    
    // Accessor for the drawer
    const entt::registry& getRegistry() const { return registry; }
};

#endif