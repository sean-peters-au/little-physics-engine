#ifndef ECS_SIMULATOR_H
#define ECS_SIMULATOR_H

#include <entt/entt.hpp>
#include "simulator_constants.h"
#include "coordinates.h"

class ECSSimulator {
private:
    entt::registry registry;
    SimulatorConstants::SimulationType currentScenario;

    // Particle generation functions
    void createCentralBody();
    void createKeplerianDisk();
    void createIsothermalBox();
    void createBouncyBalls();

    // Physics helper functions
    double calculateKeplerianVelocity(double radius_meters, double central_mass) const;
    double calculateDiskHeight(double radius, double max_radius) const;
    double calculateDiskDensity(double radius, double max_radius) const;

public:
    ECSSimulator();
    explicit ECSSimulator(CoordinateSystem* coordSystem);

    void init();
    void tick();

    void setScenario(SimulatorConstants::SimulationType scenario);
    void reset();

    const entt::registry& getRegistry() const { return registry; }
    SimulatorConstants::SimulationType getCurrentScenario() const { return currentScenario; }
};

#endif