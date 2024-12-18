#ifndef ECS_SIMULATOR_H
#define ECS_SIMULATOR_H

#include <entt/entt.hpp>
#include "nbody/core/constants.hpp"
#include "nbody/core/coordinates.hpp"

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

    entt::registry& getRegistry() { return registry; }
    const entt::registry& getRegistry() const { return registry; }
    SimulatorConstants::SimulationType getCurrentScenario() const { return currentScenario; }

    void createBoundaries();
};

#endif