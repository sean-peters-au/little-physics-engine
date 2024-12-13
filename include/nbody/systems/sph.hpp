#ifndef SPH_SYSTEM_H
#define SPH_SYSTEM_H

#include <entt/entt.hpp>

namespace Systems {
    class SPHSystem {
    public:
        // Update SPH: density, pressure, and viscosity forces
        // Should be called after gravitational forces are computed and before movement.
        static void update(entt::registry& registry);

    private:
        // Internal methods
        static void computeDensity(entt::registry& registry);
        static void computePressure(entt::registry& registry);
        static void computeForces(entt::registry& registry);
        static void applyForces(entt::registry& registry);

        // Neighbor search and kernel support
        struct Cell {
            std::vector<entt::entity> particles;
            void clear() { particles.clear(); }
        };

        using Grid = std::vector<std::vector<Cell>>;

        static Grid createGrid();
        static void populateGrid(Grid& grid, entt::registry& registry);
        static void computeDensityForParticle(entt::entity e, entt::registry& registry, const Grid& grid);
        static void computeForcesForParticle(entt::entity e, entt::registry& registry, const Grid& grid);

        // SPH kernel functions
        static double W_poly6(double r, double h);
        static double gradW_spiky(double r, double h);
        static double lapW_visc(double r, double h);

        // Artificial viscosity parameters
        static constexpr double alpha = 0.1; // Adjust as needed
        static constexpr double beta = 0.0;  // Adjust as needed

        // Helper methods
        static void findNeighbors(entt::entity e, entt::registry& registry, const Grid& grid,
                                  std::vector<entt::entity>& neighbors);
    };
}

#endif