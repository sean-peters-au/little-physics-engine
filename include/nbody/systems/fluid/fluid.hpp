#pragma once

#include <entt/entt.hpp>
#include <Metal/Metal.hpp>

namespace Systems {
    
/**
 * @class FluidSystem
 * @brief SPH-based fluid solver system, using single-precision arrays and a uniform grid for neighbor search.
 *
 * - Gathers all Liquid-phase particles that have SmoothingLength, SPHTemp, etc.
 * - Builds a grid to find neighbors.
 * - Computes densities and pressures.
 * - Computes forces (pressure, gravity, possibly viscosity).
 * - Writes force back to velocities in ECS.
 *
 * The fluid can eventually interact with rigid bodies by querying collisions or by
 * applying pressure on boundary shapes (future extension).
 */
class FluidSystem {
public:
    /**
     * @brief Constructor
     */
    FluidSystem();

    /**
     * @brief Destructor
     */
    ~FluidSystem();

    /**
     * @brief Main update function called each frame/tick
     * @param registry ECS registry
     */
    void update(entt::registry &registry);

private:
    // metal objects
    MTL::Device*            device         = nullptr;
    MTL::CommandQueue*      commandQueue   = nullptr;
    MTL::Library*           metalLibrary   = nullptr;  // <-- Keep the library alive!
    
    // pipeline states
    MTL::ComputePipelineState* clearGridPSO      = nullptr;
    MTL::ComputePipelineState* assignCellsPSO    = nullptr;
    MTL::ComputePipelineState* computeDensityPSO = nullptr;
    MTL::ComputePipelineState* computeForcesPSO  = nullptr;
    MTL::ComputePipelineState* verletHalfPSO     = nullptr;
    MTL::ComputePipelineState* verletFinishPSO   = nullptr;

    MTL::ComputePipelineState* createPSO(const char* fnName, MTL::Library* lib);
};

} // namespace Systems
