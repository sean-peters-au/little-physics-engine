/**
 * @fileoverview fluid.hpp
 * @brief SPH-based fluid solver system, now with integrated rigid-fluid impulse solver on the GPU.
 *
 * Provides data structures for particles, grid cells, parameters, and a GPU-friendly
 * rigid body struct with velocity, mass, inertia, bounding boxes, plus code for:
 *  - neighbor search
 *  - fluid density/pressure/forces
 *  - rigid-fluid impulse exchange
 *  - integrated position solvers
 * All in a single FluidSystem.
 */

#pragma once

#include <entt/entt.hpp>
#include <Metal/Metal.hpp>
#include <vector>
#include <functional>
#include <cstddef>
#include <limits>
#include "nbody/systems/i_system.hpp"

namespace Systems {

/**
 * @struct GPUFluidParticle
 * @brief GPU-friendly representation of a single fluid particle.
 *
 * Mirrors the data structures and logic in fluid_kernels.metal.
 */
struct GPUFluidParticle {
    float x;
    float y;
    float vx;
    float vy;
    float vxHalf;
    float vyHalf;
    float ax;
    float ay;
    float mass;
    float h;        ///< Smoothing length
    float c;        ///< Speed of sound
    float density;
    float pressure;
};

/**
 * @brief Maximum neighbors allowed in a single grid cell in GPU code.
 */
static constexpr int GPU_MAX_PER_CELL = 64;

/**
 * @struct GPUGridCell
 * @brief Stores the indices of particles that fall within a specific grid cell.
 */
struct GPUGridCell {
    int count;
    int indices[GPU_MAX_PER_CELL];
};

/**
 * @struct GPUFluidParams
 * @brief Parameters controlling the SPH and integration steps on the GPU.
 */
struct GPUFluidParams {
    float cellSize;
    int   gridMinX;
    int   gridMinY;
    int   gridDimX;
    int   gridDimY;
    float restDensity;
    float stiffness;
    float viscosity;
    float dt;
    float halfDt;
    unsigned int particleCount;
};

/**
 * @struct BBoxParams
 * @brief Input parameters for the GPU bounding box kernel.
 */
struct BBoxParams {
    int particleCount;
    int numThreadgroups;
};

/**
 * @enum GPURigidShapeType
 * @brief Enumerates shape types for GPU-based rigid bodies.
 */
enum class GPURigidShapeType : int {
    Circle = 0,
    Polygon = 1
};

/**
 * @struct GPURigidBody
 * @brief GPU-friendly representation of a rigid body, extended to store
 * bounding box, velocity, mass, and partial force/torque for the impulse solver.
 */
static constexpr int GPU_POLYGON_MAX_VERTS = 16;
struct GPURigidBody {
    // Basic shape data
    GPURigidShapeType shapeType;
    float posX;
    float posY;
    float angle;      ///< current orientation
    float radius;     ///< if Circle

    // Polygon data
    int vertCount;
    float vertsX[GPU_POLYGON_MAX_VERTS];
    float vertsY[GPU_POLYGON_MAX_VERTS];

    // Extended rigid-body properties
    float vx;         ///< linear velocity x
    float vy;         ///< linear velocity y
    float omega;      ///< angular velocity
    float mass;       ///< rigid mass
    float inertia;    ///< moment of inertia

    // Simple bounding box for broad-phase fluid check
    float minX;
    float maxX;
    float minY;
    float maxY;

    // Accumulated force/torque from fluid collisions
    float accumFx;
    float accumFy;
    float accumTorque;
};

/**
 * @struct FluidConfig
 * @brief Configuration parameters specific to the fluid simulation
 */
struct FluidConfig {
    // Fluid properties
    float restDensity = 0.5f;  // Default density of fluid (kg/m³)
    float stiffness = 50.0f;      // Pressure stiffness coefficient
    float viscosity = 0.03f;      // Viscosity coefficient
    float dampingFactor = 0.98f;  // Velocity damping for rigid bodies
    
    // Simulation parameters
    int numSubSteps = 10;         // Number of substeps per frame
    int threadsPerGroup = 256;    // Threads per threadgroup for GPU
};

/**
 * @class FluidSystem
 * @brief SPH-based fluid solver with GPU-accelerated neighbor search & rigid-fluid interactions.
 */
class FluidSystem : public ConfigurableSystem<FluidConfig> {
public:
    /**
     * @brief Constructor
     */
    FluidSystem();
    
    /**
     * @brief Destructor
     */
    ~FluidSystem() override;

    /**
     * @brief Main update function called each frame/tick.
     * @param registry The ECS registry containing fluid entities & rigid bodies.
     */
    void update(entt::registry &registry) override;

private:
    /**
     * @brief Creates a Metal compute pipeline state for a given function name.
     */
    MTL::ComputePipelineState* createPSO(const char* fnName, MTL::Library* lib);

    /**
     * @brief Dispatches a Metal compute pass with a typical command encoder setup.
     */
    void dispatchComputePass(
        MTL::ComputePipelineState* pipelineState,
        const std::function<void(MTL::ComputeCommandEncoder*)>& encodeFunc,
        size_t threadsCount,
        size_t threadsPerGroup = 256) const;

    /**
     * @brief Gathers fluid particles from the ECS into GPU-friendly structures.
     */
    std::vector<GPUFluidParticle> gatherFluidParticles(
        entt::registry& registry,
        std::vector<entt::entity>& entityList) const;

    /**
     * @brief Gathers rigid bodies from the ECS (including velocity, mass, bounding box) into GPU-friendly structures.
     */
    std::vector<GPURigidBody> gatherRigidBodies(
        entt::registry& registry,
        std::vector<entt::entity>& rigidEntityList) const;

    /**
     * @brief Part of Velocity Verlet: half-step velocity update.
     */
    void dispatchVelocityVerletHalf(
        MTL::Buffer* particleBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    /**
     * @brief GPU bounding box kernel to compute partial bounding boxes for each threadgroup.
     */
    void dispatchComputeBoundingBox(
        MTL::Buffer* particleBuf,
        MTL::Buffer* boundingBoxParamsBuf,
        MTL::Buffer* boundingBoxBuf,
        int realCount,
        int numThreadgroups) const;

    /**
     * @brief CPU-side reduction of partial bounding boxes.
     */
    void reduceBoundingBoxOnCPU(
        MTL::Buffer* boundingBoxBuf,
        int numThreadgroups,
        float& minX,
        float& maxX,
        float& minY,
        float& maxY) const;

    /**
     * @brief Clears the grid buffer on the GPU for neighbor searches.
     */
    void dispatchClearGrid(MTL::Buffer* gridBuf, int gridSize) const;

    /**
     * @brief Assigns each particle to a grid cell on the GPU.
     */
    void dispatchAssignCells(
        MTL::Buffer* particleBuf,
        MTL::Buffer* gridBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    /**
     * @brief Computes density for each particle using neighbor lookups in the grid.
     */
    void dispatchComputeDensity(
        MTL::Buffer* particleBuf,
        MTL::Buffer* gridBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    /**
     * @brief Computes forces (pressure, viscosity) for each particle.
     */
    void dispatchComputeForces(
        MTL::Buffer* particleBuf,
        MTL::Buffer* gridBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    /**
     * @brief Finishes Velocity Verlet step (velocity by half dt again, final position).
     */
    void dispatchVelocityVerletFinish(
        MTL::Buffer* particleBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    /**
     * @brief GPU-based position solver for fluid -> rigid boundary collisions (already stubbed).
     */
    void dispatchRigidFluidPosition(
        MTL::Buffer* particleBuf,
        int fluidCount,
        MTL::Buffer* rigidBuf,
        int rigidCount) const;

    /**
     * @brief **New**: GPU-based impulse solver for fluid->rigid forces (like drag, buoyancy).
     */
    void dispatchRigidFluidImpulse(
        MTL::Buffer* particleBuf,
        int fluidCount,
        MTL::Buffer* rigidBuf,
        MTL::Buffer* paramsBuf,
        int rigidCount) const;

    /**
     * @brief Writes updated fluid positions & velocities back to ECS.
     */
    void writeBackToECS(
        entt::registry& registry,
        const std::vector<entt::entity>& entityList,
        MTL::Buffer* particleBuf,
        int realCount) const;

    /**
     * @brief Writes updated rigid velocities/omegas back to ECS after GPU impulses.
     */
    void writeBackRigidBodies(
        entt::registry& registry,
        const std::vector<entt::entity>& rigidEntityList,
        MTL::Buffer* rigidBuf,
        int rigidCount) const;

    /**
     * @brief Multi-step integration that calls the sub-kernels, including the new rigid-fluid impulse solver.
     */
    void multiStepVelocityVerlet(
        entt::registry& registry,
        std::vector<entt::entity>& fluidEntities,
        std::vector<entt::entity>& rigidEntities,
        MTL::Buffer* particleBuf,
        MTL::Buffer* paramsBuf,
        MTL::Buffer* boundingBoxParamsBuf,
        MTL::Buffer* boundingBoxBuf,
        MTL::Buffer* rigidBuf,
        int realCount,
        int paddedCount,
        int rigidCount);

private:
    MTL::Device* device = nullptr;
    MTL::CommandQueue* commandQueue = nullptr;
    MTL::Library* metalLibrary = nullptr;  ///< Keep the library alive while pipeline states are in use

    // Pipeline states
    MTL::ComputePipelineState* clearGridPSO          = nullptr;
    MTL::ComputePipelineState* assignCellsPSO        = nullptr;
    MTL::ComputePipelineState* computeDensityPSO     = nullptr;
    MTL::ComputePipelineState* computeForcesPSO      = nullptr;
    MTL::ComputePipelineState* verletHalfPSO         = nullptr;
    MTL::ComputePipelineState* verletFinishPSO       = nullptr;
    MTL::ComputePipelineState* computeBoundingBoxPSO = nullptr;

    // Existing GPU-based position solver
    MTL::ComputePipelineState* rigidFluidPositionPSO = nullptr;

    // **New**: Impulse solver pipeline
    MTL::ComputePipelineState* rigidFluidImpulsePSO  = nullptr;
};

} // namespace Systems