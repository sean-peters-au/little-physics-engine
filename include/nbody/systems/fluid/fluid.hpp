/**
 * @fileoverview fluid.hpp
 * @brief SPH-based fluid solver system, using Metal for GPU acceleration.
 *
 * Provides data structures for particles, grid cells, parameters, and a new
 * stub for rigid body data, plus the FluidSystem class definition. The system
 * now also contains a stubbed GPU-based rigid-fluid position solver pipeline.
 */

#pragma once

#include <entt/entt.hpp>
#include <Metal/Metal.hpp>
#include <vector>
#include <functional>
#include <cstddef>
#include <limits>

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
 *
 * The GPU kernel writes out partial bounding boxes per threadgroup,
 * which are then reduced on the CPU.
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
 * @brief GPU-friendly representation of a rigid body for the position solver.
 *
 * This is a stub for now: we only store minimal data needed to compile.
 * We'll expand it later to actually perform collision resolution.
 */
static constexpr int GPU_POLYGON_MAX_VERTS = 16;  ///< Example limit for polygon vertices on the GPU
struct GPURigidBody {
    GPURigidShapeType shapeType;
    float posX;
    float posY;
    float angle;
    float radius; ///< Used if shapeType == Circle

    // Polygon data (stub)
    int vertCount;
    float vertsX[GPU_POLYGON_MAX_VERTS];
    float vertsY[GPU_POLYGON_MAX_VERTS];

    // Future expansions could include velocity, mass, etc.
};

/**
 * @class FluidSystem
 * @brief SPH-based fluid solver system, using single-precision arrays and a uniform grid for neighbor search.
 *
 * - Gathers all Liquid-phase particles.
 * - Builds a grid to find neighbors.
 * - Computes densities and pressures on the GPU.
 * - Computes forces (pressure, gravity, viscosity) on the GPU.
 * - Integrates velocities and positions over multiple sub-steps.
 * - Gathers rigid bodies (Circle/Polygon) and runs a stub GPU-based solver to correct fluid positions.
 * - Writes the final particle states (and eventually rigid states) back to the ECS registry.
 */
class FluidSystem {
public:
    FluidSystem();
    ~FluidSystem();

    /**
     * @brief Main update function called each frame/tick.
     * @param registry The ECS registry containing fluid entities.
     */
    void update(entt::registry &registry);

private:
    /**
     * @brief Creates a Metal compute pipeline state for a given function name.
     * @param fnName Name of the Metal shader function
     * @param lib    The Metal library object
     * @return Valid pipeline state pointer on success, nullptr on failure
     */
    MTL::ComputePipelineState* createPSO(const char* fnName, MTL::Library* lib);

    /**
     * @brief Dispatches a Metal compute pass to the GPU, with a typical command encoder setup.
     * @param pipelineState  The compiled compute pipeline
     * @param encodeFunc     A lambda to bind buffers and set arguments
     * @param threadsCount   Total threads to launch
     * @param threadsPerGroup Threads in each threadgroup
     */
    void dispatchComputePass(
        MTL::ComputePipelineState* pipelineState,
        const std::function<void(MTL::ComputeCommandEncoder*)>& encodeFunc,
        size_t threadsCount,
        size_t threadsPerGroup = 256) const;

    /**
     * @brief Gathers fluid particles (Phase::Liquid) from the ECS into GPU-friendly structures.
     * @param registry   ECS registry
     * @param entityList Output list of entities in the same order as gathered particles
     * @return A vector of GPUFluidParticle
     */
    std::vector<GPUFluidParticle> gatherFluidParticles(
        entt::registry& registry,
        std::vector<entt::entity>& entityList) const;

    /**
     * @brief Gathers rigid bodies from the ECS for the GPU position solver.
     * @param registry    ECS registry
     * @param rigidEntityList Output list of rigid entities for possible future write-back
     * @return A vector of GPURigidBody
     */
    std::vector<GPURigidBody> gatherRigidBodies(
        entt::registry& registry,
        std::vector<entt::entity>& rigidEntityList) const;

    /**
     * @brief Executes half of the Velocity Verlet integration (updates velocity by half dt).
     */
    void dispatchVelocityVerletHalf(
        MTL::Buffer* particleBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    /**
     * @brief Executes a bounding box kernel to compute partial bounding boxes for each threadgroup.
     */
    void dispatchComputeBoundingBox(
        MTL::Buffer* particleBuf,
        MTL::Buffer* boundingBoxParamsBuf,
        MTL::Buffer* boundingBoxBuf,
        int realCount,
        int numThreadgroups) const;

    /**
     * @brief Combines (reduces) partial bounding boxes produced by the GPU into a final bounding box on the CPU.
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
     * @brief Computes density for each particle, using neighbor lookups in the grid.
     */
    void dispatchComputeDensity(
        MTL::Buffer* particleBuf,
        MTL::Buffer* gridBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    /**
     * @brief Computes forces on each particle (pressure, viscosity), stored in the particle array.
     */
    void dispatchComputeForces(
        MTL::Buffer* particleBuf,
        MTL::Buffer* gridBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    /**
     * @brief Completes the Velocity Verlet step (updates velocity by half dt, then position).
     */
    void dispatchVelocityVerletFinish(
        MTL::Buffer* particleBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    /**
     * @brief Dispatches a GPU kernel to perform rigid-fluid position corrections (stubbed out).
     */
    void dispatchRigidFluidPosition(
        MTL::Buffer* particleBuf,
        int fluidCount,
        MTL::Buffer* rigidBuf,
        int rigidCount) const;

    /**
     * @brief Writes updated particle positions and velocities back to the ECS.
     */
    void writeBackToECS(
        entt::registry& registry,
        const std::vector<entt::entity>& entityList,
        MTL::Buffer* particleBuf,
        int realCount) const;

    /**
     * @brief Potentially writes updated rigid body transforms back to the ECS (stubbed out).
     */
    void writeBackRigidBodies(
        entt::registry& registry,
        const std::vector<entt::entity>& rigidEntityList,
        MTL::Buffer* rigidBuf,
        int rigidCount) const;

    /**
     * @brief Performs multiple sub-steps of the integration and neighbor computations,
     *        ensuring better stability for the fluid sim. Also includes rigid-fluid
     *        position solver calls in between or after sub-steps.
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

    // Pipeline states for various GPU kernels
    MTL::ComputePipelineState* clearGridPSO = nullptr;
    MTL::ComputePipelineState* assignCellsPSO = nullptr;
    MTL::ComputePipelineState* computeDensityPSO = nullptr;
    MTL::ComputePipelineState* computeForcesPSO = nullptr;
    MTL::ComputePipelineState* verletHalfPSO = nullptr;
    MTL::ComputePipelineState* verletFinishPSO = nullptr;
    MTL::ComputePipelineState* computeBoundingBoxPSO = nullptr;

    // New pipeline for rigid-fluid position solver (stub)
    MTL::ComputePipelineState* rigidFluidPositionPSO = nullptr;

    static constexpr int N_SUB_STEPS = 10; ///< Number of sub-steps per tick
};

} // namespace Systems