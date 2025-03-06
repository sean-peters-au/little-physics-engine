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
 * All in a single FluidSystem implementation. Updated to help avoid bus errors
 * by rechecking buffer sizes and clamping bounding box dimensions.
 */

#pragma once

#include <entt/entt.hpp>
#include <Metal/Metal.hpp>
#include <vector>
#include <functional>
#include <cstddef>
#include <limits>

#include "systems/i_system.hpp"

namespace Systems
{

/**
 * @struct GPUFluidParticle
 * @brief GPU-friendly representation of a single fluid particle.
 *
 * Mirrors the data structures and logic in fluid_kernels.metal.
 */
struct GPUFluidParticle
{
    float x;
    float y;
    float vx;
    float vy;
    float vxHalf;
    float vyHalf;
    float ax;
    float ay;
    float mass;
    float h;         ///< Smoothing length
    float c;         ///< Speed of sound
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
struct GPUGridCell
{
    int count;
    int indices[GPU_MAX_PER_CELL];
};

/**
 * @struct GPUFluidParams
 * @brief Parameters controlling the SPH and integration steps on the GPU.
 */
struct GPUFluidParams
{
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
struct BBoxParams
{
    int particleCount;
    int numThreadgroups;
};

/**
 * @enum GPURigidShapeType
 * @brief Enumerates shape types for GPU-based rigid bodies.
 */
enum class GPURigidShapeType : int
{
    Circle = 0,
    Polygon = 1
};

/**
 * @struct GPURigidBody
 * @brief GPU-friendly representation of a rigid body, extended to store
 * bounding box, velocity, mass, and partial force/torque for the impulse solver.
 */
static constexpr int GPU_POLYGON_MAX_VERTS = 16;
struct GPURigidBody
{
    // Basic shape data
    GPURigidShapeType shapeType;
    float posX;
    float posY;
    float angle;      ///< Current orientation
    float radius;     ///< If Circle

    // Polygon data
    int vertCount;
    float vertsX[GPU_POLYGON_MAX_VERTS];
    float vertsY[GPU_POLYGON_MAX_VERTS];

    // Extended rigid-body properties
    float vx;         ///< Linear velocity x
    float vy;         ///< Linear velocity y
    float omega;      ///< Angular velocity
    float mass;       ///< Rigid mass
    float inertia;    ///< Moment of inertia

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
 * @brief Configuration parameters specific to the fluid simulation.
 */
struct FluidConfig
{
    float restDensity = 0.5f;    ///< Default density of fluid (kg/m³)
    float stiffness = 50.0f;     ///< Pressure stiffness coefficient
    float viscosity = 0.03f;     ///< Viscosity coefficient
    float dampingFactor = 0.98f; ///< Velocity damping for rigid bodies

    int numSubSteps = 10;        ///< Number of substeps per frame
    int threadsPerGroup = 256;   ///< Threads per threadgroup for GPU
};

/**
 * @class FluidSystem
 * @brief SPH-based fluid solver with GPU-accelerated neighbor search & rigid-fluid interactions.
 *
 * This class has been refactored to reduce CPU-GPU synchronization by batching passes
 * into fewer command buffers and to reuse buffers across frames to minimize allocations.
 * We also clamp the bounding box and re-check our grid buffer sizes each sub-step
 * to avoid bus errors.
 */
class FluidSystem : public ConfigurableSystem<FluidConfig>
{
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
     * @brief Encodes a compute pass into an existing command encoder without committing.
     * @param enc The active compute command encoder
     * @param pipelineState The pipeline state to use
     * @param encodeFunc Lambda to bind buffer arguments
     * @param threadsCount Number of threads to dispatch
     * @param threadsPerGroup Threads per threadgroup
     */
    void encodeComputePass(
        MTL::ComputeCommandEncoder* enc,
        MTL::ComputePipelineState* pipelineState,
        const std::function<void(MTL::ComputeCommandEncoder*)>& encodeFunc,
        size_t threadsCount,
        size_t threadsPerGroup) const;

    /**
     * @brief Allocates or resizes internal GPU buffers if needed, based on the current simulation size.
     * @param fluidCount Number of fluid particles
     * @param paddedCount Padded fluid count
     * @param rigidCount Number of rigid bodies
     * @param threadgroups Number of bounding box partials
     * @param gridSize Current grid size (gridDimX * gridDimY)
     */
    void initBuffersIfNeeded(
        int fluidCount,
        int paddedCount,
        int rigidCount,
        int threadgroups,
        int gridSize);

    /**
     * @brief Gathers fluid particles from the ECS into GPU-friendly structures.
     */
    std::vector<GPUFluidParticle> gatherFluidParticles(
        entt::registry& registry,
        std::vector<entt::entity>& entityList) const;

    /**
     * @brief Gathers rigid bodies from the ECS into GPU-friendly structures.
     */
    std::vector<GPURigidBody> gatherRigidBodies(
        entt::registry& registry,
        std::vector<entt::entity>& rigidEntityList) const;

    /**
     * @brief CPU-side reduction of partial bounding boxes from the GPU kernel.
     * @param boundingBoxBuf GPU buffer with partial bounding boxes
     * @param numThreadgroups Number of partial bounding boxes
     * @param[out] minX Global min x
     * @param[out] maxX Global max x
     * @param[out] minY Global min y
     * @param[out] maxY Global max y
     */
    void reduceBoundingBoxOnCPU(
        MTL::Buffer* boundingBoxBuf,
        int numThreadgroups,
        float& minX,
        float& maxX,
        float& minY,
        float& maxY) const;

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
     * @brief Executes multiple sub-steps of velocity-verlet and fluid-rigid impulse in batched command buffers.
     * @param registry ECS registry
     * @param fluidEntities Fluid entity list
     * @param rigidEntities Rigid body entity list
     * @param realCount Actual fluid particle count
     * @param paddedCount Padded fluid count
     * @param rigidCount Number of rigid bodies
     */
    void multiStepVelocityVerlet(
        entt::registry& registry,
        const std::vector<entt::entity>& fluidEntities,
        const std::vector<entt::entity>& rigidEntities,
        int realCount,
        int paddedCount,
        int rigidCount);

private:
    MTL::Device* device_ = nullptr;
    MTL::CommandQueue* commandQueue_ = nullptr;
    MTL::Library* metalLibrary_ = nullptr; ///< Keep the library alive while pipeline states are in use

    // Pipeline states
    MTL::ComputePipelineState* clearGridPSO_          = nullptr;
    MTL::ComputePipelineState* assignCellsPSO_        = nullptr;
    MTL::ComputePipelineState* computeDensityPSO_     = nullptr;
    MTL::ComputePipelineState* computeForcesPSO_      = nullptr;
    MTL::ComputePipelineState* verletHalfPSO_         = nullptr;
    MTL::ComputePipelineState* verletFinishPSO_       = nullptr;
    MTL::ComputePipelineState* computeBoundingBoxPSO_ = nullptr;
    MTL::ComputePipelineState* rigidFluidPositionPSO_ = nullptr;
    MTL::ComputePipelineState* rigidFluidImpulsePSO_  = nullptr;

    // GPU buffers that are reused/expanded as needed
    MTL::Buffer* particleBuf_           = nullptr;
    MTL::Buffer* rigidBuf_              = nullptr;
    MTL::Buffer* paramsBuf_             = nullptr;
    MTL::Buffer* boundingBoxParamsBuf_  = nullptr;
    MTL::Buffer* boundingBoxBuf_        = nullptr;
    MTL::Buffer* gridBuf_               = nullptr;

    // Cached maximum sizes for reallocation checks
    int maxFluidParticles_ = 0;
    int maxRigidBodies_    = 0;
    int maxGridSize_       = 0;
    int maxThreadgroups_   = 0;
};

} // namespace Systems