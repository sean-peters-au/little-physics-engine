/**
 * @fileoverview fluid.cpp
 * @brief A GPU-based 2D SPH fluid system using Metal, refactored for readability and maintainability
 *
 * This version mirrors the CPU system's multi-sub-step Velocity Verlet integration and grid-based
 * neighbor search, but leverages GPU compute via Metal. To closely match the CPU code:
 *  - We compute integer cell indices and bounds, as in buildUniformGridContiguousDomainPartition.
 *  - We use poly6 and spiky kernels in 2D, clamping negative pressures to zero.
 *  - Boundary conditions and gravity are handled elsewhere.
 */

#include <cmath>
#include <vector>
#include <iostream>
#include <limits>
#include <climits>

#include <Metal/Metal.hpp>
#include <Foundation/Foundation.hpp>
#include <entt/entt.hpp>

#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sph.hpp"
#include "nbody/core/profile.hpp"

namespace Systems {

/**
 * @class GPUFluidParticle
 * @brief Mirrors the struct declared in fluid_kernels.metal
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
    float h;
    float c;
    float density;
    float pressure;
};

/**
 * @class GPUGridCell
 * @brief Mirrors the struct declared in fluid_kernels.metal
 */
static constexpr int GPU_MAX_PER_CELL = 64;
struct GPUGridCell {
    int count;
    int indices[GPU_MAX_PER_CELL];
};

/**
 * @class GPUFluidParams
 * @brief Matches fluid_kernels.metal for grid indexing and SPH constants
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
    uint  particleCount;
};

/**
 * @class FluidSystem
 * @brief Runs multi-sub-step SPH on the GPU using Metal. Refactored to improve readability.
 */
class FluidSystem {
public:
    FluidSystem();
    ~FluidSystem();

    /**
     * @brief Main entry point: updates the fluid system by running multiple Velocity Verlet sub-steps.
     */
    void update(entt::registry& registry);

private:
    /**
     * @brief Creates a Metal compute pipeline from a function name in the loaded library.
     */
    MTL::ComputePipelineState* createComputePipeline(const char* name, MTL::Library* lib);

    /**
     * @brief Dispatches a short Metal compute pass, performing all typical steps (command buffer,
     *        encoder, setting pipeline state, dispatching threads).
     *
     * @param pipelineState The pipeline state to use
     * @param encodeFunc    A callback to set any required buffers, bytes, etc.
     * @param threadsCount  Number of threads to dispatch
     */
    void dispatchComputePass(
        MTL::ComputePipelineState* pipelineState,
        const std::function<void(MTL::ComputeCommandEncoder*)>& encodeFunc,
        size_t threadsCount) const;

    /**
     * @brief Gathers liquid-phase particles from ECS into a CPU-side array, returning a matching
     *        list of entities for write-back after the simulation.
     */
    std::vector<GPUFluidParticle> gatherFluidParticles(
        entt::registry& registry,
        std::vector<entt::entity>& entityList) const;

    /**
     * @brief Performs the velocity-verlet half-step on the GPU,
     *        then reads back positions so we can compute cell indices.
     */
    void dispatchVelocityVerletHalf(
        const std::vector<GPUFluidParticle>& cpuParticles,
        MTL::Buffer* particleBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    /**
     * @brief Determines integer-based global min/max cell indices among all particles.
     *
     * @param cpuParticles    The particle array
     * @param[out] cellSize   Computed as 2*h0 from the first particle
     * @param[out] globalMinGx The smallest cell index in x
     * @param[out] globalMaxGx The largest cell index in x
     * @param[out] globalMinGy The smallest cell index in y
     * @param[out] globalMaxGy The largest cell index in y
     */
    void computeGlobalCellBounds(
        const std::vector<GPUFluidParticle>& cpuParticles,
        float& cellSize,
        int& globalMinGx,
        int& globalMaxGx,
        int& globalMinGy,
        int& globalMaxGy) const;

    /**
     * @brief Clears the GPU grid using the clearGrid kernel.
     */
    void dispatchClearGrid(MTL::Buffer* gridBuf, int gridSize) const;

    /**
     * @brief Assigns each particle to a grid cell on the GPU.
     */
    void dispatchAssignCells(
        const std::vector<GPUFluidParticle>& cpuParticles,
        MTL::Buffer* particleBuf,
        MTL::Buffer* gridBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    /**
     * @brief Runs the density compute kernel on the GPU.
     */
    void dispatchComputeDensity(
        MTL::Buffer* particleBuf,
        MTL::Buffer* gridBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    /**
     * @brief Runs the pressure + viscosity forces kernel on the GPU.
     */
    void dispatchComputeForces(
        MTL::Buffer* particleBuf,
        MTL::Buffer* gridBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    /**
     * @brief Finishes the velocity-verlet integration (final velocity update) on the GPU.
     */
    void dispatchVelocityVerletFinish(
        MTL::Buffer* particleBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    /**
     * @brief Writes final positions and velocities back to ECS components.
     */
    void writeBackToECS(
        entt::registry& registry,
        const std::vector<entt::entity>& entityList,
        const std::vector<GPUFluidParticle>& cpuParticles) const;

    /**
     * @brief Performs the multi-sub-step Velocity Verlet integration on the GPU.
     */
    void multiStepVelocityVerlet(
        entt::registry& registry,
        std::vector<GPUFluidParticle>& cpuParticles,
        const std::vector<entt::entity>& entityList,
        MTL::Buffer* particleBuf,
        MTL::Buffer* paramsBuf,
        int realCount,
        int paddedCount);

private:
    MTL::Device* device_ = nullptr;
    MTL::CommandQueue* commandQueue_ = nullptr;
    MTL::Library* metalLibrary_ = nullptr;

    MTL::ComputePipelineState* clearGridPSO_      = nullptr;
    MTL::ComputePipelineState* assignCellsPSO_    = nullptr;
    MTL::ComputePipelineState* computeDensityPSO_ = nullptr;
    MTL::ComputePipelineState* computeForcesPSO_  = nullptr;
    MTL::ComputePipelineState* verletHalfPSO_     = nullptr;
    MTL::ComputePipelineState* verletFinishPSO_   = nullptr;

    static constexpr int N_SUB_STEPS = 10;
};

FluidSystem::FluidSystem() {
    device_ = MTL::CreateSystemDefaultDevice();
    if (!device_) {
        std::cerr << "No Metal device found." << std::endl;
        return;
    }
    commandQueue_ = device_->newCommandQueue();

    NS::Error* error = nullptr;
    NS::String* libPath = NS::String::string("build/fluid_kernels.metallib", NS::UTF8StringEncoding);
    metalLibrary_ = device_->newLibrary(libPath, &error);
    if (!metalLibrary_) {
        std::cerr << "Failed to load metal library: "
                  << (error ? error->localizedDescription()->utf8String() : "Unknown") << std::endl;
        return;
    }

    clearGridPSO_      = createComputePipeline("clearGrid",           metalLibrary_);
    assignCellsPSO_    = createComputePipeline("assignCells",         metalLibrary_);
    computeDensityPSO_ = createComputePipeline("computeDensity",      metalLibrary_);
    computeForcesPSO_  = createComputePipeline("computeForces",       metalLibrary_);
    verletHalfPSO_     = createComputePipeline("velocityVerletHalf",  metalLibrary_);
    verletFinishPSO_   = createComputePipeline("velocityVerletFinish",metalLibrary_);
}

FluidSystem::~FluidSystem() {
    if (clearGridPSO_)      { clearGridPSO_->release(); }
    if (assignCellsPSO_)    { assignCellsPSO_->release(); }
    if (computeDensityPSO_) { computeDensityPSO_->release(); }
    if (computeForcesPSO_)  { computeForcesPSO_->release(); }
    if (verletHalfPSO_)     { verletHalfPSO_->release(); }
    if (verletFinishPSO_)   { verletFinishPSO_->release(); }
    if (commandQueue_)      { commandQueue_->release(); }
}

MTL::ComputePipelineState* FluidSystem::createComputePipeline(const char* name, MTL::Library* lib) {
    NS::String* fnName = NS::String::string(name, NS::UTF8StringEncoding);
    MTL::Function* fn = lib->newFunction(fnName);
    if (!fn) {
        std::cerr << "Missing function " << name << " in metal library." << std::endl;
        return nullptr;
    }
    NS::Error* error = nullptr;
    MTL::ComputePipelineState* pso = device_->newComputePipelineState(fn, &error);
    fn->release();
    if (!pso) {
        std::cerr << "Failed to create pipeline: " << name << " => "
                  << (error ? error->localizedDescription()->utf8String() : "?") << std::endl;
    }
    return pso;
}

void FluidSystem::dispatchComputePass(
    MTL::ComputePipelineState* pipelineState,
    const std::function<void(MTL::ComputeCommandEncoder*)>& encodeFunc,
    size_t threadsCount) const
{
    auto cmdBuf = commandQueue_->commandBuffer();
    if (!cmdBuf) {
        return;
    }
    auto enc = cmdBuf->computeCommandEncoder();
    if (!enc) {
        cmdBuf->release();
        return;
    }

    enc->setComputePipelineState(pipelineState);
    encodeFunc(enc);

    MTL::Size threads(threadsCount, 1, 1);
    MTL::Size tgroup(64, 1, 1);
    enc->dispatchThreadgroups(threads, tgroup);

    enc->endEncoding();
    cmdBuf->commit();
    cmdBuf->waitUntilCompleted();
    cmdBuf->release();
}

std::vector<GPUFluidParticle> FluidSystem::gatherFluidParticles(
    entt::registry& registry,
    std::vector<entt::entity>& entityList) const
{
    PROFILE_SCOPE("FluidSystem::gatherFluidParticles");
    std::vector<GPUFluidParticle> cpuParticles;
    cpuParticles.reserve(registry.storage<Components::Position>().size());

    auto view = registry.view<Components::Position,
                              Components::Velocity,
                              Components::Mass,
                              Components::ParticlePhase,
                              Components::SmoothingLength,
                              Components::SpeedOfSound,
                              Components::SPHTemp>();

    for (auto e : view) {
        const auto& phase = view.get<Components::ParticlePhase>(e);
        if (phase.phase != Components::Phase::Liquid) {
            continue;
        }

        const auto& pos  = view.get<Components::Position>(e);
        const auto& vel  = view.get<Components::Velocity>(e);
        const auto& mass = view.get<Components::Mass>(e);
        const auto& sl   = view.get<Components::SmoothingLength>(e);
        const auto& snd  = view.get<Components::SpeedOfSound>(e);

        GPUFluidParticle p{};
        p.x = pos.x;
        p.y = pos.y;
        p.vx = vel.x;
        p.vy = vel.y;
        p.vxHalf = p.vx;
        p.vyHalf = p.vy;
        p.ax = 0.f;
        p.ay = 0.f;
        p.mass = mass.value;
        p.h    = (sl.value <= 0.f) ? 0.05f : sl.value;
        p.c    = snd.value;
        p.density = 0.f;
        p.pressure= 0.f;

        cpuParticles.push_back(p);
        entityList.push_back(e);
    }

    return cpuParticles;
}

void FluidSystem::dispatchVelocityVerletHalf(
    const std::vector<GPUFluidParticle>& cpuParticles,
    MTL::Buffer* particleBuf,
    MTL::Buffer* paramsBuf,
    int realCount) const
{
    // Copy updated particle data into GPU buffer
    memcpy(particleBuf->contents(), cpuParticles.data(), sizeof(GPUFluidParticle) * realCount);

    // Launch the half-step kernel
    dispatchComputePass(
        verletHalfPSO_,
        [&](MTL::ComputeCommandEncoder* enc) {
            enc->setBuffer(particleBuf, 0, 0);
            enc->setBuffer(paramsBuf, 0, 1);
        },
        realCount
    );
}

void FluidSystem::computeGlobalCellBounds(
    const std::vector<GPUFluidParticle>& cpuParticles,
    float& cellSize,
    int& globalMinGx,
    int& globalMaxGx,
    int& globalMinGy,
    int& globalMaxGy) const
{
    globalMinGx = INT_MAX;
    globalMaxGx = INT_MIN;
    globalMinGy = INT_MAX;
    globalMaxGy = INT_MIN;

    if (cpuParticles.empty()) {
        cellSize = 0.1f;
        return;
    }

    // Like CPU code: cellSize = 2*h0 from the first particle
    float h0 = cpuParticles[0].h;
    if (h0 <= 0.f) {
        h0 = 0.05f;
    }
    cellSize = 2.f * h0;

    for (const auto& p : cpuParticles) {
        float px = p.x + 1e-6f;
        float py = p.y + 1e-6f;
        int gx = static_cast<int>(std::floor(px / cellSize));
        int gy = static_cast<int>(std::floor(py / cellSize));

        if (gx < globalMinGx) { globalMinGx = gx; }
        if (gx > globalMaxGx) { globalMaxGx = gx; }
        if (gy < globalMinGy) { globalMinGy = gy; }
        if (gy > globalMaxGy) { globalMaxGy = gy; }
    }
}

void FluidSystem::dispatchClearGrid(MTL::Buffer* gridBuf, int gridSize) const {
    dispatchComputePass(
        clearGridPSO_,
        [&](MTL::ComputeCommandEncoder* enc) {
            enc->setBuffer(gridBuf, 0, 0);
            enc->setBytes(&gridSize, sizeof(int), 1);
        },
        static_cast<size_t>(gridSize)
    );
}

void FluidSystem::dispatchAssignCells(
    const std::vector<GPUFluidParticle>& cpuParticles,
    MTL::Buffer* particleBuf,
    MTL::Buffer* gridBuf,
    MTL::Buffer* paramsBuf,
    int realCount) const
{
    // Copy updated CPU data into GPU buffer
    memcpy(particleBuf->contents(), cpuParticles.data(), sizeof(GPUFluidParticle) * realCount);

    dispatchComputePass(
        assignCellsPSO_,
        [&](MTL::ComputeCommandEncoder* enc) {
            enc->setBuffer(particleBuf, 0, 0);
            enc->setBytes(&realCount, sizeof(int), 1);
            enc->setBuffer(gridBuf, 0, 2);
            enc->setBuffer(paramsBuf, 0, 3);
        },
        realCount
    );
}

void FluidSystem::dispatchComputeDensity(
    MTL::Buffer* particleBuf,
    MTL::Buffer* gridBuf,
    MTL::Buffer* paramsBuf,
    int realCount) const
{
    dispatchComputePass(
        computeDensityPSO_,
        [&](MTL::ComputeCommandEncoder* enc) {
            enc->setBuffer(particleBuf, 0, 0);
            enc->setBytes(&realCount, sizeof(int), 1);
            enc->setBuffer(gridBuf, 0, 2);
            enc->setBuffer(paramsBuf, 0, 3);
        },
        realCount
    );
}

void FluidSystem::dispatchComputeForces(
    MTL::Buffer* particleBuf,
    MTL::Buffer* gridBuf,
    MTL::Buffer* paramsBuf,
    int realCount) const
{
    dispatchComputePass(
        computeForcesPSO_,
        [&](MTL::ComputeCommandEncoder* enc) {
            enc->setBuffer(particleBuf, 0, 0);
            enc->setBytes(&realCount, sizeof(int), 1);
            enc->setBuffer(gridBuf, 0, 2);
            enc->setBuffer(paramsBuf, 0, 3);
        },
        realCount
    );
}

void FluidSystem::dispatchVelocityVerletFinish(
    MTL::Buffer* particleBuf,
    MTL::Buffer* paramsBuf,
    int realCount) const
{
    dispatchComputePass(
        verletFinishPSO_,
        [&](MTL::ComputeCommandEncoder* enc) {
            enc->setBuffer(particleBuf, 0, 0);
            enc->setBuffer(paramsBuf, 0, 1);
        },
        realCount
    );
}

void FluidSystem::writeBackToECS(
    entt::registry& registry,
    const std::vector<entt::entity>& entityList,
    const std::vector<GPUFluidParticle>& cpuParticles) const
{
    PROFILE_SCOPE("FluidSystem::writeBackToECS");
    for (size_t i = 0; i < entityList.size(); i++) {
        entt::entity e = entityList[i];
        auto& pos  = registry.get<Components::Position>(e);
        auto& vel  = registry.get<Components::Velocity>(e);
        auto& spht = registry.get<Components::SPHTemp>(e);

        const GPUFluidParticle& p = cpuParticles[i];
        pos.x = p.x;
        pos.y = p.y;
        vel.x = p.vx;
        vel.y = p.vy;
        spht.density  = p.density;
        spht.pressure = p.pressure;
    }
}

void FluidSystem::multiStepVelocityVerlet(
    entt::registry& registry,
    std::vector<GPUFluidParticle>& cpuParticles,
    const std::vector<entt::entity>& entityList,
    MTL::Buffer* particleBuf,
    MTL::Buffer* paramsBuf,
    int realCount,
    int paddedCount)
{
    PROFILE_SCOPE("FluidSystem::multiStepVelocityVerlet");
    float dt = static_cast<float>(SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration);
    float subDt = dt / static_cast<float>(N_SUB_STEPS);

    for (int step = 0; step < N_SUB_STEPS; step++) {
        // (A) Velocity Verlet half-step
        {
            GPUFluidParams params{};
            params.restDensity = static_cast<float>(SimulatorConstants::ParticleDensity);
            params.stiffness   = 500.f;
            params.viscosity   = 0.1f;
            params.dt          = subDt;
            params.halfDt      = 0.5f * subDt;
            params.particleCount = static_cast<uint>(paddedCount);

            // Copy the initial params
            memcpy(paramsBuf->contents(), &params, sizeof(GPUFluidParams));

            dispatchVelocityVerletHalf(cpuParticles, particleBuf, paramsBuf, realCount);

            // Read back new positions for cell indexing
            memcpy(cpuParticles.data(), particleBuf->contents(), sizeof(GPUFluidParticle) * realCount);
        }

        // (B) Compute global cell bounds (integer-based indexing)
        float cellSize;
        int globalMinGx, globalMaxGx, globalMinGy, globalMaxGy;
        computeGlobalCellBounds(
            cpuParticles,
            cellSize,
            globalMinGx,
            globalMaxGx,
            globalMinGy,
            globalMaxGy
        );

        int gridDimX = globalMaxGx - globalMinGx + 1;
        int gridDimY = globalMaxGy - globalMinGy + 1;
        if (gridDimX < 1) { gridDimX = 1; }
        if (gridDimY < 1) { gridDimY = 1; }
        int gridSize = gridDimX * gridDimY;

        // (C) Prepare updated simulation parameters for the next kernels
        {
            GPUFluidParams params{};
            params.cellSize    = cellSize;
            params.gridMinX    = globalMinGx;
            params.gridMinY    = globalMinGy;
            params.gridDimX    = gridDimX;
            params.gridDimY    = gridDimY;
            params.restDensity = static_cast<float>(SimulatorConstants::ParticleDensity);
            params.stiffness   = 500.f;
            params.viscosity   = 0.1f;
            params.dt          = subDt;
            params.halfDt      = 0.5f * subDt;
            params.particleCount = static_cast<uint>(paddedCount);

            memcpy(paramsBuf->contents(), &params, sizeof(GPUFluidParams));
        }

        // (D) Allocate and clear a GPU grid
        auto gridBuf = device_->newBuffer(
            sizeof(GPUGridCell) * gridSize,
            MTL::ResourceStorageModeShared
        );
        memset(gridBuf->contents(), 0, sizeof(GPUGridCell) * gridSize);
        dispatchClearGrid(gridBuf, gridSize);

        // (E) Assign cells, compute densities, compute forces
        dispatchAssignCells(cpuParticles, particleBuf, gridBuf, paramsBuf, realCount);
        dispatchComputeDensity(particleBuf, gridBuf, paramsBuf, realCount);
        dispatchComputeForces(particleBuf, gridBuf, paramsBuf, realCount);

        // Release the grid before finishing
        gridBuf->release();

        // (F) Velocity Verlet finish
        dispatchVelocityVerletFinish(particleBuf, paramsBuf, realCount);

        // Read back the final velocity for the next iteration
        memcpy(cpuParticles.data(), particleBuf->contents(), sizeof(GPUFluidParticle) * realCount);
    }
}

void FluidSystem::update(entt::registry& registry) {
    PROFILE_SCOPE("FluidSystem::update (GPU-based SPH)");

    if (!device_ || !commandQueue_) {
        return;
    }

    // 1) Gather fluid particles (CPU) and their ECS entities
    std::vector<entt::entity> entityList;
    std::vector<GPUFluidParticle> cpuParticles = gatherFluidParticles(registry, entityList);
    if (cpuParticles.empty()) {
        return;
    }

    int realCount = static_cast<int>(cpuParticles.size());

    // 2) Pad to next power-of-two (common for GPU optimizations)
    int paddedCount = 1;
    while (paddedCount < realCount) {
        paddedCount <<= 1;
    }

    // 3) Create GPU buffers for particles and simulation parameters
    auto particleBuf = device_->newBuffer(
        sizeof(GPUFluidParticle) * paddedCount,
        MTL::ResourceStorageModeShared
    );
    memset(particleBuf->contents(), 0, sizeof(GPUFluidParticle) * paddedCount);

    auto paramsBuf = device_->newBuffer(
        sizeof(GPUFluidParams),
        MTL::ResourceStorageModeShared
    );

    // 4) Perform the multi-sub-step Velocity Verlet
    multiStepVelocityVerlet(registry, cpuParticles, entityList, particleBuf, paramsBuf, realCount, paddedCount);

    // 5) After all sub-steps, write final results back to ECS
    writeBackToECS(registry, entityList, cpuParticles);

    // Release resources
    particleBuf->release();
    paramsBuf->release();
}

}  // namespace Systems
