/**
 * @fileoverview fluid.cpp
 * @brief A GPU-based 2D SPH fluid system using Metal, with GPU-based bounding box (1D dispatch),
 *        correct threadgroup memory allocation, and debug prints.
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
 * Mirrors fluid_kernels.metal
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

static constexpr int GPU_MAX_PER_CELL = 64;
struct GPUGridCell {
    int count;
    int indices[GPU_MAX_PER_CELL];
};

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
 * For bounding box
 */
struct BBoxParams {
    int particleCount;
    int numThreadgroups;
};

/**
 * @class FluidSystem
 */
class FluidSystem {
public:
    FluidSystem();
    ~FluidSystem();

    void update(entt::registry& registry);

private:
    MTL::ComputePipelineState* createComputePipeline(const char* name, MTL::Library* lib);

    /**
     * @brief Dispatches a short Metal compute pass, performing typical setup.
     */
    void dispatchComputePass(
        MTL::ComputePipelineState* pipelineState,
        const std::function<void(MTL::ComputeCommandEncoder*)>& encodeFunc,
        size_t threadsCount,
        size_t threadsPerGroup = 256) const;

    std::vector<GPUFluidParticle> gatherFluidParticles(
        entt::registry& registry,
        std::vector<entt::entity>& entityList) const;

    void dispatchVelocityVerletHalf(
        MTL::Buffer* particleBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    /**
     * @brief Bounding box kernel dispatch, remembering to set threadgroupMemoryLength
     *        for localShared[] and localValidCount[].
     */
    void dispatchComputeBoundingBox(
        MTL::Buffer* particleBuf,
        MTL::Buffer* boundingBoxParamsBuf,
        MTL::Buffer* boundingBoxBuf,
        int realCount,
        int numThreadgroups) const;

    void reduceBoundingBoxOnCPU(
        MTL::Buffer* boundingBoxBuf,
        int numThreadgroups,
        float& minX,
        float& maxX,
        float& minY,
        float& maxY) const;

    void dispatchClearGrid(MTL::Buffer* gridBuf, int gridSize) const;
    void dispatchAssignCells(
        MTL::Buffer* particleBuf,
        MTL::Buffer* gridBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    void dispatchComputeDensity(
        MTL::Buffer* particleBuf,
        MTL::Buffer* gridBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    void dispatchComputeForces(
        MTL::Buffer* particleBuf,
        MTL::Buffer* gridBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    void dispatchVelocityVerletFinish(
        MTL::Buffer* particleBuf,
        MTL::Buffer* paramsBuf,
        int realCount) const;

    void writeBackToECS(
        entt::registry& registry,
        const std::vector<entt::entity>& entityList,
        MTL::Buffer* particleBuf,
        int realCount) const;

    void multiStepVelocityVerlet(
        entt::registry& registry,
        std::vector<entt::entity>& entityList,
        MTL::Buffer* particleBuf,
        MTL::Buffer* paramsBuf,
        MTL::Buffer* boundingBoxParamsBuf,
        MTL::Buffer* boundingBoxBuf,
        int realCount,
        int paddedCount);

private:
    MTL::Device* device = nullptr;
    MTL::CommandQueue* commandQueue = nullptr;
    MTL::Library* metalLibrary = nullptr;

    MTL::ComputePipelineState* clearGridPSO          = nullptr;
    MTL::ComputePipelineState* assignCellsPSO        = nullptr;
    MTL::ComputePipelineState* computeDensityPSO     = nullptr;
    MTL::ComputePipelineState* computeForcesPSO      = nullptr;
    MTL::ComputePipelineState* verletHalfPSO         = nullptr;
    MTL::ComputePipelineState* verletFinishPSO       = nullptr;
    MTL::ComputePipelineState* computeBoundingBoxPSO = nullptr;

    static constexpr int N_SUB_STEPS = 10;
};

FluidSystem::FluidSystem() {
    device = MTL::CreateSystemDefaultDevice();
    if (!device) {
        std::cerr << "No Metal device found." << std::endl;
        return;
    }
    commandQueue = device->newCommandQueue();

    NS::Error* error = nullptr;
    NS::String* libPath = NS::String::string("build/fluid_kernels.metallib", NS::UTF8StringEncoding);
    metalLibrary = device->newLibrary(libPath, &error);
    if (!metalLibrary) {
        std::cerr << "Failed to load metal library: "
                  << (error ? error->localizedDescription()->utf8String() : "Unknown") << std::endl;
        return;
    }

    clearGridPSO          = createComputePipeline("clearGrid",           metalLibrary);
    assignCellsPSO        = createComputePipeline("assignCells",         metalLibrary);
    computeDensityPSO     = createComputePipeline("computeDensity",      metalLibrary);
    computeForcesPSO      = createComputePipeline("computeForces",       metalLibrary);
    verletHalfPSO         = createComputePipeline("velocityVerletHalf",  metalLibrary);
    verletFinishPSO       = createComputePipeline("velocityVerletFinish",metalLibrary);
    computeBoundingBoxPSO = createComputePipeline("computeBoundingBox",  metalLibrary);
}

FluidSystem::~FluidSystem() {
    if (clearGridPSO)          { clearGridPSO->release(); }
    if (assignCellsPSO)        { assignCellsPSO->release(); }
    if (computeDensityPSO)     { computeDensityPSO->release(); }
    if (computeForcesPSO)      { computeForcesPSO->release(); }
    if (verletHalfPSO)         { verletHalfPSO->release(); }
    if (verletFinishPSO)       { verletFinishPSO->release(); }
    if (computeBoundingBoxPSO) { computeBoundingBoxPSO->release(); }
    if (commandQueue)          { commandQueue->release(); }
}

MTL::ComputePipelineState* FluidSystem::createComputePipeline(const char* name, MTL::Library* lib) {
    NS::String* fnName = NS::String::string(name, NS::UTF8StringEncoding);
    MTL::Function* fn = lib->newFunction(fnName);
    if (!fn) {
        std::cerr << "Missing function " << name << " in metal library." << std::endl;
        return nullptr;
    }
    NS::Error* error = nullptr;
    MTL::ComputePipelineState* pso = device->newComputePipelineState(fn, &error);
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
    size_t threadsCount,
    size_t threadsPerGroup) const
{
    auto cmdBuf = commandQueue->commandBuffer();
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

    // For bounding box kernel or others that use threadgroup arrays, we typically
    // do something like setThreadgroupMemoryLength(...) here if needed.
    // But we allow the lambda to do it (so boundingBox can do it specifically).

    size_t groups = (threadsCount + threadsPerGroup - 1) / threadsPerGroup;
    MTL::Size tgSize(threadsPerGroup, 1, 1);
    MTL::Size numGroups(groups, 1, 1);
    enc->dispatchThreadgroups(numGroups, tgSize);

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
    MTL::Buffer* particleBuf,
    MTL::Buffer* paramsBuf,
    int realCount) const
{
    dispatchComputePass(
        verletHalfPSO,
        [&](MTL::ComputeCommandEncoder* enc) {
            enc->setBuffer(particleBuf, 0, 0);
            enc->setBuffer(paramsBuf,   0, 1);
        },
        realCount,
        256
    );
}

void FluidSystem::dispatchComputeBoundingBox(
    MTL::Buffer* particleBuf,
    MTL::Buffer* boundingBoxParamsBuf,
    MTL::Buffer* boundingBoxBuf,
    int realCount,
    int numThreadgroups /* unused */) const
{
    // We'll produce one partial bounding box per threadgroup in boundingBoxBuf.
    // The kernel defines:
    //   threadgroup BBox localShared[256];
    //   threadgroup atomic_int localValidCount[1];
    // So we must allocate enough threadgroup memory for them.
    // localShared = 256 * sizeof(BBox)
    // localValidCount = 1 * sizeof(atomic_int)
    // We'll set them as argument indexes 0..N after the buffers are set.

    size_t blockSize = 256;
    size_t localSharedBBox = sizeof(float)*4 * blockSize; // or sizeof(BBox)*256
    size_t localValidCount = sizeof(int);                 // for atomic_int

    dispatchComputePass(
        computeBoundingBoxPSO,
        [&](MTL::ComputeCommandEncoder* enc) {
            enc->setBuffer(particleBuf,           0, 0);
            enc->setBuffer(boundingBoxParamsBuf,  0, 1);
            enc->setBuffer(boundingBoxBuf,        0, 2);

            // Now allocate threadgroup memory: argument index 3 => localShared
            // argument index 4 => localValidCount (the kernel signature indexes)
            enc->setThreadgroupMemoryLength(localSharedBBox, 3);
            enc->setThreadgroupMemoryLength(localValidCount, 4);
        },
        realCount,
        blockSize
    );
}

void FluidSystem::reduceBoundingBoxOnCPU(
    MTL::Buffer* boundingBoxBuf,
    int numThreadgroups,
    float& minX,
    float& maxX,
    float& minY,
    float& maxY) const
{
    struct BBox {
        float minX;
        float maxX;
        float minY;
        float maxY;
    };

    const BBox* partials = static_cast<const BBox*>(boundingBoxBuf->contents());

    minX =  std::numeric_limits<float>::max();
    maxX = -std::numeric_limits<float>::max();
    minY =  std::numeric_limits<float>::max();
    maxY = -std::numeric_limits<float>::max();

    for (int i = 0; i < numThreadgroups; i++) {
        const BBox& box = partials[i];
        if (box.minX < minX) { minX = box.minX; }
        if (box.maxX > maxX) { maxX = box.maxX; }
        if (box.minY < minY) { minY = box.minY; }
        if (box.maxY > maxY) { maxY = box.maxY; }
    }
}

void FluidSystem::dispatchClearGrid(MTL::Buffer* gridBuf, int gridSize) const {
    dispatchComputePass(
        clearGridPSO,
        [&](MTL::ComputeCommandEncoder* enc) {
            enc->setBuffer(gridBuf, 0, 0);
            enc->setBytes(&gridSize, sizeof(int), 1);
        },
        static_cast<size_t>(gridSize),
        256
    );
}

void FluidSystem::dispatchAssignCells(
    MTL::Buffer* particleBuf,
    MTL::Buffer* gridBuf,
    MTL::Buffer* paramsBuf,
    int realCount) const
{
    dispatchComputePass(
        assignCellsPSO,
        [&](MTL::ComputeCommandEncoder* enc) {
            enc->setBuffer(particleBuf, 0, 0);
            enc->setBytes(&realCount,  sizeof(int), 1);
            enc->setBuffer(gridBuf,    0, 2);
            enc->setBuffer(paramsBuf,  0, 3);
        },
        realCount,
        256
    );
}

void FluidSystem::dispatchComputeDensity(
    MTL::Buffer* particleBuf,
    MTL::Buffer* gridBuf,
    MTL::Buffer* paramsBuf,
    int realCount) const
{
    dispatchComputePass(
        computeDensityPSO,
        [&](MTL::ComputeCommandEncoder* enc) {
            enc->setBuffer(particleBuf, 0, 0);
            enc->setBytes(&realCount,   sizeof(int), 1);
            enc->setBuffer(gridBuf,     0, 2);
            enc->setBuffer(paramsBuf,   0, 3);
        },
        realCount,
        256
    );
}

void FluidSystem::dispatchComputeForces(
    MTL::Buffer* particleBuf,
    MTL::Buffer* gridBuf,
    MTL::Buffer* paramsBuf,
    int realCount) const
{
    dispatchComputePass(
        computeForcesPSO,
        [&](MTL::ComputeCommandEncoder* enc) {
            enc->setBuffer(particleBuf, 0, 0);
            enc->setBytes(&realCount,   sizeof(int), 1);
            enc->setBuffer(gridBuf,     0, 2);
            enc->setBuffer(paramsBuf,   0, 3);
        },
        realCount,
        256
    );
}

void FluidSystem::dispatchVelocityVerletFinish(
    MTL::Buffer* particleBuf,
    MTL::Buffer* paramsBuf,
    int realCount) const
{
    dispatchComputePass(
        verletFinishPSO,
        [&](MTL::ComputeCommandEncoder* enc) {
            enc->setBuffer(particleBuf, 0, 0);
            enc->setBuffer(paramsBuf,   0, 1);
        },
        realCount,
        256
    );
}

void FluidSystem::writeBackToECS(
    entt::registry& registry,
    const std::vector<entt::entity>& entityList,
    MTL::Buffer* particleBuf,
    int realCount) const
{
    PROFILE_SCOPE("FluidSystem::writeBackToECS");
    const auto* finalParticles = static_cast<const GPUFluidParticle*>(particleBuf->contents());
    for (int i = 0; i < realCount; i++) {
        entt::entity e = entityList[static_cast<size_t>(i)];
        auto& pos  = registry.get<Components::Position>(e);
        auto& vel  = registry.get<Components::Velocity>(e);
        auto& spht = registry.get<Components::SPHTemp>(e);

        const GPUFluidParticle& p = finalParticles[i];
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
    std::vector<entt::entity>& entityList,
    MTL::Buffer* particleBuf,
    MTL::Buffer* paramsBuf,
    MTL::Buffer* boundingBoxParamsBuf,
    MTL::Buffer* boundingBoxBuf,
    int realCount,
    int paddedCount)
{
    PROFILE_SCOPE("FluidSystem::multiStepVelocityVerlet");
    float dt = float(SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration);
    float subDt = dt / float(N_SUB_STEPS);

    // We'll fix the block size = 256 for bounding box kernel
    const int BLOCK_SIZE = 256;
    int numThreadgroups = (realCount + BLOCK_SIZE - 1) / BLOCK_SIZE;

    for (int step = 0; step < N_SUB_STEPS; step++) {
        // (A) velocityVerletHalf
        {
            GPUFluidParams params{};
            params.restDensity = float(SimulatorConstants::ParticleDensity);
            params.stiffness   = 50.f;
            params.viscosity   = 0.03f;
            params.dt          = subDt;
            params.halfDt      = 0.5f * subDt;
            params.particleCount = static_cast<uint>(paddedCount);

            memcpy(paramsBuf->contents(), &params, sizeof(GPUFluidParams));
            dispatchVelocityVerletHalf(particleBuf, paramsBuf, realCount);
        }

        // (B) bounding box
        {
            BBoxParams bboxp;
            bboxp.particleCount   = realCount;
            bboxp.numThreadgroups = numThreadgroups;
            memcpy(boundingBoxParamsBuf->contents(), &bboxp, sizeof(bboxp));
        }
        dispatchComputeBoundingBox(
            particleBuf,
            boundingBoxParamsBuf,
            boundingBoxBuf,
            realCount,
            numThreadgroups
        );

        float minX =  std::numeric_limits<float>::max();
        float maxX = -std::numeric_limits<float>::max();
        float minY =  std::numeric_limits<float>::max();
        float maxY = -std::numeric_limits<float>::max();
        reduceBoundingBoxOnCPU(boundingBoxBuf, numThreadgroups, minX, maxX, minY, maxY);

        // (C) find maximum h among all particles on CPU
        float maxH = 0.05f;
        {
            const auto* pData = static_cast<const GPUFluidParticle*>(particleBuf->contents());
            for (int i = 0; i < realCount; i++) {
                float hi = pData[i].h;
                if (hi > maxH) {
                    maxH = hi;
                }
            }
        }
        float cellSize = 2.f * maxH;

        // tiny offset
        minX -= 1e-6f;
        minY -= 1e-6f;

        int globalMinGx = int(std::floor(minX / cellSize));
        int globalMinGy = int(std::floor(minY / cellSize));
        int globalMaxGx = int(std::floor(maxX / cellSize));
        int globalMaxGy = int(std::floor(maxY / cellSize));

        int gridDimX = globalMaxGx - globalMinGx + 1;
        int gridDimY = globalMaxGy - globalMinGy + 1;
        if (gridDimX < 1) { gridDimX = 1; }
        if (gridDimY < 1) { gridDimY = 1; }
        int gridSize = gridDimX * gridDimY;

        // (D) fluid params
        {
            GPUFluidParams params{};
            params.cellSize    = cellSize;
            params.gridMinX    = globalMinGx;
            params.gridMinY    = globalMinGy;
            params.gridDimX    = gridDimX;
            params.gridDimY    = gridDimY;
            params.restDensity = float(SimulatorConstants::ParticleDensity);
            params.stiffness   = 50.f;
            params.viscosity   = 0.03f;
            params.dt          = subDt;
            params.halfDt      = 0.5f * subDt;
            params.particleCount = static_cast<uint>(paddedCount);

            memcpy(paramsBuf->contents(), &params, sizeof(GPUFluidParams));
        }

        auto gridBuf = device->newBuffer(
            sizeof(GPUGridCell) * gridSize,
            MTL::ResourceStorageModeShared
        );
        memset(gridBuf->contents(), 0, sizeof(GPUGridCell) * gridSize);
        dispatchClearGrid(gridBuf, gridSize);

        // (E) neighbor steps
        dispatchAssignCells(particleBuf, gridBuf, paramsBuf, realCount);
        dispatchComputeDensity(particleBuf, gridBuf, paramsBuf, realCount);

        dispatchComputeForces(particleBuf, gridBuf, paramsBuf, realCount);
        gridBuf->release();

        // (F) velocityVerletFinish
        dispatchVelocityVerletFinish(particleBuf, paramsBuf, realCount);
    }
}

void FluidSystem::update(entt::registry& registry) {
    PROFILE_SCOPE("FluidSystem::update (GPU-based SPH)");

    if (!device || !commandQueue) {
        return;
    }

    // 1) gather CPU fluid
    std::vector<entt::entity> entityList;
    auto cpuParticles = gatherFluidParticles(registry, entityList);
    if (cpuParticles.empty()) {
        return;
    }
    int realCount = int(cpuParticles.size());

    // 2) pad
    int paddedCount = 1;
    while (paddedCount < realCount) {
        paddedCount <<= 1;
    }

    // 3) create GPU buffers
    auto particleBuf = device->newBuffer(sizeof(GPUFluidParticle)*paddedCount,
                                          MTL::ResourceStorageModeShared);
    memset(particleBuf->contents(), 0, sizeof(GPUFluidParticle)*paddedCount);
    memcpy(particleBuf->contents(), cpuParticles.data(), sizeof(GPUFluidParticle)*realCount);

    auto paramsBuf = device->newBuffer(sizeof(GPUFluidParams),
                                        MTL::ResourceStorageModeShared);

    // boundingBoxParams: has BBoxParams { particleCount, numThreadgroups }
    auto boundingBoxParamsBuf = device->newBuffer(sizeof(BBoxParams),
                                                   MTL::ResourceStorageModeShared);
    memset(boundingBoxParamsBuf->contents(), 0, sizeof(BBoxParams));

    // boundingBoxBuf: array of BBox partial results, length = #threadgroups
    const int BLOCK_SIZE = 256;
    int numThreadgroups = (realCount + BLOCK_SIZE - 1) / BLOCK_SIZE;
    auto boundingBoxBuf = device->newBuffer(sizeof(float)*4*numThreadgroups,
                                             MTL::ResourceStorageModeShared);
    memset(boundingBoxBuf->contents(), 0, sizeof(float)*4*numThreadgroups);

    // 4) multi-step velocity-verlet
    multiStepVelocityVerlet(registry,
                            entityList,
                            particleBuf,
                            paramsBuf,
                            boundingBoxParamsBuf,
                            boundingBoxBuf,
                            realCount,
                            paddedCount);

    // 5) final writeback
    writeBackToECS(registry, entityList, particleBuf, realCount);

    boundingBoxBuf->release();
    boundingBoxParamsBuf->release();
    particleBuf->release();
    paramsBuf->release();
}

} // namespace Systems
