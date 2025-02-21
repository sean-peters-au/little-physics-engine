/**
 * @file fluid_system.cpp
 * @brief A complete SPH FluidSystem using Metal for neighbor search & force computation.
 * 
 * This file can stand alone if your build environment supports Metal C++.
 * For older macOS, you'd likely do Objective-C++ bridging.
 */

#include <Metal/Metal.hpp>
#include <Foundation/Foundation.hpp>
#include <cmath>
#include <vector>
#include <iostream>
#include <entt/entt.hpp>

// Include your ECS components
#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sph.hpp"
#include "nbody/core/profile.hpp"

namespace Systems {

/**
 * The GPU side "Particle" must match this layout exactly in the .metal source.
 */
struct GPUFluidParticle {
    float x, y;          // position
    float vx, vy;        // velocity
    float vxHalf, vyHalf;// velocity at half-step
    float ax, ay;        // acceleration
    float mass, h, c;    // mass, smoothing length, speed of sound
    float density, pressure;
};

/**
 * The GPU side "GridCell" must match this layout in the .metal code.
 * We do a naive approach with a fixed capacity per cell.
 */
static constexpr int GPU_MAX_PER_CELL = 64;
struct GPUGridCell {
    int count;  // atomic int on GPU
    int indices[GPU_MAX_PER_CELL];
};

/**
 * The GPU side "FluidParams" struct. Must match .metal definition
 */
struct GPUFluidParams {
    float cellSize;
    int   gridDimX;
    int   gridDimY;
    int   gridMinX;
    int   gridMinY;
    float restDensity;
    float stiffness;
    float viscosity;
    float dt;
    float halfDt;  // optional
    uint  particleCount;
};


/**
 * The FluidSystem class: uses the above .metal code for GPU-based SPH sub-steps.
 */
class FluidSystem {
public:
    FluidSystem();
    ~FluidSystem();

    void update(entt::registry &registry);

private:
    // metal objects
    MTL::Device*            device         = nullptr;
    MTL::CommandQueue*      commandQueue   = nullptr;
    MTL::Library*           metalLibrary   = nullptr;

    // pipeline states
    MTL::ComputePipelineState* clearGridPSO      = nullptr;
    MTL::ComputePipelineState* assignCellsPSO    = nullptr;
    MTL::ComputePipelineState* computeDensityPSO = nullptr;
    MTL::ComputePipelineState* computeForcesPSO  = nullptr;
    MTL::ComputePipelineState* verletHalfPSO     = nullptr;
    MTL::ComputePipelineState* verletFinishPSO   = nullptr;

    // Some config
    static constexpr int   N_SUB_STEPS = 4;
    static constexpr int   GRID_DIM_X  = 128;
    static constexpr int   GRID_DIM_Y  = 128;
    static constexpr int   GRID_SIZE   = GRID_DIM_X * GRID_DIM_Y;

    // Helper for pipeline creation
    MTL::ComputePipelineState* createPSO(const char* fnName, MTL::Library* lib);

    // Minimally naive approach: we create/destroy buffers each frame. A real system might re-use them.
};

FluidSystem::FluidSystem()
{
    // Create MTLDevice, command queue
    device = MTL::CreateSystemDefaultDevice();
    if (!device) {
        std::cerr << "No Metal device found. This requires macOS with Metal support." << std::endl;
        return;
    }

    commandQueue = device->newCommandQueue();

    // Load the precompiled library from file.
    NS::Error* error = nullptr;
    NS::String* libPath = NS::String::string("build/fluid_kernels.metallib", NS::UTF8StringEncoding);
    // Save the loaded library into our member variable.
    metalLibrary = device->newLibrary(libPath, &error);
    if (!metalLibrary) {
        std::cerr << "Failed to load metal library from file: " 
                  << (error ? error->localizedDescription()->utf8String() : "Unknown error") << std::endl;
        return;
    }

    // Create pipeline states using the external library.
    clearGridPSO      = createPSO("clearGrid",           metalLibrary);
    if (!clearGridPSO) {
        std::cerr << "Failed to create clearGrid pipeline state." << std::endl;
        return;
    }
    assignCellsPSO    = createPSO("assignCells",         metalLibrary);
    if (!assignCellsPSO) {
        std::cerr << "Failed to create assignCells pipeline state." << std::endl;
        return;
    }
    computeDensityPSO = createPSO("computeDensity",      metalLibrary);
    if (!computeDensityPSO) {
        std::cerr << "Failed to create computeDensity pipeline state." << std::endl;
        return;
    }
    computeForcesPSO  = createPSO("computeForces",       metalLibrary);
    if (!computeForcesPSO) {
        std::cerr << "Failed to create computeForces pipeline state." << std::endl;
        return;
    }
    verletHalfPSO     = createPSO("velocityVerletHalf",  metalLibrary);
    if (!verletHalfPSO) {
        std::cerr << "Failed to create velocityVerletHalf pipeline state." << std::endl;
        return;
    }
    verletFinishPSO   = createPSO("velocityVerletFinish",metalLibrary);
    if (!verletFinishPSO) {
        std::cerr << "Failed to create velocityVerletFinish pipeline state." << std::endl;
        return;
    }

    // Do not release metalLibrary here! It must live until the FluidSystem is done.
    // library->release();
}

FluidSystem::~FluidSystem()
{
    if(clearGridPSO) clearGridPSO->release();
    if(assignCellsPSO) assignCellsPSO->release();
    if(computeDensityPSO) computeDensityPSO->release();
    if(computeForcesPSO) computeForcesPSO->release();
    if(verletHalfPSO) verletHalfPSO->release();
    if(verletFinishPSO) verletFinishPSO->release();
    if(commandQueue) commandQueue->release();
    // device is autoreleased typically
}

MTL::ComputePipelineState* FluidSystem::createPSO(const char* fnName, MTL::Library* lib)
{
    NS::String* name = NS::String::string(fnName, NS::UTF8StringEncoding);
    MTL::Function* fn = lib->newFunction(name);
    if(!fn) {
        std::cerr << "Missing function " << fnName << " in metal library.\n";
        return nullptr;
    }
    NS::Error* error = nullptr;
    auto pso = device->newComputePipelineState(fn, &error);
    fn->release();
    if(!pso){
        std::cerr << "Failed to create pipeline for " << fnName << ": "
                  << (error ? error->localizedDescription()->utf8String() : "") << "\n";
    }
    return pso;
}

/**
 * Actually performs the fluid simulation each tick.
 */
void FluidSystem::update(entt::registry &registry)
{
    PROFILE_SCOPE("FluidSystem::update (Metal GPU)");

    if(!device || !commandQueue) {
        // fallback to CPU or do nothing
        std::cout << "No device or command queue found." << std::endl;
        return;
    }

    // 1) Gather fluid particles from ECS
    std::vector<GPUFluidParticle> cpuParticles;
    cpuParticles.reserve(registry.storage<Components::Position>().size()); // or registry.size_hint() if you like

    // We'll also store an array of entt::entity so we can map back
    std::vector<entt::entity> entityList;

    // Grabbing a typical dt:
    float dt = float(SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration);

    auto view = registry.view<Components::Position,
                              Components::Velocity,
                              Components::Mass,
                              Components::ParticlePhase,
                              Components::SmoothingLength,
                              Components::SpeedOfSound,
                              Components::SPHTemp>();

    for (auto e : view) {
        auto &ph = view.get<Components::ParticlePhase>(e);
        if(ph.phase != Components::Phase::Liquid) continue;

        auto &pos = view.get<Components::Position>(e);
        auto &vel = view.get<Components::Velocity>(e);
        auto &mass= view.get<Components::Mass>(e);
        auto &sl  = view.get<Components::SmoothingLength>(e);
        auto &snd = view.get<Components::SpeedOfSound>(e);

        GPUFluidParticle p{};
        p.x= pos.x;
        p.y= pos.y;
        p.vx=vel.x;
        p.vy=vel.y;
        p.vxHalf= p.vx; 
        p.vyHalf= p.vy;
        p.ax=0; p.ay=0;
        p.mass= mass.value;
        p.h   = sl.value;
        p.c   = snd.value;
        p.density=0.f; p.pressure=0.f;

        cpuParticles.push_back(p);
        entityList.push_back(e);
    }

    if(cpuParticles.empty()) return;

    int particleCount = pow(2, ceil(log2((int)cpuParticles.size())));

    // 2) Create GPU buffers
    // We'll re-create them each frame for simplicity
    auto particleBuf = device->newBuffer(sizeof(GPUFluidParticle)*particleCount,
                                         MTL::ResourceStorageModeShared);
    memcpy(particleBuf->contents(), cpuParticles.data(), sizeof(GPUFluidParticle)*particleCount);

    // We store grid in a fixed size array:
    auto gridBuf = device->newBuffer(sizeof(GPUGridCell)*GRID_SIZE,
                                     MTL::ResourceStorageModeShared);

    // Fluid params
    GPUFluidParams params{};
    params.cellSize = 2.f * (particleCount > 0 ? cpuParticles[0].h : 0.05f);
    params.gridDimX = GRID_DIM_X;
    params.gridDimY = GRID_DIM_Y;
    params.gridMinX = 0;
    params.gridMinY = 0;
    params.restDensity = float(SimulatorConstants::ParticleDensity);
    params.stiffness   = 4000.f;
    params.viscosity   = 0.1f;
    // We'll do dt / N_SUB_STEPS per sub-step
    float subDt = dt / float(N_SUB_STEPS);
    params.dt = subDt;
    params.halfDt = 0.5f * subDt;
    params.particleCount = particleCount;

    auto paramsBuf = device->newBuffer(sizeof(GPUFluidParams), MTL::ResourceStorageModeShared);
    memcpy(paramsBuf->contents(), &params, sizeof(GPUFluidParams));

    // 3) We'll do N_SUB_STEPS velocity-verlet sub steps
    for(int step=0; step<N_SUB_STEPS; step++){
        // (a) velocityVerletHalf kernel
        {
            auto cmdBuf = commandQueue->commandBuffer();
            auto enc = cmdBuf->computeCommandEncoder();
            enc->setComputePipelineState(verletHalfPSO);
            enc->setBuffer(particleBuf, 0, 0);
            enc->setBuffer(paramsBuf, 0, 1);

            MTL::Size threadsPerGrid(particleCount,1,1);
            MTL::Size threadsPerGroup(64,1,1);
            enc->dispatchThreadgroups(threadsPerGrid, threadsPerGroup);
            enc->endEncoding();
            cmdBuf->commit();
            cmdBuf->waitUntilCompleted();
            if (cmdBuf->status() != MTL::CommandBufferStatusCompleted) {
                std::cout << "Command buffer status: " << cmdBuf->status() << std::endl;
            }
        }

        // (b) clearGrid
        {
            auto cmdBuf = commandQueue->commandBuffer();
            auto enc = cmdBuf->computeCommandEncoder();
            enc->setComputePipelineState(clearGridPSO);
            int cellCount = GRID_SIZE;
            enc->setBuffer(gridBuf, 0, 0);
            enc->setBytes(&cellCount, sizeof(int), 1);

            MTL::Size threads(GRID_SIZE,1,1);
            MTL::Size tgroup(64,1,1);
            enc->dispatchThreadgroups(threads, tgroup);
            enc->endEncoding();
            cmdBuf->commit();
            cmdBuf->waitUntilCompleted();
            if (cmdBuf->status() != MTL::CommandBufferStatusCompleted) {
                std::cout << "Command buffer status: " << cmdBuf->status() << std::endl;
            }
        }

        // (c) assignCells
        {
            auto cmdBuf = commandQueue->commandBuffer();
            auto enc = cmdBuf->computeCommandEncoder();
            enc->setComputePipelineState(assignCellsPSO);
            enc->setBuffer(particleBuf, 0, 0);
            enc->setBytes(&particleCount, sizeof(int), 1);
            enc->setBuffer(gridBuf, 0, 2);
            enc->setBuffer(paramsBuf, 0, 3);

            MTL::Size threads(particleCount,1,1);
            MTL::Size tgroup(64,1,1);
            enc->dispatchThreadgroups(threads, tgroup);
            enc->endEncoding();
            cmdBuf->commit();
            cmdBuf->waitUntilCompleted();
            if (cmdBuf->status() != MTL::CommandBufferStatusCompleted) {
                std::cout << "Command buffer status: " << cmdBuf->status() << std::endl;
            }
        }

        // (d) computeDensity
        {
            auto cmdBuf = commandQueue->commandBuffer();
            auto enc = cmdBuf->computeCommandEncoder();
            enc->setComputePipelineState(computeDensityPSO);
            enc->setBuffer(particleBuf, 0, 0);
            enc->setBytes(&particleCount, sizeof(int), 1);
            enc->setBuffer(gridBuf, 0, 2);
            enc->setBuffer(paramsBuf, 0, 3);

            MTL::Size threads(particleCount,1,1);
            MTL::Size tgroup(64,1,1);
            enc->dispatchThreadgroups(threads, tgroup);
            enc->endEncoding();
            cmdBuf->commit();
            cmdBuf->waitUntilCompleted();
            if (cmdBuf->status() != MTL::CommandBufferStatusCompleted) {
                std::cout << "Command buffer status: " << cmdBuf->status() << std::endl;
            }
        }

        // (e) computeForces
        {
            auto cmdBuf = commandQueue->commandBuffer();
            auto enc = cmdBuf->computeCommandEncoder();
            enc->setComputePipelineState(computeForcesPSO);
            enc->setBuffer(particleBuf, 0, 0);
            enc->setBytes(&particleCount, sizeof(int), 1);
            enc->setBuffer(gridBuf, 0, 2);
            enc->setBuffer(paramsBuf, 0, 3);

            MTL::Size threads(particleCount,1,1);
            MTL::Size tgroup(64,1,1);
            enc->dispatchThreadgroups(threads, tgroup);
            enc->endEncoding();
            cmdBuf->commit();
            cmdBuf->waitUntilCompleted();
            if (cmdBuf->status() != MTL::CommandBufferStatusCompleted) {
                std::cout << "Command buffer status: " << cmdBuf->status() << std::endl;
            }
        }

        // (f) velocityVerletFinish
        {
            auto cmdBuf = commandQueue->commandBuffer();
            auto enc = cmdBuf->computeCommandEncoder();
            enc->setComputePipelineState(verletFinishPSO);
            enc->setBuffer(particleBuf, 0, 0);
            enc->setBuffer(paramsBuf, 0, 1);

            MTL::Size threadsPerGrid(particleCount,1,1);
            MTL::Size threadsPerGroup(64,1,1);
            enc->dispatchThreadgroups(threadsPerGrid, threadsPerGroup);
            enc->endEncoding();
            cmdBuf->commit();
            cmdBuf->waitUntilCompleted();
            if (cmdBuf->status() != MTL::CommandBufferStatusCompleted) {
                std::cout << "Command buffer status: " << cmdBuf->status() << std::endl;
            }
        }
    }

    // 4) read results back
    memcpy(cpuParticles.data(), particleBuf->contents(), sizeof(GPUFluidParticle)*particleCount);

    // 5) Store final positions & velocities back to ECS
    for(size_t i=0, c=0; i<entityList.size(); i++){
        entt::entity e = entityList[i];
        auto &pos = registry.get<Components::Position>(e);
        auto &vel = registry.get<Components::Velocity>(e);
        auto &sphtemp = registry.get<Components::SPHTemp>(e);

        GPUFluidParticle &p = cpuParticles[i];
        pos.x = p.x;
        pos.y = p.y;
        vel.x = p.vx;
        vel.y = p.vy;
        sphtemp.density = p.density;
        sphtemp.pressure= p.pressure;
    }

    // release buffers
    particleBuf->release();
    gridBuf->release();
    paramsBuf->release();
}

// Register your system somewhere or just use this code as a normal ECS system
} // namespace Systems
