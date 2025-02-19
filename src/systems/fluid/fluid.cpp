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
 * Below is the .metal shader code in a string. In practice, you'd store this in a separate
 * .metal file and compile it with Xcode. For demonstration, we embed it inline.
 */
static const char* FluidMetalSource = R"METAL(
#include <metal_stdlib>
using namespace metal;

/**
 * Constants
 */
constant int MAX_PER_CELL = 64;

/**
 * Particle layout must match CPU side
 */
struct Particle {
    float x, y;
    float vx, vy;
    float vxHalf, vyHalf;
    float ax, ay;
    float mass, h, c;
    float density, pressure;
};

/**
 * GridCell layout
 */
struct GridCell {
    atomic_int count;
    int indices[MAX_PER_CELL];
};

/**
 * FluidParams
 */
struct FluidParams {
    float cellSize;
    int   gridDimX;
    int   gridDimY;
    int   gridMinX;
    int   gridMinY;
    float restDensity;
    float stiffness;
    float viscosity;
    float dt;
    float halfDt; // not used in these kernels, but possible
};

kernel void clearGrid(device GridCell *gridCells,
                      uint   gridCount [[ buffer(1) ]],
                      uint   tid [[ thread_position_in_grid ]])
{
    if(tid < gridCount){
        atomic_store_explicit(&(gridCells[tid].count), 0, memory_order_relaxed);
    }
}

kernel void assignCells(device Particle *particles,
                        uint particleCount [[ buffer(1) ]],
                        device GridCell *gridCells,
                        constant FluidParams &params [[ buffer(2) ]],
                        uint tid [[ thread_position_in_grid ]])
{
    if(tid >= particleCount) return;

    Particle p = particles[tid];
    int gx = int(floor(p.x / params.cellSize)) - params.gridMinX;
    int gy = int(floor(p.y / params.cellSize)) - params.gridMinY;

    if(gx<0) gx=0; 
    else if(gx >= params.gridDimX) gx = params.gridDimX-1;
    if(gy<0) gy=0;
    else if(gy >= params.gridDimY) gy = params.gridDimY-1;

    int cellID = gy*params.gridDimX + gx;

    int oldCount = atomic_fetch_add_explicit(&(gridCells[cellID].count), 1, memory_order_relaxed);
    if(oldCount < MAX_PER_CELL){
        gridCells[cellID].indices[oldCount] = int(tid);
    }
}

// 2D poly6
inline float poly6Coeff2D(float h){
    float h2 = h*h; float h4 = h2*h2; float h8 = h4*h4;
    return 4.0f/(M_PI*h8);
}

kernel void computeDensity(device Particle *particles,
                           uint particleCount [[ buffer(1) ]],
                           device GridCell *gridCells,
                           constant FluidParams &params [[ buffer(2) ]],
                           uint tid [[ thread_position_in_grid ]])
{
    if(tid >= particleCount) return;
    Particle pi = particles[tid];

    float hi = pi.h;
    float h2 = hi*hi;
    float coeff = poly6Coeff2D(hi);

    int gx = int(floor(pi.x/params.cellSize)) - params.gridMinX;
    int gy = int(floor(pi.y/params.cellSize)) - params.gridMinY;
    if(gx<0) gx=0; else if(gx >= params.gridDimX) gx=params.gridDimX-1;
    if(gy<0) gy=0; else if(gy >= params.gridDimY) gy=params.gridDimY-1;

    float accum=0.f;

    for(int nx=gx-1; nx<=gx+1; nx++){
        if(nx<0 || nx>=params.gridDimX) continue;
        for(int ny=gy-1; ny<=gy+1; ny++){
            if(ny<0 || ny>=params.gridDimY) continue;

            int cellID = ny*params.gridDimX + nx;
            int count = atomic_load_explicit(&gridCells[cellID].count, memory_order_relaxed);
            for(int c=0; c<count; c++){
                int j = gridCells[cellID].indices[c];
                Particle pj = particles[j];
                float dx = pi.x - pj.x;
                float dy = pi.y - pj.y;
                float r2 = dx*dx + dy*dy;
                if(r2 < h2){
                    float diff = (h2 - r2);
                    float w = coeff*diff*diff*diff;
                    accum += pj.mass*w;
                }
            }
        }
    }
    pi.density = accum;
    particles[tid] = pi;
}

// spiky
inline float spikyCoeff2D(float h){
    float h2 = h*h; float h4 = h2*h2; float h5 = h4*h;
    return -30.f/(M_PI*h5);
}
inline float viscLaplacianCoeff2D(float h){
    float h2 = h*h; float h4 = h2*h2; float h5 = h4*h;
    return 40.f/(M_PI*h5);
}

kernel void computeForces(device Particle *particles,
                          uint particleCount [[ buffer(1) ]],
                          device GridCell *gridCells,
                          constant FluidParams &params [[ buffer(2) ]],
                          uint tid [[ thread_position_in_grid ]])
{
    if(tid >= particleCount) return;
    Particle pi = particles[tid];
    float rhoi = pi.density;
    if(rhoi<1e-12f){
        pi.ax=0.f; pi.ay=0.f;
        particles[tid]=pi;
        return;
    }
    float pressi = params.stiffness*(rhoi - params.restDensity);
    if(pressi<0.f) pressi=0.f;

    int gx = int(floor(pi.x/params.cellSize)) - params.gridMinX;
    int gy = int(floor(pi.y/params.cellSize)) - params.gridMinY;
    if(gx<0) gx=0; else if(gx>=params.gridDimX) gx=params.gridDimX-1;
    if(gy<0) gy=0; else if(gy>=params.gridDimY) gy=params.gridDimY-1;

    float sumAx=0.f; float sumAy=0.f;

    for(int nx=gx-1; nx<=gx+1; nx++){
        if(nx<0||nx>=params.gridDimX) continue;
        for(int ny=gy-1; ny<=gy+1; ny++){
            if(ny<0||ny>=params.gridDimY) continue;
            int cellID = ny*params.gridDimX + nx;
            int count = atomic_load_explicit(&gridCells[cellID].count, memory_order_relaxed);
            for(int c=0; c<count; c++){
                int j = gridCells[cellID].indices[c];
                if(j==tid) continue;
                Particle pj = particles[j];
                float rhoj = pj.density;
                if(rhoj<1e-12f) continue;
                float pressj = params.stiffness*(rhoj - params.restDensity);
                if(pressj<0.f) pressj=0.f;

                float dx = pi.x - pj.x;
                float dy = pi.y - pj.y;
                float r2 = dx*dx + dy*dy;
                if(r2<1e-12f) continue;

                float hj = pj.h;
                float h_ij=0.5f*(pi.h+hj);
                if(r2 >= (h_ij*h_ij)) continue;

                float r = sqrt(r2);
                float term = (pressi/(rhoi*rhoi))+(pressj/(rhoj*rhoj));
                float spikyF= spikyCoeff2D(h_ij);
                float diff = (h_ij - r);
                float w_spiky = spikyF*(diff*diff);

                float rx = dx/r;
                float ry = dy/r;

                float fPress = - pj.mass*term*w_spiky;
                float fx = fPress*rx;
                float fy = fPress*ry;

                // Visc
                float vx_ij = pi.vx - pj.vx;
                float vy_ij = pi.vy - pj.vy;
                float lapF = viscLaplacianCoeff2D(h_ij);
                float w_visc = lapF*diff;
                float fVisc = params.viscosity * pj.mass*(w_visc/rhoj);
                fx -= fVisc*vx_ij;
                fy -= fVisc*vy_ij;

                sumAx += fx;
                sumAy += fy;
            }
        }
    }

    pi.pressure=pressi;
    pi.ax=sumAx; pi.ay=sumAy;
    particles[tid] = pi;
}

/**
 * velocityVerletHalf:
 *   vHalf = v + 0.5*a*dt
 *   x     = x + vHalf*dt
 */
kernel void velocityVerletHalf(device Particle *particles,
                               uint particleCount [[ buffer(1) ]],
                               constant FluidParams &params [[ buffer(2) ]],
                               uint tid [[ thread_position_in_grid ]])
{
    if(tid >= particleCount) return;
    Particle p = particles[tid];
    float vxH = p.vx + 0.5f*p.ax*params.dt;
    float vyH = p.vy + 0.5f*p.ay*params.dt;
    p.x += vxH*params.dt;
    p.y += vyH*params.dt;

    p.vxHalf=vxH;
    p.vyHalf=vyH;
    particles[tid]=p;
}

/**
 * velocityVerletFinish:
 *   v = vHalf + 0.5*a*dt
 */
kernel void velocityVerletFinish(device Particle *particles,
                                 uint particleCount [[ buffer(1) ]],
                                 constant FluidParams &params [[ buffer(2) ]],
                                 uint tid [[ thread_position_in_grid ]])
{
    if(tid >= particleCount) return;
    Particle p = particles[tid];
    p.vx = p.vxHalf + 0.5f*p.ax*params.dt;
    p.vy = p.vyHalf + 0.5f*p.ay*params.dt;
    particles[tid]=p;
}
)METAL";

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
    if(!device) {
        std::cerr << "No Metal device found. This requires macOS with Metal support.\n";
        return;
    }
    commandQueue = device->newCommandQueue();

    // Create library from the inline metal source
    NS::Error* error = nullptr;
    NS::String* msrc = NS::String::string(FluidMetalSource, NS::UTF8StringEncoding);
    MTL::Library* library = device->newLibrary(msrc, nullptr, &error);
    if(!library) {
        std::cerr << "Failed to compile metal source: " 
                  << (error ? error->localizedDescription()->utf8String() : "") << "\n";
        return;
    }

    // Create pipeline states
    clearGridPSO      = createPSO("clearGrid",           library);
    assignCellsPSO    = createPSO("assignCells",         library);
    computeDensityPSO = createPSO("computeDensity",      library);
    computeForcesPSO  = createPSO("computeForces",       library);
    verletHalfPSO     = createPSO("velocityVerletHalf",  library);
    verletFinishPSO   = createPSO("velocityVerletFinish",library);

    library->release();
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
    std::cout << "fluid system update" << std::endl;

    std::cout << "Total entities in registry before reserve: " << registry.size() << std::endl;
    std::cout << "Registry storage sizes before reserve:" << std::endl;
    std::cout << "Position: " << registry.storage<Components::Position>().size() << std::endl;
    std::cout << "Velocity: " << registry.storage<Components::Velocity>().size() << std::endl;
    std::cout << "Mass: " << registry.storage<Components::Mass>().size() << std::endl;
    std::cout << "Phase: " << registry.storage<Components::ParticlePhase>().size() << std::endl;
    std::cout << "SmoothingLength: " << registry.storage<Components::SmoothingLength>().size() << std::endl;
    std::cout << "SpeedOfSound: " << registry.storage<Components::SpeedOfSound>().size() << std::endl;
    std::cout << "SPHTemp: " << registry.storage<Components::SPHTemp>().size() << std::endl;
    return;


    if(!device || !commandQueue) {
        // fallback to CPU or do nothing
        return;
    }

    // 1) Gather fluid particles from ECS
    std::vector<GPUFluidParticle> cpuParticles;
    auto posSize = registry.storage<Components::Position>().size();
    std::cout << "Number of Position components: " << posSize << std::endl << std::flush;
    cpuParticles.reserve(registry.storage<Components::Position>().size()); // or registry.size_hint() if you like
    return;

    // We'll also store an array of entt::entity so we can map back
    std::vector<entt::entity> entityList;

    // Grabbing a typical dt:
    float dt = float(SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration);

    std::cout << "gathering fluid particles 1" << std::endl << std::flush;
    std::cout << "Registry sizes: " << std::endl << std::flush;
    std::cout << " Position: " << registry.storage<Components::Position>().size() << std::endl << std::flush;
    std::cout << " Velocity: " << registry.storage<Components::Velocity>().size() << std::endl << std::flush;
    std::cout << " Mass: " << registry.storage<Components::Mass>().size() << std::endl << std::flush;
    std::cout << " Phase: " << registry.storage<Components::ParticlePhase>().size() << std::endl << std::flush;
    std::cout << " SmoothingLength: " << registry.storage<Components::SmoothingLength>().size() << std::endl << std::flush;
    std::cout << " SpeedOfSound: " << registry.storage<Components::SpeedOfSound>().size() << std::endl << std::flush;
    std::cout << " SPHTemp: " << registry.storage<Components::SPHTemp>().size() << std::endl << std::flush;
    // flush the buffer
    std::cout << std::flush;
    return;

    auto view = registry.view<Components::Position,
                              Components::Velocity,
                              Components::Mass,
                              Components::ParticlePhase,
                              Components::SmoothingLength,
                              Components::SpeedOfSound,
                              Components::SPHTemp>();

    std::cout << "gathering fluid particles 2" << std::endl;
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

    std::cout << "creating gpu buffers" << std::endl;
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
            enc->setBytes(&particleCount, sizeof(int), 1);
            enc->setBuffer(paramsBuf, 0, 2);

            MTL::Size threads(particleCount,1,1);
            MTL::Size tgroup(64,1,1);
            enc->dispatchThreadgroups(threads, tgroup);
            enc->endEncoding();
            cmdBuf->commit();
            cmdBuf->waitUntilCompleted();
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
        }

        // (f) velocityVerletFinish
        {
            auto cmdBuf = commandQueue->commandBuffer();
            auto enc = cmdBuf->computeCommandEncoder();
            enc->setComputePipelineState(verletFinishPSO);
            enc->setBuffer(particleBuf, 0, 0);
            enc->setBytes(&particleCount, sizeof(int), 1);
            enc->setBuffer(paramsBuf, 0, 2);

            MTL::Size threads(particleCount,1,1);
            MTL::Size tgroup(64,1,1);
            enc->dispatchThreadgroups(threads, tgroup);
            enc->endEncoding();
            cmdBuf->commit();
            cmdBuf->waitUntilCompleted();
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
