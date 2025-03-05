/**
 * @fileoverview fluid.cpp
 * @brief Implements the GPU-based 2D SPH fluid system with integrated rigid-fluid impulse solver.
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
#include "nbody/core/profile.hpp"
#include "nbody/entities/entity_components.hpp"
#include "nbody/components/sph.hpp"
#include "nbody/math/polygon.hpp"

#include "nbody/systems/fluid/fluid.hpp"

namespace Systems {

FluidSystem::FluidSystem() {
    device = MTL::CreateSystemDefaultDevice();
    if (!device) {
        std::cerr << "No Metal device found. FluidSystem will be disabled." << std::endl;
        return;
    }
    commandQueue = device->newCommandQueue();

    NS::Error* error = nullptr;
    // Load the Metal library (compiled .metallib)
    NS::String* libPath = NS::String::string("build/fluid_kernels.metallib", NS::UTF8StringEncoding);
    metalLibrary = device->newLibrary(libPath, &error);
    if (!metalLibrary) {
        std::cerr << "Failed to load metal library: "
                  << (error ? error->localizedDescription()->utf8String() : "Unknown") << std::endl;
        return;
    }

    // Create all pipeline states
    clearGridPSO          = createPSO("clearGrid",           metalLibrary);
    assignCellsPSO        = createPSO("assignCells",         metalLibrary);
    computeDensityPSO     = createPSO("computeDensity",      metalLibrary);
    computeForcesPSO      = createPSO("computeForces",       metalLibrary);
    verletHalfPSO         = createPSO("velocityVerletHalf",  metalLibrary);
    verletFinishPSO       = createPSO("velocityVerletFinish",metalLibrary);
    computeBoundingBoxPSO = createPSO("computeBoundingBox",  metalLibrary);

    // Existing GPU-based position solver
    rigidFluidPositionPSO = createPSO("rigidFluidPositionSolver", metalLibrary);

    // **New** impulse solver pipeline
    rigidFluidImpulsePSO  = createPSO("rigidFluidImpulseSolver", metalLibrary);
}

FluidSystem::~FluidSystem() {
    if (clearGridPSO)          { clearGridPSO->release(); }
    if (assignCellsPSO)        { assignCellsPSO->release(); }
    if (computeDensityPSO)     { computeDensityPSO->release(); }
    if (computeForcesPSO)      { computeForcesPSO->release(); }
    if (verletHalfPSO)         { verletHalfPSO->release(); }
    if (verletFinishPSO)       { verletFinishPSO->release(); }
    if (computeBoundingBoxPSO) { computeBoundingBoxPSO->release(); }
    if (rigidFluidPositionPSO) { rigidFluidPositionPSO->release(); }
    if (rigidFluidImpulsePSO)  { rigidFluidImpulsePSO->release(); }
    if (commandQueue)          { commandQueue->release(); }
    if (metalLibrary)          { metalLibrary->release(); }
}

MTL::ComputePipelineState* FluidSystem::createPSO(const char* fnName, MTL::Library* lib) {
    if (!lib) {
        return nullptr;
    }

    NS::String* functionName = NS::String::string(fnName, NS::UTF8StringEncoding);
    MTL::Function* fn = lib->newFunction(functionName);
    if (!fn) {
        std::cerr << "Missing function " << fnName << " in metal library." << std::endl;
        return nullptr;
    }

    NS::Error* error = nullptr;
    MTL::ComputePipelineState* pso = device->newComputePipelineState(fn, &error);
    fn->release();
    if (!pso) {
        std::cerr << "Failed to create pipeline " << fnName << ": "
                  << (error ? error->localizedDescription()->utf8String() : "Unknown error") << std::endl;
    }
    return pso;
}

void FluidSystem::dispatchComputePass(
    MTL::ComputePipelineState* pipelineState,
    const std::function<void(MTL::ComputeCommandEncoder*)>& encodeFunc,
    size_t threadsCount,
    size_t threadsPerGroup) const
{
    if (!pipelineState || !commandQueue) {
        return;
    }
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

    auto view = registry.view<
        Components::Position,
        Components::Velocity,
        Components::Mass,
        Components::ParticlePhase,
        Components::SmoothingLength,
        Components::SpeedOfSound,
        Components::SPHTemp
    >();

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
        p.mass = static_cast<float>(mass.value);
        p.h    = sl.value <= 0.0 ? 0.05f : static_cast<float>(sl.value);
        p.c    = static_cast<float>(snd.value);
        p.density = 0.f;
        p.pressure= 0.f;

        cpuParticles.push_back(p);
        entityList.push_back(e);
    }

    return cpuParticles;
}

std::vector<GPURigidBody> FluidSystem::gatherRigidBodies(
    entt::registry& registry,
    std::vector<entt::entity>& rigidEntityList) const
{
    std::vector<GPURigidBody> cpuRigids;
    cpuRigids.reserve(registry.storage<Components::Shape>().size());

    // We'll collect all non-fluid shapes
    auto view = registry.view<Components::Position, Components::Shape>();

    for (auto e : view) {
        // Skip if it's a fluid entity
        if (registry.all_of<Components::ParticlePhase>(e)) {
            const auto &phase = registry.get<Components::ParticlePhase>(e);
            if (phase.phase == Components::Phase::Liquid) {
                continue;
            }
        }

        const auto &pos   = view.get<Components::Position>(e);
        const auto &shape = view.get<Components::Shape>(e);

        GPURigidBody rb{};
        rb.posX  = static_cast<float>(pos.x);
        rb.posY  = static_cast<float>(pos.y);
        rb.angle = 0.0f;  // default if no AngularPosition
        rb.accumFx = 0.f;
        rb.accumFy = 0.f;
        rb.accumTorque = 0.f;

        // If the entity has AngularPosition, read it
        if (auto *angPos = registry.try_get<Components::AngularPosition>(e)) {
            rb.angle = static_cast<float>(angPos->angle);
        }

        // If the entity has a Velocity or AngularVelocity, read them
        if (auto *vel = registry.try_get<Components::Velocity>(e)) {
            rb.vx = static_cast<float>(vel->x);
            rb.vy = static_cast<float>(vel->y);
        }
        if (auto *angVel = registry.try_get<Components::AngularVelocity>(e)) {
            rb.omega = static_cast<float>(angVel->omega);
        }

        // Read mass & inertia if present
        float mVal = 1.f;
        float iVal = 1.f;
        if (auto *massC = registry.try_get<Components::Mass>(e)) {
            mVal = static_cast<float>(massC->value);
        }
        if (auto *inertiaC = registry.try_get<Components::Inertia>(e)) {
            iVal = static_cast<float>(inertiaC->I);
        }
        rb.mass = mVal;
        rb.inertia = iVal;

        // Attempt bounding box from shape size, or from the polygon
        rb.minX = rb.posX - 0.5f;
        rb.maxX = rb.posX + 0.5f;
        rb.minY = rb.posY - 0.5f;
        rb.maxY = rb.posY + 0.5f;

        if (shape.type == Components::ShapeType::Circle) {
            rb.shapeType = GPURigidShapeType::Circle;
            rb.radius    = static_cast<float>(shape.size);
            rb.vertCount = 0;
            // bounding box for broad phase
            rb.minX = rb.posX - rb.radius;
            rb.maxX = rb.posX + rb.radius;
            rb.minY = rb.posY - rb.radius;
            rb.maxY = rb.posY + rb.radius;
        }
        else if (shape.type == Components::ShapeType::Polygon) {
            rb.shapeType = GPURigidShapeType::Polygon;
            rb.radius    = 0.f;

            // Must have PolygonShape
            if (!registry.all_of<PolygonShape>(e)) {
                continue;
            }
            const auto &poly = registry.get<PolygonShape>(e);

            int actualCount = static_cast<int>(poly.vertices.size());
            if (actualCount > GPU_POLYGON_MAX_VERTS) {
                actualCount = GPU_POLYGON_MAX_VERTS;
            }
            rb.vertCount = actualCount;

            double c = std::cos(rb.angle);
            double s = std::sin(rb.angle);

            // Build bounding box as we go
            float minx = 1e9f;
            float maxx = -1e9f;
            float miny = 1e9f;
            float maxy = -1e9f;

            for (int i = 0; i < actualCount; i++) {
                double lx = poly.vertices[i].x;
                double ly = poly.vertices[i].y;
                // transform to world
                double wx = pos.x + (lx*c - ly*s);
                double wy = pos.y + (lx*s + ly*c);

                rb.vertsX[i] = static_cast<float>(wx);
                rb.vertsY[i] = static_cast<float>(wy);

                if (wx < minx) minx = (float)wx;
                if (wx > maxx) maxx = (float)wx;
                if (wy < miny) miny = (float)wy;
                if (wy > maxy) maxy = (float)wy;
            }
            rb.minX = minx;
            rb.maxX = maxx;
            rb.minY = miny;
            rb.maxY = maxy;
        }
        else {
            // skip or convert squares to polygons
            continue;
        }

        cpuRigids.push_back(rb);
        rigidEntityList.push_back(e);
    }

    return cpuRigids;
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
    int numThreadgroups) const
{
    size_t blockSize = 256;
    size_t localSharedBBox = sizeof(float) * 4 * blockSize;
    size_t localValidCount = sizeof(int);

    dispatchComputePass(
        computeBoundingBoxPSO,
        [&](MTL::ComputeCommandEncoder* enc) {
            enc->setBuffer(particleBuf,           0, 0);
            enc->setBuffer(boundingBoxParamsBuf,  0, 1);
            enc->setBuffer(boundingBoxBuf,        0, 2);

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

void FluidSystem::dispatchRigidFluidPosition(
    MTL::Buffer* particleBuf,
    int fluidCount,
    MTL::Buffer* rigidBuf,
    int rigidCount) const
{
    dispatchComputePass(
        rigidFluidPositionPSO,
        [&](MTL::ComputeCommandEncoder* enc) {
            enc->setBuffer(particleBuf, 0, 0);
            enc->setBytes(&fluidCount, sizeof(int), 1);

            enc->setBuffer(rigidBuf, 0, 2);
            enc->setBytes(&rigidCount, sizeof(int), 3);
        },
        static_cast<size_t>(fluidCount),
        256
    );
}

void FluidSystem::dispatchRigidFluidImpulse(
    MTL::Buffer* particleBuf,
    int fluidCount,
    MTL::Buffer* rigidBuf,
    MTL::Buffer* paramsBuf,
    int rigidCount) const
{
    // We'll do 1 thread per fluid particle, each loops over all rigid bodies (O(N*M)).
    dispatchComputePass(
        rigidFluidImpulsePSO,
        [&](MTL::ComputeCommandEncoder* enc) {
            // arg 0: fluid particles
            enc->setBuffer(particleBuf, 0, 0);
            // arg 1: fluidCount
            enc->setBytes(&fluidCount, sizeof(int), 1);
            // arg 2: rigidBuf
            enc->setBuffer(rigidBuf, 0, 2);
            // arg 3: rigidCount
            enc->setBytes(&rigidCount, sizeof(int), 3);
            // arg 4: params
            enc->setBuffer(paramsBuf, 0, 4);
        },
        static_cast<size_t>(fluidCount),
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

void FluidSystem::writeBackRigidBodies(
    entt::registry& registry,
    const std::vector<entt::entity>& rigidEntityList,
    MTL::Buffer* rigidBuf,
    int rigidCount) const
{
    // We'll do a final pass on CPU to apply the accumulated forces/torques to velocities
    // with damping or any other logic (similar to the old CPU impulse solver).
    if (rigidCount < 1 || !rigidBuf) return;

    auto* rbData = static_cast<GPURigidBody*>(rigidBuf->contents());

    // We'll assume the GPU stored final accumFx, accumFy, accumTorque in the same struct
    // but hasn't yet applied them to vx, vy, omega. You can do it in the kernel or do it here.
    for (int i = 0; i < rigidCount; i++) {
        GPURigidBody &rb = rbData[i];

        // If mass or inertia is huge, effectively immovable
        float invMass    = (rb.mass > 1e-12f)    ? (1.f / rb.mass)    : 0.f;
        float invInertia = (rb.inertia > 1e-12f) ? (1.f / rb.inertia) : 0.f;

        // Apply force scaling or damping
        rb.vx += rb.accumFx * invMass;
        rb.vy += rb.accumFy * invMass;
        rb.vx *= getSpecificConfig().dampingFactor;
        rb.vy *= getSpecificConfig().dampingFactor;

        rb.omega += rb.accumTorque * invInertia;
        rb.omega *= getSpecificConfig().dampingFactor;

        // store back accum=0 for next frame
        rb.accumFx = 0.f;
        rb.accumFy = 0.f;
        rb.accumTorque = 0.f;
    }

    // Now copy them back to ECS
    for (int i = 0; i < rigidCount; i++) {
        entt::entity ent = rigidEntityList[static_cast<size_t>(i)];
        GPURigidBody &rb = rbData[i];

        // set velocity, angular velocity
        if (auto *vel = registry.try_get<Components::Velocity>(ent)) {
            vel->x = rb.vx;
            vel->y = rb.vy;
        }
        if (auto *angVel = registry.try_get<Components::AngularVelocity>(ent)) {
            angVel->omega = rb.omega;
        }
    }
}

void FluidSystem::multiStepVelocityVerlet(
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
    int rigidCount)
{
    PROFILE_SCOPE("FluidSystem::multiStepVelocityVerlet");

    float dt = float(getSystemConfig().SecondsPerTick * getSystemConfig().TimeAcceleration);
    float subDt = dt / float(getSpecificConfig().numSubSteps);

    const int BLOCK_SIZE = getSpecificConfig().threadsPerGroup;
    int numThreadgroups = (realCount + BLOCK_SIZE - 1) / BLOCK_SIZE;

    for (int step = 0; step < getSpecificConfig().numSubSteps; step++) {
        // A) velocityVerletHalf
        {
            GPUFluidParams params{};
            params.restDensity = getSpecificConfig().restDensity;
            params.stiffness   = getSpecificConfig().stiffness;
            params.viscosity   = getSpecificConfig().viscosity;
            params.dt          = subDt;
            params.halfDt      = 0.5f * subDt;
            params.particleCount = static_cast<unsigned int>(paddedCount);

            memcpy(paramsBuf->contents(), &params, sizeof(GPUFluidParams));
            dispatchVelocityVerletHalf(particleBuf, paramsBuf, realCount);
        }

        // B) bounding box
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

        // C) find maximum smoothing length (h) among particles
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

        // offset
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

        // D) fluid params for neighbor steps
        {
            GPUFluidParams params{};
            params.cellSize    = cellSize;
            params.gridMinX    = globalMinGx;
            params.gridMinY    = globalMinGy;
            params.gridDimX    = gridDimX;
            params.gridDimY    = gridDimY;
            params.restDensity = getSpecificConfig().restDensity;
            params.stiffness   = getSpecificConfig().stiffness;
            params.viscosity   = getSpecificConfig().viscosity;
            params.dt          = subDt;
            params.halfDt      = 0.5f * subDt;
            params.particleCount = static_cast<unsigned int>(paddedCount);

            memcpy(paramsBuf->contents(), &params, sizeof(GPUFluidParams));
        }

        auto gridBuf = device->newBuffer(
            sizeof(GPUGridCell) * gridSize,
            MTL::ResourceStorageModeShared
        );
        memset(gridBuf->contents(), 0, sizeof(GPUGridCell) * gridSize);

        dispatchClearGrid(gridBuf, gridSize);
        dispatchAssignCells(particleBuf, gridBuf, paramsBuf, realCount);
        dispatchComputeDensity(particleBuf, gridBuf, paramsBuf, realCount);
        dispatchComputeForces(particleBuf, gridBuf, paramsBuf, realCount);

        gridBuf->release();

        // F) velocityVerletFinish
        dispatchVelocityVerletFinish(particleBuf, paramsBuf, realCount);

        // G) GPU-based rigid-fluid position solver
        dispatchRigidFluidPosition(particleBuf, realCount, rigidBuf, rigidCount);

        // H) GPU-based rigid-fluid impulse solver
        if (rigidCount > 0) {
            // For each fluid, we compute drag/buoyancy etc. on each rigid,
            // using bounding boxes and atomic adds to accumFx/Fy/Torque.
            dispatchRigidFluidImpulse(particleBuf, realCount, rigidBuf, paramsBuf, rigidCount);
        }
    }

    // After sub-steps, we finalize rigid velocities on CPU
    writeBackRigidBodies(registry, rigidEntities, rigidBuf, rigidCount);
}

void FluidSystem::update(entt::registry& registry) {
    PROFILE_SCOPE("FluidSystem::update (GPU-based SPH)");

    if (!device || !commandQueue) {
        return;
    }

    // 1) Gather fluid
    std::vector<entt::entity> fluidEntityList;
    auto cpuParticles = gatherFluidParticles(registry, fluidEntityList);
    if (cpuParticles.empty()) {
        return;
    }
    int fluidCount = static_cast<int>(cpuParticles.size());

    // 2) Gather rigid
    std::vector<entt::entity> rigidEntityList;
    auto cpuRigids = gatherRigidBodies(registry, rigidEntityList);
    int rigidCount = static_cast<int>(cpuRigids.size());

    // 3) Pad fluid
    int paddedCount = 1;
    while (paddedCount < fluidCount) {
        paddedCount <<= 1;
    }

    // 4) GPU buffers for fluid
    auto particleBuf = device->newBuffer(
        sizeof(GPUFluidParticle) * paddedCount,
        MTL::ResourceStorageModeShared
    );
    memset(particleBuf->contents(), 0, sizeof(GPUFluidParticle)*paddedCount);
    memcpy(particleBuf->contents(), cpuParticles.data(), sizeof(GPUFluidParticle)*fluidCount);

    auto paramsBuf = device->newBuffer(sizeof(GPUFluidParams), MTL::ResourceStorageModeShared);

    // bounding box
    auto boundingBoxParamsBuf = device->newBuffer(sizeof(BBoxParams), MTL::ResourceStorageModeShared);
    memset(boundingBoxParamsBuf->contents(), 0, sizeof(BBoxParams));

    const int BLOCK_SIZE = 256;
    int numThreadgroups = (fluidCount + BLOCK_SIZE - 1) / BLOCK_SIZE;
    auto boundingBoxBuf = device->newBuffer(
        sizeof(float)*4*numThreadgroups,
        MTL::ResourceStorageModeShared
    );
    memset(boundingBoxBuf->contents(), 0, sizeof(float)*4*numThreadgroups);

    // 5) GPU buffers for rigid
    MTL::Buffer* rigidBuf = nullptr;
    if (rigidCount > 0) {
        rigidBuf = device->newBuffer(sizeof(GPURigidBody) * rigidCount,
                                     MTL::ResourceStorageModeShared);
        memset(rigidBuf->contents(), 0, sizeof(GPURigidBody)*rigidCount);
        memcpy(rigidBuf->contents(), cpuRigids.data(), sizeof(GPURigidBody)*rigidCount);
    }

    // 6) Multi-step velocity-verlet
    multiStepVelocityVerlet(
        registry,
        fluidEntityList,
        rigidEntityList,
        particleBuf,
        paramsBuf,
        boundingBoxParamsBuf,
        boundingBoxBuf,
        rigidBuf,
        fluidCount,
        paddedCount,
        rigidCount
    );

    // 7) Write fluid back
    writeBackToECS(registry, fluidEntityList, particleBuf, fluidCount);

    // Cleanup
    if (rigidBuf) {
        rigidBuf->release();
    }
    boundingBoxBuf->release();
    boundingBoxParamsBuf->release();
    particleBuf->release();
    paramsBuf->release();
}

} // namespace Systems