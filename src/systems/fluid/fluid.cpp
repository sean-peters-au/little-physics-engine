/**
 * @fileoverview fluid.cpp
 * @brief Implements the GPU-based 2D SPH fluid system with integrated rigid-fluid impulse solver.
 *
 * This version addresses bus errors by:
 * 1) Re-checking and reallocating the grid buffer each sub-step if the bounding box grows.
 * 2) Clamping gridDimX and gridDimY so they cannot be zero or negative.
 * 3) Ensuring the GPU pipeline doesn't access out-of-range indices.
 *
 * If you still experience a bus error, verify that fluid_kernels.metal
 * performs boundary checks on indices (especially in assignCells, etc.).
 */

#include "systems/fluid/fluid.hpp"

#include <cmath>
#include <vector>
#include <iostream>
#include <limits>
#include <climits>

#include <Metal/Metal.hpp>
#include <Foundation/Foundation.hpp>
#include <entt/entt.hpp>

#include "core/constants.hpp"
#include "core/profile.hpp"
#include "entities/entity_components.hpp"
#include "math/polygon.hpp"
#include "systems/fluid/fluid_kernels.h"

namespace Systems
{

/**
 * @brief Creates a Metal compute pipeline state from the given function name.
 * @param fnName Name of the function in the .metal library
 * @param lib The Metal library
 * @param device The Metal device
 * @return Pointer to the new compute pipeline state, or nullptr on error
 */
static MTL::ComputePipelineState* createComputePipeline(
    const char* fnName,
    MTL::Library* lib,
    MTL::Device* device)
{
    if (!lib || !device)
    {
        return nullptr;
    }

    NS::String* functionName = NS::String::string(fnName, NS::UTF8StringEncoding);
    MTL::Function* fn = lib->newFunction(functionName);
    if (!fn)
    {
        std::cerr << "Missing function " << fnName << " in metal library." << std::endl;
        return nullptr;
    }

    NS::Error* error = nullptr;
    MTL::ComputePipelineState* pso = device->newComputePipelineState(fn, &error);
    fn->release();
    if (!pso)
    {
        std::cerr << "Failed to create pipeline " << fnName << ": "
                  << (error ? error->localizedDescription()->utf8String() : "Unknown error")
                  << std::endl;
    }
    return pso;
}

FluidSystem::FluidSystem()
    : device_(nullptr),
      commandQueue_(nullptr),
      metalLibrary_(nullptr),
      clearGridPSO_(nullptr),
      assignCellsPSO_(nullptr),
      computeDensityPSO_(nullptr),
      computeForcesPSO_(nullptr),
      verletHalfPSO_(nullptr),
      verletFinishPSO_(nullptr),
      computeBoundingBoxPSO_(nullptr),
      rigidFluidImpulsePSO_(nullptr),
      rigidFluidPositionPSO_(nullptr),
      particleBuf_(nullptr),
      rigidBuf_(nullptr),
      paramsBuf_(nullptr),
      boundingBoxParamsBuf_(nullptr),
      boundingBoxBuf_(nullptr),
      gridBuf_(nullptr),
      maxFluidParticles_(0),
      maxRigidBodies_(0),
      maxGridSize_(0),
      maxThreadgroups_(0)
{
    device_ = MTL::CreateSystemDefaultDevice();
    if (!device_)
    {
        std::cerr << "No Metal device found. FluidSystem will be disabled." << std::endl;
        return;
    }

    commandQueue_ = device_->newCommandQueue();

    NS::Error* error = nullptr;
    NS::String* libPath = NS::String::string("build/fluid_kernels.metallib", NS::UTF8StringEncoding);
    metalLibrary_ = device_->newLibrary(libPath, &error);
    if (!metalLibrary_)
    {
        std::cerr << "Failed to load metal library: "
                  << (error ? error->localizedDescription()->utf8String() : "Unknown")
                  << std::endl;
        return;
    }

    clearGridPSO_          = createComputePipeline("clearGrid",            metalLibrary_, device_);
    assignCellsPSO_        = createComputePipeline("assignCells",          metalLibrary_, device_);
    computeDensityPSO_     = createComputePipeline("computeDensity",       metalLibrary_, device_);
    computeForcesPSO_      = createComputePipeline("computeForces",        metalLibrary_, device_);
    verletHalfPSO_         = createComputePipeline("velocityVerletHalf",   metalLibrary_, device_);
    verletFinishPSO_       = createComputePipeline("velocityVerletFinish", metalLibrary_, device_);
    computeBoundingBoxPSO_ = createComputePipeline("computeBoundingBox",   metalLibrary_, device_);
    rigidFluidImpulsePSO_  = createComputePipeline("rigidFluidImpulseSolver",  metalLibrary_, device_);
    rigidFluidPositionPSO_ = createComputePipeline("rigidFluidPositionSolver", metalLibrary_, device_);
}

FluidSystem::~FluidSystem()
{
    if (clearGridPSO_)          { clearGridPSO_->release(); }
    if (assignCellsPSO_)        { assignCellsPSO_->release(); }
    if (computeDensityPSO_)     { computeDensityPSO_->release(); }
    if (computeForcesPSO_)      { computeForcesPSO_->release(); }
    if (verletHalfPSO_)         { verletHalfPSO_->release(); }
    if (verletFinishPSO_)       { verletFinishPSO_->release(); }
    if (computeBoundingBoxPSO_) { computeBoundingBoxPSO_->release(); }
    if (rigidFluidPositionPSO_) { rigidFluidPositionPSO_->release(); }
    if (rigidFluidImpulsePSO_)  { rigidFluidImpulsePSO_->release(); }
    if (commandQueue_)          { commandQueue_->release(); }
    if (metalLibrary_)          { metalLibrary_->release(); }

    if (particleBuf_)           { particleBuf_->release(); }
    if (rigidBuf_)              { rigidBuf_->release(); }
    if (paramsBuf_)             { paramsBuf_->release(); }
    if (boundingBoxParamsBuf_)  { boundingBoxParamsBuf_->release(); }
    if (boundingBoxBuf_)        { boundingBoxBuf_->release(); }
    if (gridBuf_)               { gridBuf_->release(); }
}

void FluidSystem::encodeComputePass(
    MTL::ComputeCommandEncoder* enc,
    MTL::ComputePipelineState* pipelineState,
    const std::function<void(MTL::ComputeCommandEncoder*)>& encodeFunc,
    size_t threadsCount,
    size_t threadsPerGroup) const
{
    if (!enc || !pipelineState)
    {
        return;
    }

    enc->setComputePipelineState(pipelineState);
    encodeFunc(enc);

    size_t groups = (threadsCount + threadsPerGroup - 1) / threadsPerGroup;
    MTL::Size tgSize(threadsPerGroup, 1, 1);
    MTL::Size numGroups(groups, 1, 1);
    enc->dispatchThreadgroups(numGroups, tgSize);
}

void FluidSystem::initBuffersIfNeeded(
    int fluidCount,
    int paddedCount,
    int rigidCount,
    int threadgroups,
    int gridSize)
{
    // If there's no valid device, do nothing
    if (!device_)
    {
        return;
    }

    // Expand particle buffer if needed
    if (paddedCount > maxFluidParticles_)
    {
        if (particleBuf_)
        {
            particleBuf_->release();
        }
        std::cout << "[FluidSystem] Resizing particleBuf_ to " << paddedCount << std::endl;
        particleBuf_ = device_->newBuffer(
            sizeof(GPUFluidParticle) * paddedCount,
            MTL::ResourceStorageModeShared
        );
        maxFluidParticles_ = paddedCount;
    }

    // Expand rigid buffer if needed
    if (rigidCount > maxRigidBodies_)
    {
        if (rigidBuf_)
        {
            rigidBuf_->release();
        }
        std::cout << "[FluidSystem] Resizing rigidBuf_ to " << rigidCount << std::endl;
        rigidBuf_ = device_->newBuffer(
            sizeof(GPURigidBody) * rigidCount,
            MTL::ResourceStorageModeShared
        );
        maxRigidBodies_ = rigidCount;
    }

    // Create param buffers if not yet allocated
    if (!paramsBuf_)
    {
        paramsBuf_ = device_->newBuffer(sizeof(GPUFluidParams), MTL::ResourceStorageModeShared);
    }
    if (!boundingBoxParamsBuf_)
    {
        boundingBoxParamsBuf_ = device_->newBuffer(sizeof(BBoxParams), MTL::ResourceStorageModeShared);
    }

    // Expand bounding box partials if needed
    if (threadgroups > maxThreadgroups_)
    {
        if (boundingBoxBuf_)
        {
            boundingBoxBuf_->release();
        }
        std::cout << "[FluidSystem] Resizing boundingBoxBuf_ to " << threadgroups << " groups" << std::endl;
        boundingBoxBuf_ = device_->newBuffer(
            sizeof(float) * 4 * threadgroups,
            MTL::ResourceStorageModeShared
        );
        maxThreadgroups_ = threadgroups;
    }

    // Expand the neighbor grid buffer if needed
    if (gridSize > maxGridSize_)
    {
        if (gridBuf_)
        {
            gridBuf_->release();
        }
        std::cout << "[FluidSystem] Resizing gridBuf_ to " << gridSize << " cells" << std::endl;
        gridBuf_ = device_->newBuffer(
            sizeof(GPUGridCell) * gridSize,
            MTL::ResourceStorageModeShared
        );
        maxGridSize_ = gridSize;
    }
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
        Components::SpeedOfSound,
        Components::SPHTemp
    >();

    for (auto e : view)
    {
        const auto& phase = view.get<Components::ParticlePhase>(e);
        if (phase.phase != Components::Phase::Liquid)
        {
            continue;
        }

        const auto& pos  = view.get<Components::Position>(e);
        const auto& vel  = view.get<Components::Velocity>(e);
        const auto& mass = view.get<Components::Mass>(e);
        const auto& snd  = view.get<Components::SpeedOfSound>(e);
        const auto& spht = view.get<Components::SPHTemp>(e);

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
        p.h    = getSpecificConfig().gridConfig.smoothingLength;
        p.c    = static_cast<float>(snd.value);
        p.density = spht.density;
        p.pressure= spht.pressure;

        cpuParticles.push_back(p);
        entityList.push_back(e);
    }

    return cpuParticles;
}

std::vector<GPURigidBody> FluidSystem::gatherRigidBodies(
    entt::registry& registry,
    std::vector<entt::entity>& rigidEntityList) const
{
    PROFILE_SCOPE("FluidSystem::gatherRigidBodies");

    std::vector<GPURigidBody> cpuRigids;
    cpuRigids.reserve(registry.storage<Components::Shape>().size());

    auto view = registry.view<Components::Position, Components::Shape>();

    for (auto e : view)
    {
        // If it is a fluid particle, skip
        if (registry.all_of<Components::ParticlePhase>(e))
        {
            const auto &phase = registry.get<Components::ParticlePhase>(e);
            if (phase.phase == Components::Phase::Liquid)
            {
                continue;
            }
        }

        const auto &pos   = view.get<Components::Position>(e);
        const auto &shape = view.get<Components::Shape>(e);

        GPURigidBody rb{};
        rb.posX  = static_cast<float>(pos.x);
        rb.posY  = static_cast<float>(pos.y);
        rb.angle = 0.0f;
        rb.accumFx = 0.f;
        rb.accumFy = 0.f;
        rb.accumTorque = 0.f;

        if (auto *angPos = registry.try_get<Components::AngularPosition>(e))
        {
            rb.angle = static_cast<float>(angPos->angle);
        }
        if (auto *vel = registry.try_get<Components::Velocity>(e))
        {
            rb.vx = static_cast<float>(vel->x);
            rb.vy = static_cast<float>(vel->y);
        }
        if (auto *angVel = registry.try_get<Components::AngularVelocity>(e))
        {
            rb.omega = static_cast<float>(angVel->omega);
        }

        float mVal = 1.f;
        float iVal = 1.f;
        if (auto *massC = registry.try_get<Components::Mass>(e))
        {
            mVal = static_cast<float>(massC->value);
        }
        if (auto *inertiaC = registry.try_get<Components::Inertia>(e))
        {
            iVal = static_cast<float>(inertiaC->I);
        }
        rb.mass = mVal;
        rb.inertia = iVal;

        rb.minX = rb.posX - 0.5f;
        rb.maxX = rb.posX + 0.5f;
        rb.minY = rb.posY - 0.5f;
        rb.maxY = rb.posY + 0.5f;

        if (shape.type == Components::ShapeType::Circle)
        {
            rb.shapeType = GPURigidShapeType::Circle;
            rb.radius    = static_cast<float>(shape.size);
            rb.vertCount = 0;

            rb.minX = rb.posX - rb.radius;
            rb.maxX = rb.posX + rb.radius;
            rb.minY = rb.posY - rb.radius;
            rb.maxY = rb.posY + rb.radius;
        }
        else if (shape.type == Components::ShapeType::Polygon)
        {
            rb.shapeType = GPURigidShapeType::Polygon;
            rb.radius    = 0.f;

            if (!registry.all_of<PolygonShape>(e))
            {
                continue;
            }
            const auto &poly = registry.get<PolygonShape>(e);

            int actualCount = static_cast<int>(poly.vertices.size());
            if (actualCount > GPU_POLYGON_MAX_VERTS)
            {
                actualCount = GPU_POLYGON_MAX_VERTS;
            }
            rb.vertCount = actualCount;

            double c = std::cos(rb.angle);
            double s = std::sin(rb.angle);

            float minx =  std::numeric_limits<float>::max();
            float maxx = -std::numeric_limits<float>::max();
            float miny =  std::numeric_limits<float>::max();
            float maxy = -std::numeric_limits<float>::max();

            for (int i = 0; i < actualCount; i++)
            {
                double lx = poly.vertices[i].x;
                double ly = poly.vertices[i].y;
                double wx = pos.x + (lx * c - ly * s);
                double wy = pos.y + (lx * s + ly * c);

                rb.vertsX[i] = static_cast<float>(wx);
                rb.vertsY[i] = static_cast<float>(wy);

                if (wx < minx) { minx = static_cast<float>(wx); }
                if (wx > maxx) { maxx = static_cast<float>(wx); }
                if (wy < miny) { miny = static_cast<float>(wy); }
                if (wy > maxy) { maxy = static_cast<float>(wy); }
            }
            rb.minX = minx;
            rb.maxX = maxx;
            rb.minY = miny;
            rb.maxY = maxy;
        }
        else
        {
            // Unrecognized shape, skip
            continue;
        }

        cpuRigids.push_back(rb);
        rigidEntityList.push_back(e);
    }

    return cpuRigids;
}

void FluidSystem::reduceBoundingBoxOnCPU(
    MTL::Buffer* boundingBoxBuf,
    int numThreadgroups,
    float& minX,
    float& maxX,
    float& minY,
    float& maxY) const
{
    PROFILE_SCOPE("FluidSystem::reduceBoundingBoxOnCPU");

    struct BBox
    {
        float minX;
        float maxX;
        float minY;
        float maxY;
    };

    const BBox* partials = static_cast<const BBox*>(boundingBoxBuf->contents());
    if (!partials)
    {
        // If something went wrong with the buffer, fallback
        minX = minY = -1.f;
        maxX = maxY = 1.f;
        return;
    }

    minX =  std::numeric_limits<float>::max();
    maxX = -std::numeric_limits<float>::max();
    minY =  std::numeric_limits<float>::max();
    maxY = -std::numeric_limits<float>::max();

    for (int i = 0; i < numThreadgroups; i++)
    {
        const BBox& box = partials[i];
        if (box.minX < minX) { minX = box.minX; }
        if (box.maxX > maxX) { maxX = box.maxX; }
        if (box.minY < minY) { minY = box.minY; }
        if (box.maxY > maxY) { maxY = box.maxY; }
    }

    // Additional clamp in case bounding box is inverted or invalid
    if (minX > maxX)
    {
        float tmp = minX;
        minX = maxX;
        maxX = tmp;
    }
    if (minY > maxY)
    {
        float tmp = minY;
        minY = maxY;
        maxY = tmp;
    }
}

void FluidSystem::writeBackToECS(
    entt::registry& registry,
    const std::vector<entt::entity>& entityList,
    MTL::Buffer* particleBuf,
    int realCount) const
{
    PROFILE_SCOPE("FluidSystem::writeBackToECS");
    const auto* finalParticles = static_cast<const GPUFluidParticle*>(particleBuf->contents());
    if (!finalParticles)
    {
        return;
    }

    for (int i = 0; i < realCount; i++)
    {
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
    PROFILE_SCOPE("FluidSystem::writeBackRigidBodies");
    if (rigidCount < 1 || !rigidBuf)
    {
        return;
    }

    auto* rbData = static_cast<GPURigidBody*>(rigidBuf->contents());
    if (!rbData)
    {
        return;
    }

    // First, apply accum forces & torques
    for (int i = 0; i < rigidCount; i++)
    {
        GPURigidBody &rb = rbData[i];
        float invMass    = (rb.mass > 1e-12f)    ? (1.f / rb.mass)    : 0.f;
        float invInertia = (rb.inertia > 1e-12f) ? (1.f / rb.inertia) : 0.f;

        rb.vx += rb.accumFx * invMass;
        rb.vy += rb.accumFy * invMass;
        rb.vx *= getSpecificConfig().dampingFactor;
        rb.vy *= getSpecificConfig().dampingFactor;

        rb.omega += rb.accumTorque * invInertia;
        rb.omega *= getSpecificConfig().dampingFactor;

        rb.accumFx = 0.f;
        rb.accumFy = 0.f;
        rb.accumTorque = 0.f;
    }

    // Then push velocity changes back into ECS
    for (int i = 0; i < rigidCount; i++)
    {
        entt::entity ent = rigidEntityList[static_cast<size_t>(i)];
        GPURigidBody &rb = rbData[i];

        if (auto *vel = registry.try_get<Components::Velocity>(ent))
        {
            vel->x = rb.vx;
            vel->y = rb.vy;
        }
        if (auto *angVel = registry.try_get<Components::AngularVelocity>(ent))
        {
            angVel->omega = rb.omega;
        }
    }
}

void FluidSystem::multiStepVelocityVerlet(
    entt::registry& registry,
    const std::vector<entt::entity>& fluidEntities,
    const std::vector<entt::entity>& rigidEntities,
    int realCount,
    int paddedCount,
    int rigidCount)
{
    PROFILE_SCOPE("FluidSystem::multiStepVelocityVerlet");

    float dt = static_cast<float>(getSharedSystemConfig().SecondsPerTick * getSharedSystemConfig().TimeAcceleration);
    float subDt = dt / static_cast<float>(getSpecificConfig().numSubSteps);

    int blockSize = getSpecificConfig().threadsPerGroup;
    int numThreadgroups = (realCount + blockSize - 1) / blockSize;

    for (int step = 0; step < getSpecificConfig().numSubSteps; step++)
    {
        // --------------------------------------------------------------------
        // 1) velocityVerletHalf + bounding box
        {
            MTL::CommandBuffer* cmdBuf = commandQueue_->commandBuffer();
            if (!cmdBuf)
            {
                return;
            }

            {
                MTL::ComputeCommandEncoder* enc = cmdBuf->computeCommandEncoder();
                if (enc)
                {
                    // velocityVerletHalf
                    GPUFluidParams params{};
                    
                    // Base SPH parameters
                    params.gravity     = getSpecificConfig().gravity;
                    params.restDensity = getSpecificConfig().restDensity;
                    params.stiffness   = getSpecificConfig().stiffness;
                    params.viscosity   = getSpecificConfig().viscosity;
                    
                    // Time parameters
                    params.dt          = subDt;
                    params.halfDt      = 0.5f * subDt;
                    params.particleCount = static_cast<unsigned int>(paddedCount);
                    
                    // Set position solver configuration
                    params.positionSolver.safetyMargin = getSpecificConfig().positionSolver.safetyMargin;
                    params.positionSolver.relaxFactor = getSpecificConfig().positionSolver.relaxFactor;
                    params.positionSolver.maxCorrection = getSpecificConfig().positionSolver.maxCorrection;
                    params.positionSolver.maxVelocityUpdate = getSpecificConfig().positionSolver.maxVelocityUpdate;
                    params.positionSolver.minSafeDistance = getSpecificConfig().positionSolver.minSafeDistance;
                    params.positionSolver.velocityDamping = getSpecificConfig().positionSolver.velocityDamping;
                    params.positionSolver.minPositionChange = getSpecificConfig().positionSolver.minPositionChange;
                    
                    // Set impulse solver configuration
                    params.impulseSolver.maxForce = getSpecificConfig().impulseSolver.maxForce;
                    params.impulseSolver.maxTorque = getSpecificConfig().impulseSolver.maxTorque;
                    params.impulseSolver.fluidForceScale = getSpecificConfig().impulseSolver.fluidForceScale;
                    params.impulseSolver.fluidForceMax = getSpecificConfig().impulseSolver.fluidForceMax;
                    params.impulseSolver.buoyancyStrength = getSpecificConfig().impulseSolver.buoyancyStrength;
                    params.impulseSolver.viscosityScale = getSpecificConfig().impulseSolver.viscosityScale;
                    params.impulseSolver.depthScale = getSpecificConfig().impulseSolver.depthScale;
                    params.impulseSolver.depthTransitionRate = getSpecificConfig().impulseSolver.depthTransitionRate;
                    params.impulseSolver.depthEstimateScale = getSpecificConfig().impulseSolver.depthEstimateScale;
                    params.impulseSolver.pressureForceRatio = getSpecificConfig().impulseSolver.pressureForceRatio;
                    params.impulseSolver.viscousForceRatio = getSpecificConfig().impulseSolver.viscousForceRatio;
                    params.impulseSolver.angularDampingThreshold = getSpecificConfig().impulseSolver.angularDampingThreshold;
                    params.impulseSolver.angularDampingFactor = getSpecificConfig().impulseSolver.angularDampingFactor;
                    params.impulseSolver.maxSafeVelocitySq = getSpecificConfig().impulseSolver.maxSafeVelocitySq;
                    params.impulseSolver.minPenetration = getSpecificConfig().impulseSolver.minPenetration;
                    params.impulseSolver.minRelVelocity = getSpecificConfig().impulseSolver.minRelVelocity;
                    
                    // Set grid configuration parameters
                    params.gridConfig.gridEpsilon = getSpecificConfig().gridConfig.gridEpsilon;
                    params.gridConfig.smoothingLength = getSpecificConfig().gridConfig.smoothingLength;
                    params.gridConfig.boundaryOffset = getSpecificConfig().gridConfig.boundaryOffset;
                    
                    // Set numerical configuration parameters
                    params.numericalConfig.minDistanceThreshold = getSpecificConfig().numericalConfig.minDistanceThreshold;
                    params.numericalConfig.minDensityThreshold = getSpecificConfig().numericalConfig.minDensityThreshold;
                    params.numericalConfig.minTimestep = getSpecificConfig().numericalConfig.minTimestep;
                    params.numericalConfig.fallbackTimestep = getSpecificConfig().numericalConfig.fallbackTimestep;
                    
                    // Update the parameter buffer
                    std::memcpy(paramsBuf_->contents(), &params, sizeof(params));

                    encodeComputePass(
                        enc,
                        verletHalfPSO_,
                        [&](MTL::ComputeCommandEncoder* innerEnc)
                        {
                            innerEnc->setBuffer(particleBuf_, 0, 0);
                            innerEnc->setBuffer(paramsBuf_,   0, 1);
                        },
                        realCount,
                        256
                    );

                    // computeBoundingBox
                    BBoxParams bboxParams{};
                    bboxParams.particleCount = realCount;
                    bboxParams.numThreadgroups = numThreadgroups;
                    std::memset(boundingBoxBuf_->contents(), 0, sizeof(float) * 4 * numThreadgroups);
                    std::memcpy(boundingBoxParamsBuf_->contents(), &bboxParams, sizeof(bboxParams));

                    encodeComputePass(
                        enc,
                        computeBoundingBoxPSO_,
                        [&](MTL::ComputeCommandEncoder* innerEnc)
                        {
                            innerEnc->setBuffer(particleBuf_,           0, 0);
                            innerEnc->setBuffer(boundingBoxParamsBuf_,  0, 1);
                            innerEnc->setBuffer(boundingBoxBuf_,        0, 2);

                            // Threadgroup memory for partial reductions
                            size_t localSharedBBox = sizeof(float) * 4 * 256;
                            size_t localValidCount = sizeof(int);

                            innerEnc->setThreadgroupMemoryLength(localSharedBBox, 3);
                            innerEnc->setThreadgroupMemoryLength(localValidCount, 4);
                        },
                        realCount,
                        256
                    );

                    enc->endEncoding();
                }
            }

            cmdBuf->commit();
            cmdBuf->waitUntilCompleted();
        }

        // --------------------------------------------------------------------
        // 2) CPU reduces partial bounding boxes
        float minX =  std::numeric_limits<float>::max();
        float maxX = -std::numeric_limits<float>::max();
        float minY =  std::numeric_limits<float>::max();
        float maxY = -std::numeric_limits<float>::max();
        reduceBoundingBoxOnCPU(boundingBoxBuf_, numThreadgroups, minX, maxX, minY, maxY);

        // 3) compute new grid size
        float maxH = 0.05f;
        const auto* pData = static_cast<const GPUFluidParticle*>(particleBuf_->contents());
        if (pData)
        {
            for (int i = 0; i < realCount; i++)
            {
                float hi = pData[i].h;
                if (hi > maxH)
                {
                    maxH = hi;
                }
            }
        }
        float cellSize = 2.f * maxH;

        // Slight padding
        minX -= 1e-6f;
        minY -= 1e-6f;

        int globalMinGx = static_cast<int>(std::floor(minX / cellSize));
        int globalMinGy = static_cast<int>(std::floor(minY / cellSize));
        int globalMaxGx = static_cast<int>(std::floor(maxX / cellSize));
        int globalMaxGy = static_cast<int>(std::floor(maxY / cellSize));

        int gridDimX = globalMaxGx - globalMinGx + 1;
        int gridDimY = globalMaxGy - globalMinGy + 1;
        if (gridDimX < 1) { gridDimX = 1; }
        if (gridDimY < 1) { gridDimY = 1; }
        int gridSize = gridDimX * gridDimY;

        // Re-init buffers if bounding box changed
        initBuffersIfNeeded(realCount, paddedCount, rigidCount, numThreadgroups, gridSize);

        // --------------------------------------------------------------------
        // 4) Fill GPUFluidParams with final neighbor search info
        GPUFluidParams params{};
        
        // Grid parameters
        params.cellSize    = cellSize;
        params.gridMinX    = globalMinGx;
        params.gridMinY    = globalMinGy;
        params.gridDimX    = gridDimX;
        params.gridDimY    = gridDimY;
        
        // Physics parameters
        params.gravity     = getSpecificConfig().gravity;
        params.restDensity = getSpecificConfig().restDensity;
        params.stiffness   = getSpecificConfig().stiffness;
        params.viscosity   = getSpecificConfig().viscosity;
        
        // Time parameters
        params.dt          = subDt;
        params.halfDt      = 0.5f * subDt;
        params.particleCount = static_cast<unsigned int>(paddedCount);
        
        // Set position solver configuration
        params.positionSolver.safetyMargin = getSpecificConfig().positionSolver.safetyMargin;
        params.positionSolver.relaxFactor = getSpecificConfig().positionSolver.relaxFactor;
        params.positionSolver.maxCorrection = getSpecificConfig().positionSolver.maxCorrection;
        params.positionSolver.maxVelocityUpdate = getSpecificConfig().positionSolver.maxVelocityUpdate;
        params.positionSolver.minSafeDistance = getSpecificConfig().positionSolver.minSafeDistance;
        params.positionSolver.velocityDamping = getSpecificConfig().positionSolver.velocityDamping;
        params.positionSolver.minPositionChange = getSpecificConfig().positionSolver.minPositionChange;
        
        // Set impulse solver configuration
        params.impulseSolver.maxForce = getSpecificConfig().impulseSolver.maxForce;
        params.impulseSolver.maxTorque = getSpecificConfig().impulseSolver.maxTorque;
        params.impulseSolver.fluidForceScale = getSpecificConfig().impulseSolver.fluidForceScale;
        params.impulseSolver.fluidForceMax = getSpecificConfig().impulseSolver.fluidForceMax;
        params.impulseSolver.buoyancyStrength = getSpecificConfig().impulseSolver.buoyancyStrength;
        params.impulseSolver.viscosityScale = getSpecificConfig().impulseSolver.viscosityScale;
        params.impulseSolver.depthScale = getSpecificConfig().impulseSolver.depthScale;
        params.impulseSolver.depthTransitionRate = getSpecificConfig().impulseSolver.depthTransitionRate;
        params.impulseSolver.depthEstimateScale = getSpecificConfig().impulseSolver.depthEstimateScale;
        params.impulseSolver.pressureForceRatio = getSpecificConfig().impulseSolver.pressureForceRatio;
        params.impulseSolver.viscousForceRatio = getSpecificConfig().impulseSolver.viscousForceRatio;
        params.impulseSolver.angularDampingThreshold = getSpecificConfig().impulseSolver.angularDampingThreshold;
        params.impulseSolver.angularDampingFactor = getSpecificConfig().impulseSolver.angularDampingFactor;
        params.impulseSolver.maxSafeVelocitySq = getSpecificConfig().impulseSolver.maxSafeVelocitySq;
        params.impulseSolver.minPenetration = getSpecificConfig().impulseSolver.minPenetration;
        params.impulseSolver.minRelVelocity = getSpecificConfig().impulseSolver.minRelVelocity;
        
        // Set grid configuration parameters
        params.gridConfig.gridEpsilon = getSpecificConfig().gridConfig.gridEpsilon;
        params.gridConfig.smoothingLength = getSpecificConfig().gridConfig.smoothingLength;
        params.gridConfig.boundaryOffset = getSpecificConfig().gridConfig.boundaryOffset;
        
        // Set numerical configuration parameters
        params.numericalConfig.minDistanceThreshold = getSpecificConfig().numericalConfig.minDistanceThreshold;
        params.numericalConfig.minDensityThreshold = getSpecificConfig().numericalConfig.minDensityThreshold;
        params.numericalConfig.minTimestep = getSpecificConfig().numericalConfig.minTimestep;
        params.numericalConfig.fallbackTimestep = getSpecificConfig().numericalConfig.fallbackTimestep;
        
        // Update the parameter buffer
        std::memcpy(paramsBuf_->contents(), &params, sizeof(params));

        // Clear the grid to zero
        if (gridBuf_->contents())
        {
            std::memset(gridBuf_->contents(), 0, sizeof(GPUGridCell) * gridSize);
        }

        // --------------------------------------------------------------------
        // 5) rest of sub-step: clearGrid, assignCells, computeDensity, etc.
        {
            MTL::CommandBuffer* cmdBuf = commandQueue_->commandBuffer();
            if (!cmdBuf)
            {
                return;
            }

            MTL::ComputeCommandEncoder* enc = cmdBuf->computeCommandEncoder();
            if (enc)
            {
                // clearGrid
                encodeComputePass(
                    enc,
                    clearGridPSO_,
                    [&](MTL::ComputeCommandEncoder* innerEnc)
                    {
                        innerEnc->setBuffer(gridBuf_, 0, 0);
                        innerEnc->setBytes(&gridSize, sizeof(int), 1);
                    },
                    static_cast<size_t>(gridSize),
                    256
                );

                // assignCells
                encodeComputePass(
                    enc,
                    assignCellsPSO_,
                    [&](MTL::ComputeCommandEncoder* innerEnc)
                    {
                        innerEnc->setBuffer(particleBuf_, 0, 0);
                        innerEnc->setBytes(&realCount, sizeof(int), 1);
                        innerEnc->setBuffer(gridBuf_, 0, 2);
                        innerEnc->setBuffer(paramsBuf_, 0, 3);
                    },
                    realCount,
                    256
                );

                // computeDensity
                encodeComputePass(
                    enc,
                    computeDensityPSO_,
                    [&](MTL::ComputeCommandEncoder* innerEnc)
                    {
                        innerEnc->setBuffer(particleBuf_, 0, 0);
                        innerEnc->setBytes(&realCount, sizeof(int), 1);
                        innerEnc->setBuffer(gridBuf_, 0, 2);
                        innerEnc->setBuffer(paramsBuf_, 0, 3);
                    },
                    realCount,
                    256
                );

                // computeForces
                encodeComputePass(
                    enc,
                    computeForcesPSO_,
                    [&](MTL::ComputeCommandEncoder* innerEnc)
                    {
                        innerEnc->setBuffer(particleBuf_, 0, 0);
                        innerEnc->setBytes(&realCount, sizeof(int), 1);
                        innerEnc->setBuffer(gridBuf_, 0, 2);
                        innerEnc->setBuffer(paramsBuf_, 0, 3);
                    },
                    realCount,
                    256
                );

                // velocityVerletFinish
                encodeComputePass(
                    enc,
                    verletFinishPSO_,
                    [&](MTL::ComputeCommandEncoder* innerEnc)
                    {
                        innerEnc->setBuffer(particleBuf_, 0, 0);
                        innerEnc->setBuffer(paramsBuf_, 0, 1);
                    },
                    realCount,
                    256
                );

                // rigidFluidImpulse (if we have rigid bodies)
                if (rigidCount > 0)
                {
                    encodeComputePass(
                        enc,
                        rigidFluidImpulsePSO_,
                        [&](MTL::ComputeCommandEncoder* innerEnc)
                        {
                            innerEnc->setBuffer(particleBuf_, 0, 0);
                            innerEnc->setBytes(&realCount, sizeof(int), 1);
                            innerEnc->setBuffer(rigidBuf_, 0, 2);
                            innerEnc->setBytes(&rigidCount, sizeof(int), 3);
                            innerEnc->setBuffer(paramsBuf_, 0, 4);
                        },
                        static_cast<size_t>(realCount),
                        256
                    );
                }

                // rigidFluidPosition
                encodeComputePass(
                    enc,
                    rigidFluidPositionPSO_,
                    [&](MTL::ComputeCommandEncoder* innerEnc)
                    {
                        innerEnc->setBuffer(particleBuf_, 0, 0);
                        innerEnc->setBytes(&realCount, sizeof(int), 1);
                        innerEnc->setBuffer(rigidBuf_, 0, 2);
                        innerEnc->setBytes(&rigidCount, sizeof(int), 3);
                        innerEnc->setBuffer(paramsBuf_, 0, 4);
                    },
                    static_cast<size_t>(realCount),
                    256
                );

                enc->endEncoding();
            }

            cmdBuf->commit();
            cmdBuf->waitUntilCompleted();
        }
    }

    // Update the ECS velocities for rigid bodies ONCE after ALL sub-steps
    if (rigidCount > 0) {
        writeBackRigidBodies(registry, rigidEntities, rigidBuf_, rigidCount);
    }
}

void FluidSystem::update(entt::registry& registry)
{
    PROFILE_SCOPE("FluidSystem::update (GPU-based SPH)");
    if (!device_ || !commandQueue_)
    {
        return;
    }

    // 1) Gather fluid
    std::vector<entt::entity> fluidEntityList;
    auto cpuParticles = gatherFluidParticles(registry, fluidEntityList);
    if (cpuParticles.empty())
    {
        return;
    }
    int fluidCount = static_cast<int>(cpuParticles.size());

    // 2) Gather rigid
    std::vector<entt::entity> rigidEntityList;
    auto cpuRigids = gatherRigidBodies(registry, rigidEntityList);
    int rigidCount = static_cast<int>(cpuRigids.size());

    // 3) Pad fluid count to next power of two
    int paddedCount = 1;
    while (paddedCount < fluidCount)
    {
        paddedCount <<= 1;
    }

    // 4) We'll guess gridSize=1 initially. It will grow after bounding box pass.
    int blockSize = getSpecificConfig().threadsPerGroup;
    int numThreadgroups = (fluidCount + blockSize - 1) / blockSize;
    int gridSize = 1;

    // 5) Init buffers with these guesses
    initBuffersIfNeeded(fluidCount, paddedCount, rigidCount, numThreadgroups, gridSize);

    // Copy fluid data to GPU
    if (particleBuf_->contents())
    {
        std::memset(particleBuf_->contents(), 0, sizeof(GPUFluidParticle) * paddedCount);
        std::memcpy(particleBuf_->contents(), cpuParticles.data(), sizeof(GPUFluidParticle) * fluidCount);
    }

    // Copy rigid data to GPU
    if (rigidCount > 0 && rigidBuf_->contents())
    {
        std::memset(rigidBuf_->contents(), 0, sizeof(GPURigidBody) * rigidCount);
        std::memcpy(rigidBuf_->contents(), cpuRigids.data(), sizeof(GPURigidBody) * rigidCount);
    }

    // 6) Do multi-step velocity-verlet
    multiStepVelocityVerlet(
        registry,
        fluidEntityList,
        rigidEntityList,
        fluidCount,
        paddedCount,
        rigidCount
    );

    // 7) Write fluid data back to ECS
    writeBackToECS(registry, fluidEntityList, particleBuf_, fluidCount);
}

}  // namespace Systems