/**
 * @fileoverview fluid.cpp
 * @brief A GPU-based 2D SPH fluid system using Metal, now mirroring the old CPU code more exactly.
 *
 * Main changes:
 * - N_SUB_STEPS = 10 (like the CPU).
 * - Recompute the bounding box *every sub-step* after velocityVerletHalf.
 * - Use 2D poly6/spiky. Clamp negative pressure. No gravity. h_ij = 0.5*(h_i + h_j).
 */

#include <cmath>
#include <vector>
#include <iostream>
#include <limits>

#include <Metal/Metal.hpp>
#include <Foundation/Foundation.hpp>
#include <entt/entt.hpp>

#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sph.hpp"
#include "nbody/core/profile.hpp"

namespace Systems {

/**
 * Must match fluid_kernels.metal
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
 * Must match fluid_kernels.metal
 */
static constexpr int GPU_MAX_PER_CELL = 64;
struct GPUGridCell {
  int count;
  int indices[GPU_MAX_PER_CELL];
};

/**
 * Must match fluid_kernels.metal
 */
struct GPUFluidParams {
  float cellSize;
  int gridDimX;
  int gridDimY;
  int gridMinX;
  int gridMinY;
  float restDensity;
  float stiffness;
  float viscosity;
  float dt;
  float halfDt;
  uint particleCount;
};

/**
 * @class FluidSystem
 * @brief Does multi-sub-step SPH, replicating the CPU logic as closely as possible.
 */
class FluidSystem {
 public:
  FluidSystem();
  ~FluidSystem();

  void update(entt::registry& registry);

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

  // CPU code used N_SUB_STEPS=10 by default
  static constexpr int N_SUB_STEPS = 25;

  MTL::ComputePipelineState* createPSO(const char* name, MTL::Library* lib);
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

  clearGridPSO_      = createPSO("clearGrid",           metalLibrary_);
  assignCellsPSO_    = createPSO("assignCells",         metalLibrary_);
  computeDensityPSO_ = createPSO("computeDensity",      metalLibrary_);
  computeForcesPSO_  = createPSO("computeForces",       metalLibrary_);
  verletHalfPSO_     = createPSO("velocityVerletHalf",  metalLibrary_);
  verletFinishPSO_   = createPSO("velocityVerletFinish",metalLibrary_);
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

MTL::ComputePipelineState* FluidSystem::createPSO(const char* name, MTL::Library* lib) {
  NS::String* fnName = NS::String::string(name, NS::UTF8StringEncoding);
  MTL::Function* fn = lib->newFunction(fnName);
  if (!fn) {
    std::cerr << "Missing function " << name << " in metal library." << std::endl;
    return nullptr;
  }
  NS::Error* error = nullptr;
  auto pso = device_->newComputePipelineState(fn, &error);
  fn->release();
  if (!pso) {
    std::cerr << "Failed to create pipeline: " << name << " => "
              << (error ? error->localizedDescription()->utf8String() : "?") << std::endl;
  }
  return pso;
}

void FluidSystem::update(entt::registry& registry) {
  PROFILE_SCOPE("FluidSystem::update (GPU-based SPH)");

  if (!device_ || !commandQueue_) {
    return;
  }

  // 1) Gather fluid particles from ECS
  std::vector<GPUFluidParticle> cpuParticles;
  cpuParticles.reserve(registry.storage<Components::Position>().size());
  std::vector<entt::entity> entityList;

  float dt = float(SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration);

  auto view = registry.view<Components::Position,
                            Components::Velocity,
                            Components::Mass,
                            Components::ParticlePhase,
                            Components::SmoothingLength,
                            Components::SpeedOfSound,
                            Components::SPHTemp>();

  for (auto e : view) {
    auto& phase = view.get<Components::ParticlePhase>(e);
    if (phase.phase != Components::Phase::Liquid) {
      continue;
    }
    auto& pos   = view.get<Components::Position>(e);
    auto& vel   = view.get<Components::Velocity>(e);
    auto& mass  = view.get<Components::Mass>(e);
    auto& sl    = view.get<Components::SmoothingLength>(e);
    auto& snd   = view.get<Components::SpeedOfSound>(e);

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
    p.h    = (sl.value <= 0.f ? 0.05f : sl.value);
    p.c    = snd.value;
    p.density = 0.f;
    p.pressure= 0.f;

    cpuParticles.push_back(p);
    entityList.push_back(e);
  }

  if (cpuParticles.empty()) {
    return;
  }

  // 2) Like CPU code: N_SUB_STEPS=10
  float subDt = dt / float(N_SUB_STEPS);

  // We'll replicate the CPU's approach to bounding box & grid each sub-step

  // For convenience, we pad to power-of-two, but we only process realCount in kernels
  int realCount = (int)cpuParticles.size();
  int paddedCount = 1;
  while (paddedCount < realCount) {
    paddedCount <<= 1;
  }

  // Create GPU buffers once, but re-do bounding box for each sub-step
  auto particleBuf = device_->newBuffer(sizeof(GPUFluidParticle)*paddedCount,
                                        MTL::ResourceStorageModeShared);
  memset(particleBuf->contents(), 0, sizeof(GPUFluidParticle)*paddedCount);
  memcpy(particleBuf->contents(), cpuParticles.data(), sizeof(GPUFluidParticle)*realCount);

  auto paramsBuf = device_->newBuffer(sizeof(GPUFluidParams),
                                      MTL::ResourceStorageModeShared);

  for (int step = 0; step < N_SUB_STEPS; step++) {
    // (A) velocityVerletHalf
    {
      GPUFluidParams params{};
      params.restDensity = 100.f; // float(SimulatorConstants::ParticleDensity); // e.g. 1000
      params.stiffness   = 100.f;  // CPU code used 500
      params.viscosity   = 0.1f;
      params.dt      = subDt;
      params.halfDt  = 0.5f*subDt;
      params.particleCount = paddedCount;

      // Copy updated accelerations into GPU buffer before half-step.
      memcpy(particleBuf->contents(), cpuParticles.data(), sizeof(GPUFluidParticle)*realCount);
      memcpy(paramsBuf->contents(), &params, sizeof(GPUFluidParams));

      auto cmdBuf = commandQueue_->commandBuffer();
      auto enc = cmdBuf->computeCommandEncoder();
      enc->setComputePipelineState(verletHalfPSO_);
      enc->setBuffer(particleBuf, 0, 0);
      enc->setBuffer(paramsBuf, 0, 1);

      MTL::Size threads(paddedCount,1,1);
      MTL::Size tgroup(64,1,1);
      enc->dispatchThreadgroups(threads, tgroup);
      enc->endEncoding();
      cmdBuf->commit();
      cmdBuf->waitUntilCompleted();

      // Read back to CPU so we can rebuild bounding box with new positions
      memcpy(cpuParticles.data(), particleBuf->contents(), sizeof(GPUFluidParticle)*realCount);
    }

    // (B) Rebuild bounding box from updated positions (like CPU does every sub-step)
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();

    for (auto& p : cpuParticles) {
      if (p.x < minX) minX = p.x;
      if (p.x > maxX) maxX = p.x;
      if (p.y < minY) minY = p.y;
      if (p.y > maxY) maxY = p.y;
    }
    float pad = cpuParticles[0].h; // same approach as CPU
    minX -= pad; maxX += pad;
    minY -= pad; maxY += pad;

    float domainSizeX = maxX - minX;
    float domainSizeY = maxY - minY;
    float cellSize    = 2.f * cpuParticles[0].h;

    // (C) Now set GPUFluidParams for assignCells/density/forces
    GPUFluidParams params{};
    params.cellSize = cellSize;
    params.gridMinX = int(floor(minX));
    params.gridMinY = int(floor(minY));
    params.gridDimX = int(ceil(domainSizeX / cellSize));
    params.gridDimY = int(ceil(domainSizeY / cellSize));
    params.restDensity = float(SimulatorConstants::ParticleDensity);
    params.stiffness   = 500.f;
    params.viscosity   = 0.1f;
    params.dt      = subDt;
    params.halfDt  = 0.5f*subDt;
    params.particleCount = paddedCount;

    memcpy(paramsBuf->contents(), &params, sizeof(GPUFluidParams));

    int gridSize = params.gridDimX * params.gridDimY;
    auto gridBuf = device_->newBuffer(sizeof(GPUGridCell)*gridSize,
                                      MTL::ResourceStorageModeShared);
    memset(gridBuf->contents(), 0, sizeof(GPUGridCell)*gridSize);

    // (D) clearGrid
    {
      auto cmdBuf = commandQueue_->commandBuffer();
      auto enc = cmdBuf->computeCommandEncoder();
      enc->setComputePipelineState(clearGridPSO_);
      enc->setBuffer(gridBuf, 0, 0);
      enc->setBytes(&gridSize, sizeof(int), 1);

      MTL::Size threads(gridSize,1,1);
      MTL::Size tgroup(64,1,1);
      enc->dispatchThreadgroups(threads, tgroup);
      enc->endEncoding();
      cmdBuf->commit();
      cmdBuf->waitUntilCompleted();
    }
    // (E) assignCells
    {
      auto cmdBuf = commandQueue_->commandBuffer();
      auto enc = cmdBuf->computeCommandEncoder();
      enc->setComputePipelineState(assignCellsPSO_);
      enc->setBuffer(particleBuf, 0, 0);
      enc->setBytes(&paddedCount, sizeof(int), 1);
      enc->setBuffer(gridBuf, 0, 2);
      enc->setBuffer(paramsBuf, 0, 3);

      MTL::Size threads(paddedCount,1,1);
      MTL::Size tgroup(64,1,1);
      enc->dispatchThreadgroups(threads, tgroup);
      enc->endEncoding();
      cmdBuf->commit();
      cmdBuf->waitUntilCompleted();
    }
    // (F) computeDensity
    {
      auto cmdBuf = commandQueue_->commandBuffer();
      auto enc = cmdBuf->computeCommandEncoder();
      enc->setComputePipelineState(computeDensityPSO_);
      enc->setBuffer(particleBuf, 0, 0);
      enc->setBytes(&paddedCount, sizeof(int), 1);
      enc->setBuffer(gridBuf, 0, 2);
      enc->setBuffer(paramsBuf, 0, 3);

      MTL::Size threads(paddedCount,1,1);
      MTL::Size tgroup(64,1,1);
      enc->dispatchThreadgroups(threads, tgroup);
      enc->endEncoding();
      cmdBuf->commit();
      cmdBuf->waitUntilCompleted();
    }
    // (G) computeForces
    {
      auto cmdBuf = commandQueue_->commandBuffer();
      auto enc = cmdBuf->computeCommandEncoder();
      enc->setComputePipelineState(computeForcesPSO_);
      enc->setBuffer(particleBuf, 0, 0);
      enc->setBytes(&paddedCount, sizeof(int), 1);
      enc->setBuffer(gridBuf, 0, 2);
      enc->setBuffer(paramsBuf, 0, 3);

      MTL::Size threads(paddedCount,1,1);
      MTL::Size tgroup(64,1,1);
      enc->dispatchThreadgroups(threads, tgroup);
      enc->endEncoding();
      cmdBuf->commit();
      cmdBuf->waitUntilCompleted();
    }

    // We no longer need gridBuf for velocityVerletFinish
    gridBuf->release();

    // (H) velocityVerletFinish
    {
      auto cmdBuf = commandQueue_->commandBuffer();
      auto enc = cmdBuf->computeCommandEncoder();
      enc->setComputePipelineState(verletFinishPSO_);
      enc->setBuffer(particleBuf, 0, 0);
      enc->setBuffer(paramsBuf, 0, 1);

      MTL::Size threads(paddedCount,1,1);
      MTL::Size tgroup(64,1,1);
      enc->dispatchThreadgroups(threads, tgroup);
      enc->endEncoding();
      cmdBuf->commit();
      cmdBuf->waitUntilCompleted();
    }

    // read back to CPU so next sub-step half-step sees new v
    memcpy(cpuParticles.data(), particleBuf->contents(), sizeof(GPUFluidParticle)*realCount);
  }

  // 3) Write final results to ECS
  for (size_t i = 0; i < entityList.size(); i++) {
    entt::entity e = entityList[i];
    auto& pos = registry.get<Components::Position>(e);
    auto& vel = registry.get<Components::Velocity>(e);
    auto& spht = registry.get<Components::SPHTemp>(e);

    GPUFluidParticle& p = cpuParticles[i];
    pos.x = p.x;
    pos.y = p.y;
    vel.x = p.vx;
    vel.y = p.vy;
    spht.density  = p.density;
    spht.pressure = p.pressure;
  }

  particleBuf->release();
  paramsBuf->release();
}

} // namespace Systems
