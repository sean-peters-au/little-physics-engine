/**
 * @fileoverview fluid.cpp
 * @brief A GPU-based SPH fluid system that uses Metal kernels for neighbor search and force computations.
 *
 * This file gathers fluid particles from the ECS, builds a bounding box, then uses Metal to:
 * - Assign each particle to a grid cell.
 * - Compute density and pressure using neighbor searches.
 * - Compute forces (pressure + viscosity) and integrate via velocity Verlet.
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
 * @brief GPU-side fluid particle struct. Must match the layout in fluid_kernels.metal.
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
 * @brief GPU-side grid cell struct. Must match layout in fluid_kernels.metal.
 */
static constexpr int GPU_MAX_PER_CELL = 64;
struct GPUGridCell {
  int count;
  int indices[GPU_MAX_PER_CELL];
};

/**
 * @brief Fluid parameters passed to Metal kernels.
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
 * @brief A system that executes SPH fluid simulation using Metal kernels.
 */
class FluidSystem {
 public:
  FluidSystem();
  ~FluidSystem();

  /**
   * @brief Updates the fluid system by running multiple velocity-Verlet sub-steps on the GPU.
   * @param registry The ECS registry that holds fluid particles.
   */
  void update(entt::registry& registry);

 private:
  MTL::Device* device_ = nullptr;
  MTL::CommandQueue* commandQueue_ = nullptr;
  MTL::Library* metalLibrary_ = nullptr;

  MTL::ComputePipelineState* clearGridPSO_ = nullptr;
  MTL::ComputePipelineState* assignCellsPSO_ = nullptr;
  MTL::ComputePipelineState* computeDensityPSO_ = nullptr;
  MTL::ComputePipelineState* computeForcesPSO_ = nullptr;
  MTL::ComputePipelineState* verletHalfPSO_ = nullptr;
  MTL::ComputePipelineState* verletFinishPSO_ = nullptr;

  static constexpr int N_SUB_STEPS = 4;

  /**
   * @brief Helper to create a compute pipeline state object from a named function.
   * @param fnName Name of the Metal kernel function.
   * @param lib Pointer to the loaded Metal library.
   * @return A pointer to the created pipeline state or nullptr on failure.
   */
  MTL::ComputePipelineState* createPSO(const char* fnName, MTL::Library* lib);
};

FluidSystem::FluidSystem() {
  device_ = MTL::CreateSystemDefaultDevice();
  if (!device_) {
    std::cerr << "No Metal device found. This requires macOS with Metal support." << std::endl;
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

  clearGridPSO_ = createPSO("clearGrid", metalLibrary_);
  if (!clearGridPSO_) {
    std::cerr << "Failed to create clearGrid pipeline state." << std::endl;
    return;
  }

  assignCellsPSO_ = createPSO("assignCells", metalLibrary_);
  if (!assignCellsPSO_) {
    std::cerr << "Failed to create assignCells pipeline state." << std::endl;
    return;
  }

  computeDensityPSO_ = createPSO("computeDensity", metalLibrary_);
  if (!computeDensityPSO_) {
    std::cerr << "Failed to create computeDensity pipeline state." << std::endl;
    return;
  }

  computeForcesPSO_ = createPSO("computeForces", metalLibrary_);
  if (!computeForcesPSO_) {
    std::cerr << "Failed to create computeForces pipeline state." << std::endl;
    return;
  }

  verletHalfPSO_ = createPSO("velocityVerletHalf", metalLibrary_);
  if (!verletHalfPSO_) {
    std::cerr << "Failed to create velocityVerletHalf pipeline state." << std::endl;
    return;
  }

  verletFinishPSO_ = createPSO("velocityVerletFinish", metalLibrary_);
  if (!verletFinishPSO_) {
    std::cerr << "Failed to create velocityVerletFinish pipeline state." << std::endl;
    return;
  }
}

FluidSystem::~FluidSystem() {
  if (clearGridPSO_) {
    clearGridPSO_->release();
  }
  if (assignCellsPSO_) {
    assignCellsPSO_->release();
  }
  if (computeDensityPSO_) {
    computeDensityPSO_->release();
  }
  if (computeForcesPSO_) {
    computeForcesPSO_->release();
  }
  if (verletHalfPSO_) {
    verletHalfPSO_->release();
  }
  if (verletFinishPSO_) {
    verletFinishPSO_->release();
  }
  if (commandQueue_) {
    commandQueue_->release();
  }
}

MTL::ComputePipelineState* FluidSystem::createPSO(const char* fnName, MTL::Library* lib) {
  NS::String* name = NS::String::string(fnName, NS::UTF8StringEncoding);
  MTL::Function* fn = lib->newFunction(name);
  if (!fn) {
    std::cerr << "Missing function " << fnName << " in metal library." << std::endl;
    return nullptr;
  }
  NS::Error* error = nullptr;
  MTL::ComputePipelineState* pso = device_->newComputePipelineState(fn, &error);
  fn->release();
  if (!pso) {
    std::cerr << "Failed to create pipeline for " << fnName << ": "
              << (error ? error->localizedDescription()->utf8String() : "Unknown error") << std::endl;
  }
  return pso;
}

void FluidSystem::update(entt::registry& registry) {
  PROFILE_SCOPE("FluidSystem::update (Metal GPU)");

  if (!device_ || !commandQueue_) {
    std::cout << "No device or command queue found." << std::endl;
    return;
  }

  std::vector<GPUFluidParticle> cpuParticles;
  cpuParticles.reserve(registry.storage<Components::Position>().size());
  std::vector<entt::entity> entityList;

  float dt = static_cast<float>(
      SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration);

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

    auto& pos = view.get<Components::Position>(e);
    auto& vel = view.get<Components::Velocity>(e);
    auto& mass = view.get<Components::Mass>(e);
    auto& smoothLen = view.get<Components::SmoothingLength>(e);
    auto& soundSpeed = view.get<Components::SpeedOfSound>(e);

    GPUFluidParticle p{};
    p.x = pos.x;
    p.y = pos.y;
    p.vx = vel.x;
    p.vy = vel.y;
    p.vxHalf = p.vx;
    p.vyHalf = p.vy;
    p.ax = 0.0f;
    p.ay = 0.0f;
    p.mass = mass.value;
    p.h = smoothLen.value;
    p.c = soundSpeed.value;
    p.density = 0.0f;
    p.pressure = 0.0f;

    cpuParticles.push_back(p);
    entityList.push_back(e);
  }

  if (cpuParticles.empty()) {
    return;
  }

  // Pad up to next power of two for GPU batch convenience
  int realCount = static_cast<int>(cpuParticles.size());
  int particleCount = 1;
  while (particleCount < realCount) {
    particleCount <<= 1;
  }

  // Compute bounding box for the particles
  float minX = std::numeric_limits<float>::max();
  float minY = std::numeric_limits<float>::max();
  float maxX = std::numeric_limits<float>::lowest();
  float maxY = std::numeric_limits<float>::lowest();
  float referenceH = cpuParticles[0].h;

  for (auto& p : cpuParticles) {
    if (p.x < minX) {
      minX = p.x;
    }
    if (p.x > maxX) {
      maxX = p.x;
    }
    if (p.y < minY) {
      minY = p.y;
    }
    if (p.y > maxY) {
      maxY = p.y;
    }
  }

  float h = (referenceH <= 0.0f ? 0.05f : referenceH);
  float padding = h;  // add a bit of padding around bounding box
  minX -= padding;
  minY -= padding;
  maxX += padding;
  maxY += padding;

  float domainSizeX = maxX - minX;
  float domainSizeY = maxY - minY;

  GPUFluidParams params{};
  params.cellSize = 2.0f * h;
  params.gridMinX = static_cast<int>(std::floor(minX));
  params.gridMinY = static_cast<int>(std::floor(minY));
  params.gridDimX = static_cast<int>(std::ceil(domainSizeX / params.cellSize));
  params.gridDimY = static_cast<int>(std::ceil(domainSizeY / params.cellSize));

  params.restDensity = static_cast<float>(SimulatorConstants::ParticleDensity);
  params.stiffness = 4000.0f;
  params.viscosity = 0.1f;

  float subDt = dt / static_cast<float>(N_SUB_STEPS);
  params.dt = subDt;
  params.halfDt = 0.5f * subDt;
  params.particleCount = particleCount;

  auto particleBuf = device_->newBuffer(sizeof(GPUFluidParticle) * particleCount,
                                        MTL::ResourceStorageModeShared);
  memset(particleBuf->contents(), 0, sizeof(GPUFluidParticle) * particleCount);
  memcpy(particleBuf->contents(), cpuParticles.data(), sizeof(GPUFluidParticle) * realCount);

  int gridSize = params.gridDimX * params.gridDimY;
  auto gridBuf = device_->newBuffer(sizeof(GPUGridCell) * gridSize,
                                    MTL::ResourceStorageModeShared);

  auto paramsBuf = device_->newBuffer(sizeof(GPUFluidParams),
                                      MTL::ResourceStorageModeShared);
  memcpy(paramsBuf->contents(), &params, sizeof(GPUFluidParams));

  for (int step = 0; step < N_SUB_STEPS; ++step) {
    {
      auto cmdBuf = commandQueue_->commandBuffer();
      auto enc = cmdBuf->computeCommandEncoder();
      enc->setComputePipelineState(verletHalfPSO_);
      enc->setBuffer(particleBuf, 0, 0);
      enc->setBuffer(paramsBuf, 0, 1);

      MTL::Size threadsPerGrid(particleCount, 1, 1);
      MTL::Size threadsPerGroup(64, 1, 1);
      enc->dispatchThreadgroups(threadsPerGrid, threadsPerGroup);
      enc->endEncoding();
      cmdBuf->commit();
      cmdBuf->waitUntilCompleted();
    }
    {
      auto cmdBuf = commandQueue_->commandBuffer();
      auto enc = cmdBuf->computeCommandEncoder();
      enc->setComputePipelineState(clearGridPSO_);
      enc->setBuffer(gridBuf, 0, 0);
      enc->setBytes(&gridSize, sizeof(int), 1);

      MTL::Size threads(gridSize, 1, 1);
      MTL::Size tgroup(64, 1, 1);
      enc->dispatchThreadgroups(threads, tgroup);
      enc->endEncoding();
      cmdBuf->commit();
      cmdBuf->waitUntilCompleted();
    }
    {
      auto cmdBuf = commandQueue_->commandBuffer();
      auto enc = cmdBuf->computeCommandEncoder();
      enc->setComputePipelineState(assignCellsPSO_);
      enc->setBuffer(particleBuf, 0, 0);
      enc->setBytes(&particleCount, sizeof(int), 1);
      enc->setBuffer(gridBuf, 0, 2);
      enc->setBuffer(paramsBuf, 0, 3);

      MTL::Size threads(particleCount, 1, 1);
      MTL::Size tgroup(64, 1, 1);
      enc->dispatchThreadgroups(threads, tgroup);
      enc->endEncoding();
      cmdBuf->commit();
      cmdBuf->waitUntilCompleted();
    }
    {
      auto cmdBuf = commandQueue_->commandBuffer();
      auto enc = cmdBuf->computeCommandEncoder();
      enc->setComputePipelineState(computeDensityPSO_);
      enc->setBuffer(particleBuf, 0, 0);
      enc->setBytes(&particleCount, sizeof(int), 1);
      enc->setBuffer(gridBuf, 0, 2);
      enc->setBuffer(paramsBuf, 0, 3);

      MTL::Size threads(particleCount, 1, 1);
      MTL::Size tgroup(64, 1, 1);
      enc->dispatchThreadgroups(threads, tgroup);
      enc->endEncoding();
      cmdBuf->commit();
      cmdBuf->waitUntilCompleted();
    }
    {
      auto cmdBuf = commandQueue_->commandBuffer();
      auto enc = cmdBuf->computeCommandEncoder();
      enc->setComputePipelineState(computeForcesPSO_);
      enc->setBuffer(particleBuf, 0, 0);
      enc->setBytes(&particleCount, sizeof(int), 1);
      enc->setBuffer(gridBuf, 0, 2);
      enc->setBuffer(paramsBuf, 0, 3);

      MTL::Size threads(particleCount, 1, 1);
      MTL::Size tgroup(64, 1, 1);
      enc->dispatchThreadgroups(threads, tgroup);
      enc->endEncoding();
      cmdBuf->commit();
      cmdBuf->waitUntilCompleted();
    }
    {
      auto cmdBuf = commandQueue_->commandBuffer();
      auto enc = cmdBuf->computeCommandEncoder();
      enc->setComputePipelineState(verletFinishPSO_);
      enc->setBuffer(particleBuf, 0, 0);
      enc->setBuffer(paramsBuf, 0, 1);

      MTL::Size threadsPerGrid(particleCount, 1, 1);
      MTL::Size threadsPerGroup(64, 1, 1);
      enc->dispatchThreadgroups(threadsPerGrid, threadsPerGroup);
      enc->endEncoding();
      cmdBuf->commit();
      cmdBuf->waitUntilCompleted();
    }
  }

  memcpy(cpuParticles.data(), particleBuf->contents(), sizeof(GPUFluidParticle) * realCount);

  for (size_t i = 0; i < entityList.size(); i++) {
    entt::entity e = entityList[i];
    auto& pos = registry.get<Components::Position>(e);
    auto& vel = registry.get<Components::Velocity>(e);
    auto& sphtemp = registry.get<Components::SPHTemp>(e);

    GPUFluidParticle& p = cpuParticles[i];
    pos.x = p.x;
    pos.y = p.y;
    vel.x = p.vx;
    vel.y = p.vy;
    sphtemp.density = p.density;
    sphtemp.pressure = p.pressure;
  }

  particleBuf->release();
  gridBuf->release();
  paramsBuf->release();
}

}  // namespace Systems
