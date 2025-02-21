/**
 * @fileoverview fluid_kernels.metal
 * @brief Metal compute kernels for 2D SPH fluid simulation with added gravity.
 *
 * - Clears and fills a grid for neighbor search.
 * - Computes density and pressure via 2D poly6 kernel.
 * - Computes forces (pressure + viscosity + gravity).
 * - Integrates via velocity Verlet.
 */

#include <metal_stdlib>
#include <metal_atomic>

using namespace metal;

#define M_PI 3.14159265358979323846

// Gravity constant for 2D (adjust as desired)
constant float GRAVITY = -9.81f;

/**
 * @brief Matches the C++ struct GPUFluidParticle.
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
 * @brief Matches the C++ struct GPUGridCell.
 */
constant int GPU_MAX_PER_CELL = 64;
struct GPUGridCell {
  atomic_int count;
  int indices[GPU_MAX_PER_CELL];
};

/**
 * @brief Matches the C++ struct GPUFluidParams.
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
 * @brief Clears the grid cell counters to zero.
 */
kernel void clearGrid(device GPUGridCell* grid [[buffer(0)]],
                      constant int& cellCount [[buffer(1)]],
                      uint gid [[thread_position_in_grid]]) {
  if (gid < cellCount) {
    atomic_store_explicit(&grid[gid].count, 0, memory_order_relaxed);
  }
}

/**
 * @brief Assigns each particle to a grid cell based on its position.
 */
kernel void assignCells(device GPUFluidParticle* particles [[buffer(0)]],
                        constant int& particleCount [[buffer(1)]],
                        device GPUGridCell* grid [[buffer(2)]],
                        device const GPUFluidParams& params [[buffer(3)]],
                        uint gid [[thread_position_in_grid]]) {
  if (gid >= (uint)particleCount) {
    return;
  }
  GPUFluidParticle p = particles[gid];

  int cellX = int(floor((p.x - float(params.gridMinX)) / params.cellSize));
  int cellY = int(floor((p.y - float(params.gridMinY)) / params.cellSize));

  if (cellX < 0 || cellX >= params.gridDimX ||
      cellY < 0 || cellY >= params.gridDimY) {
    return;
  }

  int cellIndex = cellY * params.gridDimX + cellX;
  int oldCount = atomic_fetch_add_explicit(&grid[cellIndex].count, 1, memory_order_relaxed);

  if (oldCount < GPU_MAX_PER_CELL) {
    grid[cellIndex].indices[oldCount] = int(gid);
  }
}

/**
 * @brief Computes density and pressure for each particle using 2D SPH poly6 kernel.
 *
 * 2D poly6: W(r) = (4/(πh^8)) * (h^2 - r^2)^3, for r < h.
 */
kernel void computeDensity(device GPUFluidParticle* particles [[buffer(0)]],
                           constant int& particleCount [[buffer(1)]],
                           device const GPUGridCell* grid [[buffer(2)]],
                           device const GPUFluidParams& params [[buffer(3)]],
                           uint gid [[thread_position_in_grid]]) {
  if (gid >= (uint)particleCount) {
    return;
  }

  GPUFluidParticle self = particles[gid];
  float density = 0.0f;

  float h2 = self.h * self.h;
  float poly6Const = 4.0f / (M_PI * pow(self.h, 8.0f));  // 2D poly6 factor

  int cellX = int(floor((self.x - float(params.gridMinX)) / params.cellSize));
  int cellY = int(floor((self.y - float(params.gridMinY)) / params.cellSize));

  for (int ny = -1; ny <= 1; ny++) {
    for (int nx = -1; nx <= 1; nx++) {
      int cx = cellX + nx;
      int cy = cellY + ny;
      if (cx < 0 || cx >= params.gridDimX ||
          cy < 0 || cy >= params.gridDimY) {
        continue;
      }
      int cellIndex = cy * params.gridDimX + cx;
      int count = atomic_load_explicit(&grid[cellIndex].count, memory_order_relaxed);
      for (int i = 0; i < count; i++) {
        int neighborIndex = grid[cellIndex].indices[i];
        if (neighborIndex >= particleCount) {
          continue;
        }
        GPUFluidParticle neighbor = particles[neighborIndex];
        float dx = self.x - neighbor.x;
        float dy = self.y - neighbor.y;
        float r2 = dx * dx + dy * dy;

        if (r2 < h2) {
          float diff = (h2 - r2);
          float w = poly6Const * diff * diff * diff;
          density += neighbor.mass * w;
        }
      }
    }
  }

  // Pressure with Tait equation or linear approximation
  self.density = max(density, 0.0001f);
  self.pressure = params.stiffness * (self.density - params.restDensity);
  particles[gid] = self;
}

/**
 * @brief Computes pressure, viscosity, and gravity forces for each particle (2D spiky + gravity).
 */
kernel void computeForces(device GPUFluidParticle* particles [[buffer(0)]],
                          constant int& particleCount [[buffer(1)]],
                          device const GPUGridCell* grid [[buffer(2)]],
                          device const GPUFluidParams& params [[buffer(3)]],
                          uint gid [[thread_position_in_grid]]) {
  if (gid >= (uint)particleCount) {
    return;
  }

  GPUFluidParticle self = particles[gid];
  float fx = 0.0f;
  float fy = 0.0f;

  // Add gravity
  // (Here we do mass * gravity so that total force is f=ma => a=gravity.)
  fy -= self.mass * GRAVITY;

  // 2D spiky kernel constants for pressure force
  // W_spiky_grad =  - (r/h) * (something). We'll approximate the magnitude factor:
  //   Kspiky = 15 / (π * h^5) for 2D
  // Typical or reference:  (Note: some references use different forms.)
  float Kspiky = 15.0f / (M_PI * pow(self.h, 5.0f));

  // 2D viscosity kernel:
  // W_visc(r) =  (10 / (π * h^5)) * (h - r)
  float Kvisc = 10.0f / (M_PI * pow(self.h, 5.0f));

  int cellX = int(floor((self.x - float(params.gridMinX)) / params.cellSize));
  int cellY = int(floor((self.y - float(params.gridMinY)) / params.cellSize));

  for (int ny = -1; ny <= 1; ny++) {
    for (int nx = -1; nx <= 1; nx++) {
      int cx = cellX + nx;
      int cy = cellY + ny;
      if (cx < 0 || cx >= params.gridDimX ||
          cy < 0 || cy >= params.gridDimY) {
        continue;
      }
      int cellIndex = cy * params.gridDimX + cx;
      int count = atomic_load_explicit(&grid[cellIndex].count, memory_order_relaxed);
      for (int i = 0; i < count; i++) {
        int neighborIndex = grid[cellIndex].indices[i];
        if (neighborIndex >= particleCount || neighborIndex == int(gid)) {
          continue;
        }
        GPUFluidParticle neighbor = particles[neighborIndex];

        float dx = self.x - neighbor.x;
        float dy = self.y - neighbor.y;
        float r2 = dx * dx + dy * dy;
        float r = sqrt(r2);
        float h = self.h;

        if (r > 0.0f && r < h) {
          // Pressure force
          float avgPressure = (self.pressure + neighbor.pressure) * 0.5f;
          float term = (h - r);
          float wSpiky = Kspiky * term * term * term;  // spiky kernel
          float fPres = -neighbor.mass * (avgPressure / neighbor.density) * wSpiky;

          fx += fPres * (dx / r);
          fy += fPres * (dy / r);

          // Viscosity force
          float diffVx = neighbor.vx - self.vx;
          float diffVy = neighbor.vy - self.vy;
          float wVisc = Kvisc * (h - r);
          float fVisc = params.viscosity * neighbor.mass * (1.0f / neighbor.density);

          fx += fVisc * diffVx * wVisc;
          fy += fVisc * diffVy * wVisc;
        }
      }
    }
  }

  self.ax = fx;
  self.ay = fy;
  particles[gid] = self;
}

/**
 * @brief Velocity Verlet half-step: update velocity by half dt, then position by full dt.
 */
kernel void velocityVerletHalf(device GPUFluidParticle* particles [[buffer(0)]],
                               device const GPUFluidParams& params [[buffer(1)]],
                               uint gid [[thread_position_in_grid]]) {
  if (gid >= params.particleCount) {
    return;
  }

  GPUFluidParticle p = particles[gid];
  float halfDt = params.halfDt;
  p.vxHalf = p.vx + halfDt * p.ax;
  p.vyHalf = p.vy + halfDt * p.ay;
  p.x += p.vxHalf * params.dt;
  p.y += p.vyHalf * params.dt;
  particles[gid] = p;
}

/**
 * @brief Velocity Verlet finish: finalize velocity using new acceleration from this step.
 */
kernel void velocityVerletFinish(device GPUFluidParticle* particles [[buffer(0)]],
                                 device const GPUFluidParams& params [[buffer(1)]],
                                 uint gid [[thread_position_in_grid]]) {
  if (gid >= params.particleCount) {
    return;
  }

  GPUFluidParticle p = particles[gid];
  float halfDt = params.halfDt;
  p.vx = p.vxHalf + halfDt * p.ax;
  p.vy = p.vyHalf + halfDt * p.ay;
  particles[gid] = p;
}
