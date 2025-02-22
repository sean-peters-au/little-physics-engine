/**
 * @fileoverview fluid_kernels.metal
 * @brief Metal compute kernels for 2D SPH, closely matching the old CPU code's logic.
 *
 * Main differences from your prior GPU version:
 * - Uses 2D poly6/spiky constants
 * - Clamps negative pressures to zero
 * - No gravity
 * - Uses h_ij = 0.5 * (h_i + h_j)
 * - Skips neighbors if r2 >= h_ij^2 or r2 < tiny
 */

#include <metal_stdlib>
#include <metal_atomic>

using namespace metal;

#define M_PI 3.14159265358979323846

/**
 * Matches the C++ struct GPUFluidParticle exactly:
 */
struct GPUFluidParticle {
  float x;       // position.x
  float y;       // position.y
  float vx;      // velocity.x
  float vy;      // velocity.y
  float vxHalf;  // velocity at half-step
  float vyHalf;
  float ax;      // acceleration.x
  float ay;      // acceleration.y
  float mass;
  float h;       // smoothing length
  float c;       // speed of sound (not used by old CPU code, but left in for consistency)
  float density;
  float pressure;
};

/**
 * Matches the C++ struct GPUGridCell exactly:
 */
constant int GPU_MAX_PER_CELL = 64;
struct GPUGridCell {
  atomic_int count;
  int indices[GPU_MAX_PER_CELL];
};

/**
 * Matches the C++ struct GPUFluidParams exactly:
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
 * @brief Clears the grid cell counters to zero before assigning cells.
 */
kernel void clearGrid(device GPUGridCell* grid [[buffer(0)]],
                      constant int& cellCount [[buffer(1)]],
                      uint tid [[thread_position_in_grid]]) {
  if (tid < (uint)cellCount) {
    atomic_store_explicit(&grid[tid].count, 0, memory_order_relaxed);
  }
}

/**
 * @brief Assign each particle to a cell based on position. Uses atomic increments.
 */
kernel void assignCells(device GPUFluidParticle* particles [[buffer(0)]],
                        constant int& particleCount [[buffer(1)]],
                        device GPUGridCell* grid [[buffer(2)]],
                        device const GPUFluidParams& params [[buffer(3)]],
                        uint tid [[thread_position_in_grid]]) {
  if (tid >= (uint)particleCount) {
    return;
  }

  GPUFluidParticle p = particles[tid];

  int cellX = int(floor((p.x - float(params.gridMinX)) / params.cellSize));
  int cellY = int(floor((p.y - float(params.gridMinY)) / params.cellSize));

  if (cellX < 0 || cellX >= params.gridDimX ||
      cellY < 0 || cellY >= params.gridDimY) {
    return;
  }
  int cellIndex = cellY * params.gridDimX + cellX;
  int oldCount = atomic_fetch_add_explicit(&grid[cellIndex].count, 1, memory_order_relaxed);
  if (oldCount < GPU_MAX_PER_CELL) {
    grid[cellIndex].indices[oldCount] = int(tid);
  }
}

/**
 * @brief Computes densities using a 2D poly6 kernel: W(r) = 4/(π*h^8)*(h^2−r^2)^3.
 *        Then sets pressure = stiffness*(rho−restDensity), clamped at zero if negative.
 */
kernel void computeDensity(device GPUFluidParticle* particles [[buffer(0)]],
                           constant int& particleCount [[buffer(1)]],
                           device const GPUGridCell* grid [[buffer(2)]],
                           device const GPUFluidParams& params [[buffer(3)]],
                           uint tid [[thread_position_in_grid]]) {
  if (tid >= (uint)particleCount) {
    return;
  }

  GPUFluidParticle self = particles[tid];
  float xi = self.x;
  float yi = self.y;
  float hi = self.h;

  float h2 = hi*hi;
  float h4 = h2*h2;
  float h8 = h4*h4;
  // 2D poly6 factor
  float poly6Const = 4.0f / (M_PI * h8);

  float accumDensity = 0.0f;

  // Find which cell the particle is in
  int baseCellX = int(floor((xi - float(params.gridMinX)) / params.cellSize));
  int baseCellY = int(floor((yi - float(params.gridMinY)) / params.cellSize));

  // Search the 3×3 neighborhood
  for (int ny = -1; ny <= 1; ny++) {
    for (int nx = -1; nx <= 1; nx++) {
      int cx = baseCellX + nx;
      int cy = baseCellY + ny;
      if (cx < 0 || cx >= params.gridDimX ||
          cy < 0 || cy >= params.gridDimY) {
        continue;
      }
      int cellIndex = cy * params.gridDimX + cx;
      int count = atomic_load_explicit(&grid[cellIndex].count, memory_order_relaxed);
      for (int i = 0; i < count; i++) {
        int neighborIdx = grid[cellIndex].indices[i];
        if (neighborIdx >= particleCount) {
          continue;
        }
        GPUFluidParticle neighbor = particles[neighborIdx];
        float dx = xi - neighbor.x;
        float dy = yi - neighbor.y;
        float r2 = dx*dx + dy*dy;

        // Use h_ij = 0.5*(hi + neighbor.h)
        float hj = neighbor.h;
        float h_ij = 0.5f*(hi + hj);
        float h_ij2 = h_ij*h_ij;
        if (r2 < h_ij2) {
          float diff = (h_ij2 - r2);
          float w = (4.0f/(M_PI*pow(h_ij,8.0f))) * diff*diff*diff;
          accumDensity += neighbor.mass * w;
        }
      }
    }
  }

  self.density = accumDensity;
  float rawPressure = params.stiffness * (accumDensity - params.restDensity);
  self.pressure = (rawPressure > 0.0f ? rawPressure : 0.0f);

  particles[tid] = self;
}

/**
 * @brief Computes symmetrical pressure + viscosity forces using spiky & viscosity kernels.
 *        No gravity, to mirror the CPU code exactly.
 */
kernel void computeForces(device GPUFluidParticle* particles [[buffer(0)]],
                          constant int& particleCount [[buffer(1)]],
                          device const GPUGridCell* grid [[buffer(2)]],
                          device const GPUFluidParams& params [[buffer(3)]],
                          uint tid [[thread_position_in_grid]]) {
  if (tid >= (uint)particleCount) {
    return;
  }

  GPUFluidParticle self = particles[tid];

  float fx = 0.0f;
  float fy = 0.0f;

  float xi = self.x;
  float yi = self.y;
  float pi = self.pressure;
  float rhoi = self.density;
  float hi = self.h;

  // 2D spiky: ∇W(r) factor = -30/(π*h^5)*(h−r)^2 (for r<h)
  // 2D viscosity "laplacian" factor ~ 40/(π*h^5)
  // We'll do an average h again: h_ij = 0.5*(h_i + h_j).
  // We'll skip if r2 >= (h_ij^2).

  int baseCellX = int(floor((xi - float(params.gridMinX)) / params.cellSize));
  int baseCellY = int(floor((yi - float(params.gridMinY)) / params.cellSize));

  for (int ny = -1; ny <= 1; ny++) {
    for (int nx = -1; nx <= 1; nx++) {
      int cx = baseCellX + nx;
      int cy = baseCellY + ny;
      if (cx < 0 || cx >= params.gridDimX ||
          cy < 0 || cy >= params.gridDimY) {
        continue;
      }
      int cellIndex = cy * params.gridDimX + cx;
      int count = atomic_load_explicit(&grid[cellIndex].count, memory_order_relaxed);

      for (int i = 0; i < count; i++) {
        int neighborIdx = grid[cellIndex].indices[i];
        if (neighborIdx == int(tid) || neighborIdx >= particleCount) {
          continue;
        }
        GPUFluidParticle neighbor = particles[neighborIdx];

        float dx = xi - neighbor.x;
        float dy = yi - neighbor.y;
        float r2 = dx*dx + dy*dy;
        if (r2 < 1e-14f) {
          // Very tiny separation => skip
          continue;
        }

        float h_j = neighbor.h;
        float h_ij = 0.5f*(hi + h_j);
        float h_ij2 = h_ij*h_ij;
        if (r2 >= h_ij2) {
          continue;
        }
        float r = sqrt(r2);

        float pj   = neighbor.pressure;
        float rhoj = neighbor.density;
        if (rhoj < 1e-12f) {
          continue;
        }

        // Pressure force
        float combinedP = (pi + pj) / 2.0f;  // symmetrical
        float spikyConst = -30.0f/(M_PI*pow(h_ij,5.0f));
        float diff  = (h_ij - r);
        float wSpiky = spikyConst * (diff*diff);
        float presTerm = -neighbor.mass * (combinedP / (rhoi*rhoi + rhoj*rhoj)) * wSpiky;
        float rx = dx / r;
        float ry = dy / r;
        fx += presTerm * rx;
        fy += presTerm * ry;

        // Viscosity
        float vx_ij = self.vx - neighbor.vx;
        float vy_ij = self.vy - neighbor.vy;
        float viscConst = 40.0f/(M_PI*pow(h_ij,5.0f));
        float wVisc = viscConst * diff;
        float fVisc = params.viscosity * neighbor.mass * (wVisc / rhoj);
        fx -= fVisc * vx_ij;
        fy -= fVisc * vy_ij;
      }
    }
  }

  self.ax = fx;
  self.ay = fy;
  particles[tid] = self;
}

/**
 * @brief Velocity Verlet half-step: vHalf = v + 0.5*a*dt, x = x + vHalf*dt
 */
kernel void velocityVerletHalf(device GPUFluidParticle* particles [[buffer(0)]],
                               device const GPUFluidParams& params [[buffer(1)]],
                               uint tid [[thread_position_in_grid]]) {
  if (tid >= params.particleCount) {
    return;
  }
  GPUFluidParticle p = particles[tid];

  float halfDt = params.halfDt;
  p.vxHalf = p.vx + halfDt * p.ax;
  p.vyHalf = p.vy + halfDt * p.ay;

  // Move positions
  float dt = params.dt;
  p.x += p.vxHalf * dt;
  p.y += p.vyHalf * dt;

  particles[tid] = p;
}

/**
 * @brief Velocity Verlet finish: v = vHalf + 0.5*a*dt
 */
kernel void velocityVerletFinish(device GPUFluidParticle* particles [[buffer(0)]],
                                 device const GPUFluidParams& params [[buffer(1)]],
                                 uint tid [[thread_position_in_grid]]) {
  if (tid >= params.particleCount) {
    return;
  }
  GPUFluidParticle p = particles[tid];

  float halfDt = params.halfDt;
  p.vx = p.vxHalf + halfDt * p.ax;
  p.vy = p.vyHalf + halfDt * p.ay;

  particles[tid] = p;
}
