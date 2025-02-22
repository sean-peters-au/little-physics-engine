/**
 * @fileoverview fluid_kernels.metal
 * @brief EXACT mirror of your old CPU solver in 2D. 
 *
 * Key differences from your prior GPU code:
 *  - computeDensity uses only hi, not h_ij
 *  - computeForces uses h_ij = 0.5*(hi + hj)
 *  - Pressure term = pi/(rhoi^2) + pj/(rhoj^2)
 *  - Negative pressures clamped to 0
 *  - 2D kernel constants match your CPU code (poly6Coeff2D, spikyCoeff2D, viscLaplacianCoeff2D)
 */

#include <metal_stdlib>
#include <metal_atomic>

using namespace metal;

// For convenience
#define M_PI 3.14159265358979323846

// 2D poly6 factor:   4 / (π * h^8)
inline float poly6Coeff2D(float h) {
    float h2 = h * h;
    float h4 = h2 * h2;
    float h8 = h4 * h4;
    return 4.0f / (M_PI * h8);
}

// 2D spiky factor:  -30 / (π * h^5)
inline float spikyCoeff2D(float h) {
    float h2 = h*h;
    float h4 = h2*h2;
    float h5 = h4*h;
    return -30.0f / (M_PI * h5);
}

// 2D viscosity Laplacian factor: 40 / (π * h^5)
inline float viscLaplacianCoeff2D(float h) {
    float h2 = h*h;
    float h4 = h2*h2;
    float h5 = h4*h;
    return 40.0f / (M_PI * h5);
}

/**
 * @brief Matches the C++ GPUFluidParticle struct exactly.
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
    float h;   // smoothing length
    float c;   // speed of sound (unused by CPU code, but left for completeness)
    float density;
    float pressure;
};

/**
 * @brief Matches the CPU struct GPUGridCell (int count, int indices[]).
 */
constant int GPU_MAX_PER_CELL = 64;
struct GPUGridCell {
    atomic_int count;
    int indices[GPU_MAX_PER_CELL];
};

/**
 * @brief Matches the CPU struct GPUFluidParams.
 */
struct GPUFluidParams {
    float cellSize;
    int gridDimX;
    int gridDimY;
    int gridMinX;
    int gridMinY;
    float restDensity;  // e.g. 1000
    float stiffness;    // e.g. 500
    float viscosity;    // e.g. 0.1
    float dt;           // sub-step
    float halfDt;       // 0.5f * dt
    uint particleCount;
};

/**
 * @brief Clears the grid cell counters to zero.
 */
kernel void clearGrid(device GPUGridCell* grid [[buffer(0)]],
                      constant int& cellCount [[buffer(1)]],
                      uint gid [[thread_position_in_grid]]) {
    if (gid < (uint)cellCount) {
        atomic_store_explicit(&grid[gid].count, 0, memory_order_relaxed);
    }
}

/**
 * @brief Assigns each particle to a cell, just like CPU code does with uniform grid.
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
 * @brief Compute densities, EXACTLY like CPU code:
 *        - uses hi (NOT h_ij) for the search radius
 *        - uses 2D poly6
 *        - clamp negative pressure to zero *AFTER* we compute it in a separate kernel
 */
kernel void computeDensity(device GPUFluidParticle* particles [[buffer(0)]],
                           constant int& particleCount [[buffer(1)]],
                           device const GPUGridCell* grid [[buffer(2)]],
                           device const GPUFluidParams& params [[buffer(3)]],
                           uint gid [[thread_position_in_grid]]) {
    if (gid >= (uint)particleCount) {
        return;
    }

    // Retrieve "i" particle
    GPUFluidParticle self = particles[gid];
    float xi = self.x;
    float yi = self.y;
    float hi = self.h;  // CPU code uses hi here
    if (hi <= 0.0f) {
        hi = 0.05f; // fallback
    }
    float h2 = hi*hi;

    // Precompute poly6 coefficient
    float poly6C = poly6Coeff2D(hi);

    float accumDensity = 0.0f;

    // figure out which cell we're in
    int baseCellX = int(floor((xi - float(params.gridMinX)) / params.cellSize));
    int baseCellY = int(floor((yi - float(params.gridMinY)) / params.cellSize));

    // 3x3 neighbor cells
    for (int ny = -1; ny <= 1; ny++) {
        for (int nx = -1; nx <= 1; nx++) {
            int cx = baseCellX + nx;
            int cy = baseCellY + ny;
            // skip invalid cells
            if (cx < 0 || cy < 0 || cx >= params.gridDimX || cy >= params.gridDimY) {
                continue;
            }
            int cellIndex = cy * params.gridDimX + cx;
            int count = atomic_load_explicit(&grid[cellIndex].count, memory_order_relaxed);
            for (int c = 0; c < count; c++) {
                int neighborIdx = grid[cellIndex].indices[c];
                if (neighborIdx >= (int)particleCount) {
                    continue;
                }
                GPUFluidParticle neighbor = particles[neighborIdx];
                float dx = xi - neighbor.x;
                float dy = yi - neighbor.y;
                float r2 = dx*dx + dy*dy;
                // CPU code: if (r2 < hi^2) => do poly6
                if (r2 < h2) {
                    float diff = h2 - r2;
                    float w = poly6C * diff * diff * diff;
                    accumDensity += neighbor.mass * w;
                }
            }
        }
    }

    self.density = accumDensity;
    // We'll compute pressure in a separate step or in the same step, but let's do it here:
    float p = params.stiffness * (accumDensity - params.restDensity);
    // CPU code clamps negative to zero:
    if (p < 0.0f) {
        p = 0.0f;
    }
    self.pressure = p;

    particles[gid] = self;
}

/**
 * @brief Computes symmetrical pressure + viscosity forces, EXACTLY like CPU code:
 *        - uses h_ij = 0.5*(h_i + h_j)
 *        - pressure term = pi/(rhoi^2) + pj/(rhoj^2)
 *        - spiky & viscosity in 2D
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
    float xi = self.x;
    float yi = self.y;

    float pi   = self.pressure;
    float rhoi = self.density;
    float hi   = self.h;
    if (hi <= 0.f) {
        hi = 0.05f;
    }

    float sumFx = 0.0f;
    float sumFy = 0.0f;

    // find base cell
    int baseCellX = int(floor((xi - float(params.gridMinX)) / params.cellSize));
    int baseCellY = int(floor((yi - float(params.gridMinY)) / params.cellSize));

    // Explore 3x3 neighbors
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

            for (int c = 0; c < count; c++) {
                int neighborIdx = grid[cellIndex].indices[c];
                if (neighborIdx == int(gid) || neighborIdx >= (int)particleCount) {
                    continue;
                }
                GPUFluidParticle neighbor = particles[neighborIdx];

                float dx = xi - neighbor.x;
                float dy = yi - neighbor.y;
                float r2 = dx*dx + dy*dy;
                if (r2 < 1e-14f) {
                    // skip extremely small distances
                    continue;
                }

                // average smoothing length for i & j
                float hj = neighbor.h;
                if (hj <= 0.f) {
                    hj = 0.05f;
                }
                float h_ij = 0.5f*(hi + hj);
                float h_ij2 = h_ij*h_ij;
                if (r2 >= h_ij2) {
                    continue;
                }

                float r = sqrt(r2);
                float pj   = neighbor.pressure;
                float rhoj = neighbor.density;
                // CPU code does:
                //    float term = pi/(rhoi*rhoi) + pj/(rhoj*rhoj);
                //    spikyFactor, etc.
                if (rhoj < 1e-12f || rhoi < 1e-12f) {
                    continue;
                }
                float term = (pi / (rhoi*rhoi)) + (pj / (rhoj*rhoj));

                // Spiky
                float spikyF = spikyCoeff2D(h_ij);
                float diff   = (h_ij - r);
                float wSpiky = spikyF * (diff*diff);  // as CPU code does
                float massj  = neighbor.mass;

                float fxPress = -massj * term * wSpiky;
                float rx = dx / r;
                float ry = dy / r;
                float fx = fxPress * rx;
                float fy = fxPress * ry;

                // viscosity
                float vx_ij = self.vx - neighbor.vx;
                float vy_ij = self.vy - neighbor.vy;

                float lapC = viscLaplacianCoeff2D(h_ij);
                float wVisc = lapC * diff; 
                float fVisc = params.viscosity * massj * (wVisc / rhoj);
                fx -= fVisc * vx_ij;
                fy -= fVisc * vy_ij;

                sumFx += fx;
                sumFy += fy;
            }
        }
    }

    self.ax = sumFx;
    self.ay = sumFy;
    particles[gid] = self;
}

/**
 * @brief Velocity Verlet half-step: 
 *   vHalf = v + 0.5*a*dt
 *   x     = x + vHalf*dt
 */
kernel void velocityVerletHalf(device GPUFluidParticle* particles [[buffer(0)]],
                               device const GPUFluidParams& params [[buffer(1)]],
                               uint gid [[thread_position_in_grid]]) {
    if (gid >= params.particleCount) {
        return;
    }
    GPUFluidParticle p = particles[gid];

    float halfDt = params.halfDt;
    p.vxHalf = p.vx + halfDt*p.ax;
    p.vyHalf = p.vy + halfDt*p.ay;
    float dt = params.dt;
    p.x += p.vxHalf * dt;
    p.y += p.vyHalf * dt;

    particles[gid] = p;
}

/**
 * @brief Velocity Verlet finish:
 *   v = vHalf + 0.5*a*dt
 */
kernel void velocityVerletFinish(device GPUFluidParticle* particles [[buffer(0)]],
                                 device const GPUFluidParams& params [[buffer(1)]],
                                 uint gid [[thread_position_in_grid]]) {
    if (gid >= params.particleCount) {
        return;
    }
    GPUFluidParticle p = particles[gid];

    float halfDt = params.halfDt;
    p.vx = p.vxHalf + halfDt*p.ax;
    p.vy = p.vyHalf + halfDt*p.ay;

    particles[gid] = p;
}
