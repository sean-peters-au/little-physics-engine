/**
 * @fileoverview fluid_kernels.metal
 * @brief EXACT mirror of the CPU solver in 2D, with integer-based grid indexing.
 *
 * Differences from the previous version:
 *  - GPUFluidParams reordered to match CPU code.
 *  - assignCells kernel uses:
 *       float px = p.x + 1e-6f;
 *       int gx = int(floor(px / params.cellSize));
 *       int cellX = gx - params.gridMinX;
 *       etc.
 *    rather than subtracting world-based gridMinX.
 */

#include <metal_stdlib>
#include <metal_atomic>

using namespace metal;

#define M_PI 3.14159265358979323846

// 2D poly6 factor: 4 / (π * h^8)
inline float poly6Coeff2D(float h) {
    float h2 = h * h;
    float h4 = h2 * h2;
    float h8 = h4 * h4;
    return 4.0f / (M_PI * h8);
}

// 2D spiky factor: -30 / (π * h^5)
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
 * Matches the C++ GPUFluidParticle struct exactly.
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

constant int GPU_MAX_PER_CELL = 64;
struct GPUGridCell {
    atomic_int count;
    int indices[GPU_MAX_PER_CELL];
};

/**
 * Matches the CPU struct GPUFluidParams, with the order changed to:
 *   float cellSize;
 *   int   gridMinX;
 *   int   gridMinY;
 *   int   gridDimX;
 *   int   gridDimY;
 *   float restDensity;
 *   float stiffness;
 *   float viscosity;
 *   float dt;
 *   float halfDt;
 *   uint  particleCount;
 */
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

kernel void clearGrid(device GPUGridCell* grid [[buffer(0)]],
                      constant int& cellCount [[buffer(1)]],
                      uint gid [[thread_position_in_grid]]) {
    if (gid < (uint)cellCount) {
        atomic_store_explicit(&grid[gid].count, 0, memory_order_relaxed);
    }
}

/**
 * @brief Assigns each particle to a cell using integer-based indexing:
 *        gx = floor((x+1e-6)/cellSize), cellX = gx - gridMinX
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

    float px = p.x + 1e-6f;
    float py = p.y + 1e-6f;
    int gx = int(floor(px / params.cellSize));
    int gy = int(floor(py / params.cellSize));

    int cellX = gx - params.gridMinX;
    int cellY = gy - params.gridMinY;

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
 * @brief Compute densities exactly like CPU code:
 *        - uses hi for the search radius
 *        - 2D poly6
 *        - clamp negative pressure in computeDensity itself
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
    float xi = self.x;
    float yi = self.y;
    float hi = self.h;
    if (hi <= 0.0f) {
        hi = 0.05f;
    }
    float h2 = hi * hi;
    float poly6C = poly6Coeff2D(hi);

    float accumDensity = 0.0f;

    float px = xi + 1e-6f;
    float py = yi + 1e-6f;
    int baseGx = int(floor(px / params.cellSize));
    int baseGy = int(floor(py / params.cellSize));

    int cellX = baseGx - params.gridMinX;
    int cellY = baseGy - params.gridMinY;

    for (int ny = -1; ny <= 1; ny++) {
        for (int nx = -1; nx <= 1; nx++) {
            int cx = cellX + nx;
            int cy = cellY + ny;
            if (cx < 0 || cx >= params.gridDimX || cy < 0 || cy >= params.gridDimY) {
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
                if (r2 < h2) {
                    float diff = h2 - r2;
                    float w = poly6C * diff * diff * diff;
                    accumDensity += neighbor.mass * w;
                }
            }
        }
    }

    self.density = accumDensity;
    float p = params.stiffness * (accumDensity - params.restDensity);
    if (p < 0.0f) {
        p = 0.0f;
    }
    self.pressure = p;

    particles[gid] = self;
}

/**
 * @brief Computes symmetrical pressure + viscosity forces using h_ij = 0.5*(h_i + h_j).
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
    float pi = self.pressure;
    float rhoi = self.density;
    float hi = self.h;
    if (hi <= 0.f) {
        hi = 0.05f;
    }
    float sumFx = 0.0f;
    float sumFy = 0.0f;

    float px = xi + 1e-6f;
    float py = yi + 1e-6f;
    int baseGx = int(floor(px / params.cellSize));
    int baseGy = int(floor(py / params.cellSize));
    int cellX = baseGx - params.gridMinX;
    int cellY = baseGy - params.gridMinY;

    for (int ny = -1; ny <= 1; ny++) {
        for (int nx = -1; nx <= 1; nx++) {
            int cx = cellX + nx;
            int cy = cellY + ny;
            if (cx < 0 || cx >= params.gridDimX || cy < 0 || cy >= params.gridDimY) {
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
                    continue;
                }
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
                if (rhoj < 1e-12f || rhoi < 1e-12f) {
                    continue;
                }
                float term = (pi / (rhoi*rhoi)) + (pj / (rhoj*rhoj));
                float spikyF = spikyCoeff2D(h_ij);
                float diff   = (h_ij - r);
                float wSpiky = spikyF * (diff*diff);
                float rx = dx / r;
                float ry = dy / r;
                float fxPress = -neighbor.mass * term * wSpiky;
                float fx = fxPress * rx;
                float fy = fxPress * ry;

                float vx_ij = self.vx - neighbor.vx;
                float vy_ij = self.vy - neighbor.vy;
                float lapC = viscLaplacianCoeff2D(h_ij);
                float wVisc = lapC * diff;
                float fVisc = params.viscosity * neighbor.mass * (wVisc / rhoj);
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
    p.vxHalf = p.vx + halfDt * p.ax;
    p.vyHalf = p.vy + halfDt * p.ay;
    p.x += p.vxHalf * params.dt;
    p.y += p.vyHalf * params.dt;
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
    p.vx = p.vxHalf + halfDt * p.ax;
    p.vy = p.vyHalf + halfDt * p.ay;
    particles[gid] = p;
}
