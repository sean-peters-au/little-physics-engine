/**
 * @fileoverview fluid_kernels.metal
 * @brief 2D SPH solver kernels with GPU-based bounding box (no grid-stride loop),
 *        plus neutral bounding box for empty groups, requiring threadgroup memory allocation.
 */

#include <metal_stdlib>
#include <metal_atomic>

using namespace metal;

#define M_PI 3.14159265358979323846

inline float poly6Coeff2D(float h) {
    float h2 = h * h;
    float h4 = h2*h2;
    float h8 = h4*h4;
    return 4.0f / (M_PI * h8);
}

inline float spikyCoeff2D(float h) {
    float h2 = h*h;
    float h4 = h2*h2;
    float h5 = h4*h;
    return -30.0f / (M_PI * h5);
}

inline float viscLaplacianCoeff2D(float h) {
    float h2 = h*h;
    float h4 = h2*h2;
    float h5 = h4*h;
    return 40.0f / (M_PI * h5);
}

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

struct BBoxParams {
    int particleCount;
    int numThreadgroups;
};

struct BBox {
    float minX;
    float maxX;
    float minY;
    float maxY;
};

/**
 * Clear grid counts
 */
kernel void clearGrid(
    device GPUGridCell* grid [[buffer(0)]],
    constant int& cellCount  [[buffer(1)]],
    uint globalID            [[thread_position_in_grid]])
{
    if (globalID < (uint)cellCount) {
        atomic_store_explicit(&grid[globalID].count, 0, memory_order_relaxed);
    }
}

/**
 * Assign cells
 */
kernel void assignCells(
    device GPUFluidParticle* particles [[buffer(0)]],
    constant int& particleCount        [[buffer(1)]],
    device GPUGridCell* grid          [[buffer(2)]],
    device const GPUFluidParams& p     [[buffer(3)]],
    uint globalID                      [[thread_position_in_grid]])
{
    if (globalID >= (uint)particleCount) {
        return;
    }
    GPUFluidParticle part = particles[globalID];

    float px = part.x + 1e-6f;
    float py = part.y + 1e-6f;
    int gx = int(floor(px / p.cellSize));
    int gy = int(floor(py / p.cellSize));

    int cellX = gx - p.gridMinX;
    int cellY = gy - p.gridMinY;
    if (cellX < 0 || cellX >= p.gridDimX ||
        cellY < 0 || cellY >= p.gridDimY) {
        return;
    }

    int cellIndex = cellY * p.gridDimX + cellX;
    int oldCount = atomic_fetch_add_explicit(&grid[cellIndex].count, 1, memory_order_relaxed);
    if (oldCount < GPU_MAX_PER_CELL) {
        grid[cellIndex].indices[oldCount] = int(globalID);
    }
}

/**
 * Compute densities (poly6)
 */
kernel void computeDensity(
    device GPUFluidParticle* particles [[buffer(0)]],
    constant int& particleCount        [[buffer(1)]],
    device const GPUGridCell* grid     [[buffer(2)]],
    device const GPUFluidParams& p     [[buffer(3)]],
    uint globalID                      [[thread_position_in_grid]])
{
    if (globalID >= (uint)particleCount) {
        return;
    }
    GPUFluidParticle self = particles[globalID];

    float xi = self.x;
    float yi = self.y;
    float hi = (self.h <= 0.f) ? 0.05f : self.h;
    float h2 = hi*hi;
    float poly6 = poly6Coeff2D(hi);

    float accumDensity = 0.0f;
    float px = xi + 1e-6f;
    float py = yi + 1e-6f;
    int gx = int(floor(px / p.cellSize));
    int gy = int(floor(py / p.cellSize));
    int cellX = gx - p.gridMinX;
    int cellY = gy - p.gridMinY;

    for (int ny=-1; ny<=1; ny++) {
        for (int nx=-1; nx<=1; nx++) {
            int cx = cellX + nx;
            int cy = cellY + ny;
            if (cx<0 || cx>=p.gridDimX ||
                cy<0 || cy>=p.gridDimY) {
                continue;
            }
            int cellIdx = cy*p.gridDimX + cx;
            int count = atomic_load_explicit(&grid[cellIdx].count, memory_order_relaxed);
            for (int c=0; c<count; c++) {
                int nbrID = grid[cellIdx].indices[c];
                if (nbrID >= (int)particleCount) {
                    continue;
                }
                GPUFluidParticle nbr = particles[nbrID];
                float dx = xi - nbr.x;
                float dy = yi - nbr.y;
                float r2 = dx*dx + dy*dy;
                if (r2 < h2) {
                    float diff = h2 - r2;
                    float w = poly6*diff*diff*diff;
                    accumDensity += nbr.mass * w;
                }
            }
        }
    }

    self.density = accumDensity;
    float pres = p.stiffness*(accumDensity - p.restDensity);
    if (pres < 0.f) {
        pres = 0.f;
    }
    self.pressure = pres;
    particles[globalID] = self;
}

/**
 * Compute symmetrical pressure + viscosity forces
 */
kernel void computeForces(
    device GPUFluidParticle* particles [[buffer(0)]],
    constant int& particleCount        [[buffer(1)]],
    device const GPUGridCell* grid     [[buffer(2)]],
    device const GPUFluidParams& pa    [[buffer(3)]],
    uint globalID                      [[thread_position_in_grid]])
{
    if (globalID >= (uint)particleCount) {
        return;
    }
    GPUFluidParticle self = particles[globalID];

    float xi = self.x;
    float yi = self.y;
    float pi = self.pressure;
    float rhoi = self.density;
    float hi = (self.h<=0.f) ? 0.05f : self.h;

    float sumFx = 0.f;
    float sumFy = 0.f;

    float px = xi + 1e-6f;
    float py = yi + 1e-6f;
    int gx = int(floor(px / pa.cellSize));
    int gy = int(floor(py / pa.cellSize));
    int cellX = gx - pa.gridMinX;
    int cellY = gy - pa.gridMinY;

    for (int ny=-1; ny<=1; ny++) {
        for (int nx=-1; nx<=1; nx++) {
            int cx = cellX + nx;
            int cy = cellY + ny;
            if (cx<0 || cx>=pa.gridDimX ||
                cy<0 || cy>=pa.gridDimY) {
                continue;
            }
            int cellIdx = cy*pa.gridDimX + cx;
            int count = atomic_load_explicit(&grid[cellIdx].count, memory_order_relaxed);
            for (int c=0; c<count; c++) {
                int nbrID = grid[cellIdx].indices[c];
                if (nbrID == int(globalID) || nbrID >= (int)particleCount) {
                    continue;
                }
                GPUFluidParticle nbr = particles[nbrID];
                float dx = xi - nbr.x;
                float dy = yi - nbr.y;
                float r2 = dx*dx + dy*dy;
                if (r2 < 1e-14f) {
                    continue;
                }
                float hj = (nbr.h<=0.f) ? 0.05f : nbr.h;
                float h_ij = 0.5f*(hi + hj);
                float h_ij2 = h_ij*h_ij;
                if (r2 >= h_ij2) {
                    continue;
                }
                float r = sqrt(r2);
                float pj = nbr.pressure;
                float rhoj = nbr.density;
                if (rhoj<1e-12f || rhoi<1e-12f) {
                    continue;
                }
                float term = (pi/(rhoi*rhoi)) + (pj/(rhoj*rhoj));
                float spF  = spikyCoeff2D(h_ij);
                float diff = (h_ij - r);
                float wSpiky = spF*(diff*diff);
                float rx = dx/r;
                float ry = dy/r;
                float fxPress = -nbr.mass*term*wSpiky;
                float fx = fxPress*rx;
                float fy = fxPress*ry;

                // viscosity
                float vx_ij = self.vx - nbr.vx;
                float vy_ij = self.vy - nbr.vy;
                float lapC = viscLaplacianCoeff2D(h_ij);
                float wVisc = lapC*diff;
                float fVisc = pa.viscosity*nbr.mass*(wVisc/rhoj);
                fx -= fVisc * vx_ij;
                fy -= fVisc * vy_ij;

                sumFx += fx;
                sumFy += fy;
            }
        }
    }

    self.ax = sumFx;
    self.ay = sumFy;
    particles[globalID] = self;
}

/**
 * velocityVerletHalf
 */
kernel void velocityVerletHalf(
    device GPUFluidParticle* particles [[buffer(0)]],
    device const GPUFluidParams& p     [[buffer(1)]],
    uint globalID                      [[thread_position_in_grid]])
{
    if (globalID >= p.particleCount) {
        return;
    }
    GPUFluidParticle part = particles[globalID];
    float halfDt = p.halfDt;
    part.vxHalf = part.vx + halfDt*part.ax;
    part.vyHalf = part.vy + halfDt*part.ay;
    part.x += part.vxHalf * p.dt;
    part.y += part.vyHalf * p.dt;
    particles[globalID] = part;
}

/**
 * velocityVerletFinish
 */
kernel void velocityVerletFinish(
    device GPUFluidParticle* particles [[buffer(0)]],
    device const GPUFluidParams& p     [[buffer(1)]],
    uint globalID                      [[thread_position_in_grid]])
{
    if (globalID >= p.particleCount) {
        return;
    }
    GPUFluidParticle part = particles[globalID];
    float halfDt = p.halfDt;
    part.vx = part.vxHalf + halfDt*part.ax;
    part.vy = part.vyHalf + halfDt*part.ay;
    particles[globalID] = part;
}

/**
 * @brief 1D bounding box kernel that writes one BBox partial per threadgroup.
 * Must have setThreadgroupMemoryLength for localShared and localValidCount.
 */
kernel void computeBoundingBox(
    device const GPUFluidParticle* particles [[buffer(0)]],
    device const BBoxParams& bboxParams       [[buffer(1)]],
    device BBox* partialBoxes                 [[buffer(2)]],
    uint globalID                             [[thread_position_in_grid]],
    uint localID                              [[thread_position_in_threadgroup]],
    uint groupID                              [[threadgroup_position_in_grid]],
    threadgroup BBox* localShared             [[threadgroup(3)]],
    threadgroup atomic_int* localValidCount   [[threadgroup(4)]])
{
    float minX =  1e30f;
    float maxX = -1e30f;
    float minY =  1e30f;
    float maxY = -1e30f;

    bool isValid = false;
    if (globalID < (uint)bboxParams.particleCount) {
        isValid = true;
        float x = particles[globalID].x;
        float y = particles[globalID].y;
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
    }

    if (localID == 0) {
        atomic_store_explicit(localValidCount, 0, memory_order_relaxed);
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);

    if (isValid) {
        atomic_fetch_add_explicit(localValidCount, 1, memory_order_relaxed);
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);

    localShared[localID].minX = minX;
    localShared[localID].maxX = maxX;
    localShared[localID].minY = minY;
    localShared[localID].maxY = maxY;
    threadgroup_barrier(mem_flags::mem_threadgroup);

    // local reduce
    const uint blockSize = 256;
    for (uint offset = blockSize / 2; offset > 0; offset >>= 1) {
        if (localID < offset) {
            BBox other = localShared[localID + offset];
            BBox self  = localShared[localID];
            if (other.minX < self.minX) self.minX = other.minX;
            if (other.maxX > self.maxX) self.maxX = other.maxX;
            if (other.minY < self.minY) self.minY = other.minY;
            if (other.maxY > self.maxY) self.maxY = other.maxY;
            localShared[localID] = self;
        }
        threadgroup_barrier(mem_flags::mem_threadgroup);
    }

    if (localID == 0) {
        int countInGroup = atomic_load_explicit(localValidCount, memory_order_relaxed);
        if (countInGroup == 0) {
            // no valid threads => store neutral bounding box
            partialBoxes[groupID].minX =  1e30f;
            partialBoxes[groupID].maxX = -1e30f;
            partialBoxes[groupID].minY =  1e30f;
            partialBoxes[groupID].maxY = -1e30f;
        } else {
            partialBoxes[groupID] = localShared[0];
        }
    }
}
