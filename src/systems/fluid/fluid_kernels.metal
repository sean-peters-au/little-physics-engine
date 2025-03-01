/**
 * @fileoverview fluid_kernels.metal
 * @brief 2D SPH solver kernels with GPU-based bounding box,
 *        GPU-based rigid-fluid position solver, and
 *        a new GPU-based rigid-fluid impulse solver for drag/buoyancy.
 */

#include <metal_stdlib>
#include <metal_atomic>

using namespace metal;

/**
 * Constants / math utilities
 */
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

/**
 * GPU data structures
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
 * @brief Params for fluid sub-steps (already used by other kernels).
 * Also holds dt = sub-step delta-time.
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
    float dt;       // sub-step time
    float halfDt;
    uint  particleCount;
};

/**
 * @brief For bounding box reductions
 */
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
 * Rigid body data
 *
 * We assume these fields match your CPU's GPURigidBody. Notice accumFx/Fy/Torque
 * are stored as normal floats; we’ll do atomic ops on them via reinterpret_cast.
 */
enum GPURigidShapeType : int {
    Circle = 0,
    Polygon = 1
};

#define GPU_POLYGON_MAX_VERTS 16

struct GPURigidBody {
    GPURigidShapeType shapeType;
    float posX;
    float posY;
    float angle;
    float radius;
    int vertCount;
    float vertsX[GPU_POLYGON_MAX_VERTS];
    float vertsY[GPU_POLYGON_MAX_VERTS];

    // Extended fields used for impulse accumulation:
    float vx;
    float vy;
    float omega;
    float mass;
    float inertia;

    float minX;
    float maxX;
    float minY;
    float maxY;

    float accumFx;
    float accumFy;
    float accumTorque;
};

/**
 * Inline helpers for rigid-fluid solver
 */
inline bool pointInPolygon(float px, float py,
                           thread const GPURigidBody &body)
{
    int vCount = body.vertCount;
    if (vCount < 3) {
        return false;
    }

    bool inside = false;
    for (int i = 0, j = vCount - 1; i < vCount; j = i++) {
        float xi = body.vertsX[i];
        float yi = body.vertsY[i];
        float xj = body.vertsX[j];
        float yj = body.vertsY[j];

        bool intersect = ((yi > py) != (yj > py)) &&
                         (px < (xj - xi)*(py - yi)/(yj - yi) + xi);
        if (intersect) {
            inside = !inside;
        }
    }
    return inside;
}

inline void closestPointOnPolygon(float px, float py,
                                  thread const GPURigidBody &body,
                                  thread float &outClosestX,
                                  thread float &outClosestY)
{
    int vCount = body.vertCount;
    outClosestX = px;
    outClosestY = py;

    if (vCount < 2) {
        return;
    }

    float minDistSq = 1e12f;

    for (int i = 0; i < vCount; i++) {
        int j = (i + 1) % vCount;
        float x1 = body.vertsX[i];
        float y1 = body.vertsY[i];
        float x2 = body.vertsX[j];
        float y2 = body.vertsY[j];

        float ex = x2 - x1;
        float ey = y2 - y1;
        float eLenSq = ex*ex + ey*ey;
        if (eLenSq < 1e-16f) {
            continue;
        }
        float dx = px - x1;
        float dy = py - y1;
        float t = (dx*ex + dy*ey) / eLenSq;
        if (t < 0.f) t = 0.f;
        if (t > 1.f) t = 1.f;
        float closestX = x1 + t*ex;
        float closestY = y1 + t*ey;

        float cdx = px - closestX;
        float cdy = py - closestY;
        float distSq = cdx*cdx + cdy*cdy;
        if (distSq < minDistSq) {
            minDistSq = distSq;
            outClosestX = closestX;
            outClosestY = closestY;
        }
    }
}

/**
 * Kernel: clearGrid (unchanged)
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
 * Kernel: assignCells (unchanged)
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
 * Kernel: computeDensity (unchanged)
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
 * Kernel: computeForces (unchanged)
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
 * Kernel: velocityVerletHalf (unchanged)
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
 * Kernel: velocityVerletFinish (unchanged)
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
 * Kernel: computeBoundingBox (unchanged)
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

    // Local reduce
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
            partialBoxes[groupID].minX =  1e30f;
            partialBoxes[groupID].maxX = -1e30f;
            partialBoxes[groupID].minY =  1e30f;
            partialBoxes[groupID].maxY = -1e30f;
        } else {
            partialBoxes[groupID] = localShared[0];
        }
    }
}

/**
 * Kernel: rigidFluidPositionSolver (unchanged but with your sign flip)
 */
kernel void rigidFluidPositionSolver(
    device GPUFluidParticle* particles [[buffer(0)]],
    constant int& fluidCount           [[buffer(1)]],
    device GPURigidBody* rigids        [[buffer(2)]],
    constant int& rigidCount           [[buffer(3)]],
    uint globalID                      [[thread_position_in_grid]])
{
    if (globalID >= (uint)fluidCount) {
        return;
    }

    float2 accumCorr = float2(0.0f, 0.0f);
    GPUFluidParticle p = particles[globalID];
    float px = p.x;
    float py = p.y;

    const float safetyMargin = 0.001f;
    const float relaxFactor  = 0.3f;

    for (int r = 0; r < rigidCount; r++) {
        GPURigidBody body = rigids[r];

        // bounding box check
        if (px < body.minX || px > body.maxX ||
            py < body.minY || py > body.maxY) {
            continue;
        }

        if (body.shapeType == Circle) {
            float dx = px - body.posX;
            float dy = py - body.posY;
            float dist2 = dx*dx + dy*dy;
            float radius = body.radius;
            if (dist2 < radius*radius) {
                float dist = sqrt(dist2);
                if (dist < 1e-10f) {
                    dist = 1e-10f;
                    dx = 1.0f; dy = 0.0f;
                }
                float pen = (radius - dist) + safetyMargin;
                float2 dir = float2(dx, dy) / dist;
                float2 corr = dir * pen * relaxFactor;
                accumCorr += corr;
            }
        }
        else if (body.shapeType == Polygon) {
            if (body.vertCount < 3) {
                continue;
            }
            bool inside = pointInPolygon(px, py, body);
            if (inside) {
                float cx, cy;
                closestPointOnPolygon(px, py, body, cx, cy);
                float cdx = px - cx;
                float cdy = py - cy;
                float d2 = cdx*cdx + cdy*cdy;
                float d = sqrt(d2);
                if (d < 1e-10f) {
                    d = 1e-10f;
                    cdx = 1.0f; cdy = 0.0f;
                }
                float pen = d + safetyMargin;
                float2 dir = float2(cdx, cdy) / d;
                float2 corr = dir * pen * relaxFactor;
                accumCorr += corr;
            }
        }
    }

    // Flip sign to push fluid outwards
    p.x -= accumCorr.x;
    p.y -= accumCorr.y;

    // Some minimal clamping
    if (p.x < 0.f) p.x = 0.f;
    if (p.y < 0.f) p.y = 0.f;

    particles[globalID] = p;
}

/**
 * Kernel: rigidFluidImpulseSolver
 * @brief For each fluid thread, we loop over all rigids. If the fluid is near/inside the rigid,
 *        we compute drag & buoyancy, then atomically add to rigids[r].accumFx, accumFy, accumTorque.
 *
 * This is a naive O(N_fluid * N_rigid) approach. For large simulations, use better broad-phase or reduce logic.
 *
 * We incorporate param.dt each sub-step so the total effect accumulates across multiple sub-steps.
 */
kernel void rigidFluidImpulseSolver(
    device GPUFluidParticle* fluidParticles [[buffer(0)]],
    constant int& fluidCount                [[buffer(1)]],
    device GPURigidBody* rigids            [[buffer(2)]],
    constant int& rigidCount                [[buffer(3)]],
    device const GPUFluidParams& param      [[buffer(4)]],
    uint globalID                           [[thread_position_in_grid]])
{
    if (globalID >= (uint)fluidCount || rigidCount <= 0) {
        return;
    }

    // Basic constants
    const float dt = param.dt;      // sub-step time
    const float dragCoefficient = 0.5f;
    const float maxForce = 5.0f;    // limit per fluid
    const float maxTorque= 1.0f;    // limit per fluid

    GPUFluidParticle fp = fluidParticles[globalID];

    float px = fp.x;
    float py = fp.y;
    float vxF = fp.vx;
    float vyF = fp.vy;
    float densityF = fp.density;  // fluid density near this particle
    if (densityF < 1e-6f) {
        densityF = 1000.f;        // fallback if uninitialized
    }

    // For each rigid
    for (int r = 0; r < rigidCount; r++) {
        // Access rigid
        GPURigidBody rb = rigids[r];

        // Broad-phase check
        if (px < rb.minX || px > rb.maxX ||
            py < rb.minY || py > rb.maxY) {
            continue;
        }

        // Check circle or polygon
        bool inside = false;
        float rx = px - rb.posX;
        float ry = py - rb.posY;
        if (rb.shapeType == Circle) {
            float dist2 = rx*rx + ry*ry;
            if (dist2 < (rb.radius * rb.radius)) {
                inside = true;
            }
        }
        else if (rb.shapeType == Polygon && rb.vertCount >= 3) {
            // naive polygon test
            inside = pointInPolygon(px, py, rb);
        }
        if (!inside) {
            continue;
        }

        //--------------------------------------------------
        // If the particle is inside or near the rigid,
        // compute e.g. drag, buoyancy, etc.
        //--------------------------------------------------
        float2 relativePos = float2(rx, ry);

        // Rigid's velocity at that point (including rotation)
        // cross(omega, relativePos) = [-omega*ry, +omega*rx]
        float2 rotVel = float2(-rb.omega * ry, rb.omega * rx);
        float2 rigidVel = float2(rb.vx, rb.vy) + rotVel;
        float2 relVel = float2(vxF, vyF) - rigidVel;

        // Simple drag: 0.5 * C_d * fluidDensity * v^2 * dt
        float vMag = length(relVel);
        if (vMag > 1e-6f) {
            float dragMag = 0.5f * dragCoefficient * densityF * (vMag*vMag) * dt;
            if (dragMag > maxForce) {
                dragMag = maxForce;
            }
            float2 dragForce = (relVel / vMag) * dragMag;

            // Optional buoyancy: approximate each particle as a small volume
            // volume = mass/density => fluid mass / fluid density => same for this
            // We'll do a small upward force
            float vol = fp.mass / densityF;  // mass / fluidDensity
            float buoyFactor = 0.02f;        // tune as needed
            float buoy = densityF * vol * 9.81f * buoyFactor * dt;
            float2 buoyForce = float2(0.0f, -buoy);

            // sum
            float2 totalForce = dragForce + buoyForce;

            // clamp magnitude
            float fmag = length(totalForce);
            if (fmag > maxForce) {
                totalForce = totalForce * (maxForce / fmag);
            }

            // torque = cross(relativePos, totalForce) in 2D = rx*Fy - ry*Fx
            float torque = rx*totalForce.y - ry*totalForce.x;
            if (torque > maxTorque) {
                torque = maxTorque;
            }
            else if (torque < -maxTorque) {
                torque = -maxTorque;
            }

            // Atomically add to accumFx, accumFy, accumTorque
            // We reinterpret them as atomic_float*
            device atomic_float* aFx = reinterpret_cast<device atomic_float*>(&rigids[r].accumFx);
            device atomic_float* aFy = reinterpret_cast<device atomic_float*>(&rigids[r].accumFy);
            device atomic_float* aTq = reinterpret_cast<device atomic_float*>(&rigids[r].accumTorque);

            atomic_fetch_add_explicit(aFx, totalForce.x, memory_order_relaxed);
            atomic_fetch_add_explicit(aFy, totalForce.y, memory_order_relaxed);
            atomic_fetch_add_explicit(aTq, torque,       memory_order_relaxed);
        }
    }
    // end for r in rigidCount
}