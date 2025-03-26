/**
 * @fileoverview fluid_kernels.metal
 * @brief 2D SPH solver kernels with GPU-based bounding box,
 *        GPU-based rigid-fluid position solver, and
 *        a new GPU-based rigid-fluid impulse solver for drag/buoyancy.
 */

#include <metal_stdlib>
#include <metal_atomic>
#include "systems/fluid/fluid_kernels.h"

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
 * are stored as normal floats; we'll do atomic ops on them via reinterpret_cast.
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

    float px = part.x + p.gridConfig.gridEpsilon;
    float py = part.y + p.gridConfig.gridEpsilon;
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
    float hi = (self.h <= 0.f) ? p.gridConfig.defaultSmoothingLength : self.h;
    float h2 = hi*hi;
    float poly6 = poly6Coeff2D(hi);

    float accumDensity = 0.0f;
    float px = xi + p.gridConfig.gridEpsilon;
    float py = yi + p.gridConfig.gridEpsilon;
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
    float hi = (self.h<=0.f) ? pa.gridConfig.defaultSmoothingLength : self.h;

    float sumFx = 0.f;
    float sumFy = 0.f;

    float px = xi + pa.gridConfig.gridEpsilon;
    float py = yi + pa.gridConfig.gridEpsilon;
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
                if (r2 < pa.numericalConfig.minDistanceThreshold) {
                    continue;
                }
                float hj = (nbr.h<=0.f) ? pa.gridConfig.defaultSmoothingLength : nbr.h;
                float h_ij = 0.5f*(hi + hj);
                float h_ij2 = h_ij*h_ij;
                if (r2 >= h_ij2) {
                    continue;
                }
                float r = sqrt(r2);
                float pj = nbr.pressure;
                float rhoj = nbr.density;
                if (rhoj < pa.numericalConfig.minDensityThreshold || 
                    rhoi < pa.numericalConfig.minDensityThreshold) {
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
 * @brief Resolves collisions between fluid particles and rigid bodies using Position-Based Dynamics
 * 
 * For each fluid particle, this kernel:
 * 1. Detects overlaps with rigid bodies (circles and polygons)
 * 2. Applies position corrections to move particles outside rigid bodies
 * 3. Updates velocities based on position changes using PBD principles
 *
 * Uses a direction-aware velocity update to prevent artificial bouncing while
 * maintaining physical consistency between positions and velocities.
 * 
 * @param particles Fluid particle array to process
 * @param fluidCount Number of fluid particles
 * @param rigids Array of rigid bodies to check against
 * @param rigidCount Number of rigid bodies
 * @param params Fluid simulation parameters (for timestep)
 */
kernel void rigidFluidPositionSolver(
    device GPUFluidParticle* particles [[buffer(0)]],
    constant int& fluidCount           [[buffer(1)]],
    device GPURigidBody* rigids        [[buffer(2)]],
    constant int& rigidCount           [[buffer(3)]],
    device const GPUFluidParams& params [[buffer(4)]],
    uint globalID                      [[thread_position_in_grid]])
{
    if (globalID >= (uint)fluidCount) {
        return;
    }

    // -----------------------------------------------------------------------
    // Position solver parameters - using configuration passed from CPU
    // -----------------------------------------------------------------------
    // Collision resolution parameters
    const float SAFETY_MARGIN = params.positionSolver.safetyMargin;    // Extra separation to prevent re-collision
    const float RELAX_FACTOR = params.positionSolver.relaxFactor;      // Lower factor for more gradual corrections
    
    // Numerical stability thresholds
    const float MIN_SAFE_DISTANCE = params.positionSolver.minSafeDistance; // Minimum safe distance to avoid divide-by-zero
    const float MIN_POSITION_CHANGE = params.positionSolver.minPositionChange; // Minimum position change to consider
    const float VELOCITY_DAMPING = params.positionSolver.velocityDamping; // Damping to prevent bouncing
    const float MAX_VELOCITY_UPDATE = params.positionSolver.maxVelocityUpdate; // Clamp maximum velocity change
    
    // Get time step
    float dt = params.dt;
    if (dt < params.numericalConfig.minTimestep) dt = params.numericalConfig.fallbackTimestep;
    
    // Local variables for the current particle
    GPUFluidParticle p = particles[globalID];
    float2 oldPos = float2(p.x, p.y);            // Save initial position for PBD
    float2 accumCorr = float2(0.0f, 0.0f);
    float px = p.x;
    float py = p.y;

    // Track if we had any collisions
    bool hadCollision = false;

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
                hadCollision = true;
                float dist = sqrt(dist2);
                if (dist < MIN_SAFE_DISTANCE) {
                    dist = MIN_SAFE_DISTANCE;
                    dx = 1.0f; dy = 0.0f;
                }
                float pen = (radius - dist) + SAFETY_MARGIN;
                float2 dir = float2(dx, dy) / dist;
                float2 corr = dir * pen * RELAX_FACTOR;
                accumCorr += corr;
            }
        }
        else if (body.shapeType == Polygon) {
            if (body.vertCount < 3) {
                continue;
            }
            bool inside = pointInPolygon(px, py, body);
            if (inside) {
                hadCollision = true;
                float cx, cy;
                closestPointOnPolygon(px, py, body, cx, cy);
                float cdx = px - cx;
                float cdy = py - cy;
                float d2 = cdx*cdx + cdy*cdy;
                float d = sqrt(d2);
                if (d < MIN_SAFE_DISTANCE) {
                    d = MIN_SAFE_DISTANCE;
                    cdx = 1.0f; cdy = 0.0f;
                }
                float pen = d + SAFETY_MARGIN;
                float2 dir = float2(cdx, cdy) / d;
                float2 corr = dir * pen * RELAX_FACTOR;
                accumCorr += corr;
            }
        }
    }

    // Apply position correction
    // Add a safety limit to prevent extreme position corrections
    const float MAX_CORRECTION = params.positionSolver.maxCorrection; // Maximum correction per frame
    float corrMagnitude = length(accumCorr);
    if (corrMagnitude > MAX_CORRECTION) {
        accumCorr = (accumCorr / corrMagnitude) * MAX_CORRECTION;
    }
    
    p.x -= accumCorr.x;
    p.y -= accumCorr.y;

    // Simple bounds clamp - use configurable boundary offset
    if (p.x < 0.f) p.x = params.gridConfig.boundaryOffset;
    if (p.y < 0.f) p.y = params.gridConfig.boundaryOffset;
    
    // PBD: Update velocities based on position change only if we had collisions
    if (hadCollision) {
        float2 newPos = float2(p.x, p.y);
        float2 posDelta = newPos - oldPos;
        float posDeltaMag = length(posDelta);
        
        if (posDeltaMag > MIN_POSITION_CHANGE) {
            // Derive new velocity from position change
            float2 derivedVel = posDelta / dt;
            
            // Clamp maximum velocity changes
            float derivedVelMag = length(derivedVel);
            if (derivedVelMag > MAX_VELOCITY_UPDATE) {
                derivedVel = derivedVel * (MAX_VELOCITY_UPDATE / derivedVelMag);
            }
            
            // Get current velocity direction
            float2 curVel = float2(p.vx, p.vy);
            float curVelMag = length(curVel);
            
            // For collision response, prioritize slowing particles down over speeding them up
            // This reduces the bouncing effect while still preventing penetration
            if (curVelMag > 0.001f) {
                float2 curVelDir = curVel / curVelMag;
                float dotProd = dot(curVelDir, derivedVel);
                
                // If our velocity is being opposed (going into rigid body)
                if (dotProd < 0.0f) {
                    // Apply stronger damping to opposing velocity
                    p.vx = mix(p.vx, derivedVel.x, VELOCITY_DAMPING * 1.5f);
                    p.vy = mix(p.vy, derivedVel.y, VELOCITY_DAMPING * 1.5f);
                } else {
                    // Otherwise just apply normal damping
                    p.vx = mix(p.vx, derivedVel.x, VELOCITY_DAMPING);
                    p.vy = mix(p.vy, derivedVel.y, VELOCITY_DAMPING);
                }
            } else {
                // If almost no velocity, apply minimal damping to avoid artificial energy
                p.vx = mix(p.vx, derivedVel.x, VELOCITY_DAMPING * 0.5f);
                p.vy = mix(p.vy, derivedVel.y, VELOCITY_DAMPING * 0.5f);
            }
            
            // Update half-step velocity for Verlet integration consistency
            p.vxHalf = p.vx;
            p.vyHalf = p.vy;
        }
    }

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

    // -----------------------------------------------------------------------
    // Impulse solver parameters - using configuration passed from CPU
    // -----------------------------------------------------------------------
    // Physics constants 
    const float GRAVITY = param.gravity; // Now configured from CPU
    const float WATER_DENSITY = param.restDensity; // Use the configured rest density
    
    // Force limits for stability
    const float MAX_FORCE = param.impulseSolver.maxForce;
    const float MAX_TORQUE = param.impulseSolver.maxTorque;
    
    // Force calculation parameters
    const float VISCOSITY_SCALE = param.impulseSolver.viscosityScale;
    const float DEPTH_SCALE = param.impulseSolver.depthScale;
    const float DEPTH_TRANSITION_RATE = param.impulseSolver.depthTransitionRate;
    const float PRESSURE_FORCE_LIMIT_RATIO = param.impulseSolver.pressureForceRatio;
    const float VISCOUS_FORCE_LIMIT_RATIO = param.impulseSolver.viscousForceRatio;
    
    // Angular damping
    const float ANGULAR_DAMPING_THRESHOLD = param.impulseSolver.angularDampingThreshold;
    const float ANGULAR_DAMPING_FACTOR = param.impulseSolver.angularDampingFactor;
    
    // Hydrostatic pressure estimation
    const float DEPTH_ESTIMATE_SCALE = param.impulseSolver.depthEstimateScale;
    
    // Safety thresholds
    const float MAX_SAFE_VELOCITY_SQ = param.impulseSolver.maxSafeVelocitySq;
    const float MIN_PENETRATION = param.impulseSolver.minPenetration;
    const float MIN_REL_VELOCITY = param.impulseSolver.minRelVelocity;
    
    // Two-way coupling parameters
    const float FLUID_FORCE_SCALE = param.impulseSolver.fluidForceScale;
    const float FLUID_FORCE_MAX = param.impulseSolver.fluidForceMax;
    const float BUOYANCY_STRENGTH = param.impulseSolver.buoyancyStrength;

    // Get time step from parameters
    const float dt = param.dt;
    
    // Get fluid particle data
    GPUFluidParticle fp = fluidParticles[globalID];
    float densityF = fp.density > 0.0f ? fp.density : WATER_DENSITY;
    float pressureF = fp.pressure;
    
    // Accumulators for forces on this fluid particle (two-way coupling)
    float2 totalFluidForce = float2(0.0f, 0.0f);
    bool hadInteraction = false;

    // For each rigid
    for (int r = 0; r < rigidCount; r++) {
        GPURigidBody rb = rigids[r];
        
        // Safety velocity check
        float rbVelSq = rb.vx*rb.vx + rb.vy*rb.vy + rb.omega*rb.omega;
        if (rbVelSq > MAX_SAFE_VELOCITY_SQ) {
            continue;
        }
        
        // Skip broad-phase check
        if (fp.x < rb.minX || fp.x > rb.maxX || fp.y < rb.minY || fp.y > rb.maxY) {
            continue;
        }

        // Check if particle is inside rigid body
        bool inside = false;
        float penetrationDepth = 0.0f;
        float2 relativePos;
        float2 normal; // Surface normal (points from rigid to fluid)
        
        if (rb.shapeType == Circle) {
            float rx = fp.x - rb.posX;
            float ry = fp.y - rb.posY;
            float dist2 = rx*rx + ry*ry;
            float radiusSq = rb.radius * rb.radius;
            
            if (dist2 < radiusSq) {
                inside = true;
                float dist = sqrt(dist2);
                if (dist < MIN_PENETRATION) dist = MIN_PENETRATION;
                penetrationDepth = rb.radius - dist;
                if (penetrationDepth < 0.0f) penetrationDepth = 0.0f;
                relativePos = float2(rx, ry);
                
                // Normal points from rigid to fluid (outward from circle center)
                normal = relativePos / dist;
            }
        }
        else if (rb.shapeType == Polygon && rb.vertCount >= 3) {
            inside = pointInPolygon(fp.x, fp.y, rb);
            if (inside) {
                float cx, cy;
                closestPointOnPolygon(fp.x, fp.y, rb, cx, cy);
                float dx = fp.x - cx;
                float dy = fp.y - cy;
                float d2 = dx*dx + dy*dy;
                float d = sqrt(d2);
                if (d < MIN_PENETRATION) d = MIN_PENETRATION;
                
                penetrationDepth = d;
                if (penetrationDepth < 0.0f) penetrationDepth = 0.0f;
                relativePos = float2(fp.x - rb.posX, fp.y - rb.posY);
                
                // Normal points from rigid (closest point) to fluid
                normal = float2(dx, dy) / d;
            }
        }
        
        if (!inside || penetrationDepth < MIN_PENETRATION) continue;
        
        hadInteraction = true;
        
        // Calculate rigid body velocity at this point
        float2 rotVel = float2(-rb.omega * relativePos.y, rb.omega * relativePos.x);
        float2 rigidVel = float2(rb.vx, rb.vy) + rotVel;
        float2 relVel = float2(fp.vx, fp.vy) - rigidVel;
        
        // Skip if minimal relative velocity
        float relVelMag = length(relVel);
        
        // Depth factor for smooth transition
        float depthFactor = tanh(DEPTH_TRANSITION_RATE * penetrationDepth / DEPTH_SCALE);
        
        // Decompose relative velocity into normal and tangential components
        float normalVel = dot(relVel, normal);
        float2 normalVelVec = normal * normalVel;
        float2 tangentVelVec = relVel - normalVelVec;
        
        // ---------------------------------------------------------------------
        // 1. PRESSURE FORCE: Based on fluid pressure and hydrostatic effects
        // ---------------------------------------------------------------------
        // Calculate effective area for this particle's interaction
        float particleVolume = fp.mass / densityF;
        float effectiveArea = pow(particleVolume, 2.0f/3.0f); // Approximate area from volume
        
        // Consider both dynamic pressure from SPH and hydrostatic pressure
        // The hydrostatic component naturally creates buoyancy
        float depth = min(fp.y / DEPTH_ESTIMATE_SCALE, 1.0f); // Estimate depth from top (y=0)
        float hydrostaticComponent = densityF * GRAVITY * depth;
        float totalPressure = pressureF + hydrostaticComponent;
        
        // Scale by depth factor and area
        float pressureForce = totalPressure * effectiveArea * depthFactor;
        
        // Apply pressure force in normal direction (create floating effect)
        float2 pressureForceVec = normal * min(pressureForce, MAX_FORCE * PRESSURE_FORCE_LIMIT_RATIO);
        
        // ---------------------------------------------------------------------
        // 2. VISCOUS FORCE: Apply to tangential direction for fluid friction
        // ---------------------------------------------------------------------
        float tangentVelMag = length(tangentVelVec);
        if (tangentVelMag > MIN_REL_VELOCITY) {
            float2 tangentDir = tangentVelVec / tangentVelMag;
            // Viscous force opposes tangential motion
            float viscosityCoef = param.viscosity * VISCOSITY_SCALE;
            float viscousForce = viscosityCoef * tangentVelMag * densityF * depthFactor * dt;
            float2 viscousForceVec = -tangentDir * min(viscousForce, MAX_FORCE * VISCOUS_FORCE_LIMIT_RATIO);
            
            // Add to total force
            pressureForceVec += viscousForceVec;
        }
        
        // ---------------------------------------------------------------------
        // 3. BUOYANCY EFFECT: Enhanced upward force for realistic floating
        // ---------------------------------------------------------------------
        // Add stronger buoyancy effect if object is denser than water
        if (rb.mass > 0.1f) {  // Only for non-trivial mass objects
            // Calculate buoyancy based on depth and relative density
            float2 buoyancyForce = float2(0.0f, -1.0f) * // Upward direction
                                 BUOYANCY_STRENGTH * // Strength multiplier
                                 penetrationDepth * // More force for deeper penetration
                                 effectiveArea * // Scale by interaction area
                                 GRAVITY * // Scale by gravity
                                 densityF; // Scale by fluid density
            
            // Add to total force (only apply buoyancy if it doesn't exceed max force)
            float2 combinedForce = pressureForceVec + buoyancyForce;
            if (length(combinedForce) <= MAX_FORCE) {
                pressureForceVec = combinedForce;
            }
        }
        
        // ---------------------------------------------------------------------
        // 4. COMBINE FORCES with safety clamping
        // ---------------------------------------------------------------------
        float2 totalForce = pressureForceVec;
        
        // Safety clamp
        float forceMag = length(totalForce);
        if (forceMag > MAX_FORCE) {
            totalForce = totalForce * (MAX_FORCE / forceMag);
        }
        
        // Calculate torque
        float torque = relativePos.x * totalForce.y - relativePos.y * totalForce.x;
        torque = clamp(torque, -MAX_TORQUE, MAX_TORQUE);
        
        // Minimal angular damping only at high angular velocities
        if (abs(rb.omega) > ANGULAR_DAMPING_THRESHOLD) {
            torque -= ANGULAR_DAMPING_FACTOR * sign(rb.omega) * abs(rb.omega) * rb.inertia;
        }
        
        // Apply forces to rigid body
        device atomic_float* aFx = reinterpret_cast<device atomic_float*>(&rigids[r].accumFx);
        device atomic_float* aFy = reinterpret_cast<device atomic_float*>(&rigids[r].accumFy);
        device atomic_float* aTq = reinterpret_cast<device atomic_float*>(&rigids[r].accumTorque);
        
        atomic_fetch_add_explicit(aFx, totalForce.x, memory_order_relaxed);
        atomic_fetch_add_explicit(aFy, totalForce.y, memory_order_relaxed);
        atomic_fetch_add_explicit(aTq, torque, memory_order_relaxed);
        
        // TWO-WAY COUPLING: Apply equal and opposite force to fluid particle
        // Newton's third law - forces are equal and opposite
        totalFluidForce -= totalForce * FLUID_FORCE_SCALE;
    }
    
    // Apply accumulated forces to fluid particle if we had any interactions
    if (hadInteraction) {
        // Safety clamp forces for stability
        float fluidForceMag = length(totalFluidForce);
        if (fluidForceMag > FLUID_FORCE_MAX) {
            totalFluidForce = totalFluidForce * (FLUID_FORCE_MAX / fluidForceMag);
        }
        
        // Convert force to acceleration (F = ma => a = F/m)
        float invMass = (fp.mass > 0.0001f) ? 1.0f / fp.mass : 1.0f;
        float2 accel = totalFluidForce * invMass;
        
        // Update accelerations 
        fp.ax += accel.x;
        fp.ay += accel.y;
        
        // Store the updated particle
        fluidParticles[globalID] = fp;
    }
}