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
 * Kernel: rigidFluidPositionSolver
 * @brief For each fluid thread, we loop over all rigids. If the fluid is near/inside the rigid,
 *        we compute drag & buoyancy, then atomically add to rigids[r].accumFx, accumFy, accumTorque.
 *
 * This is a naive O(N_fluid * N_rigid) approach. For large simulations, use better broad-phase or reduce logic.
 *
 * We incorporate param.dt each sub-step so the total effect accumulates across multiple sub-steps.
 */
kernel void rigidFluidPositionSolver(
    device GPUFluidParticle* particles [[buffer(0)]],
    constant int& fluidCount           [[buffer(1)]],
    device GPURigidBody* rigids        [[buffer(2)]],
    constant int& rigidCount           [[buffer(3)]],
    device const GPUFluidParams& param      [[buffer(4)]],
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
    
    // Velocity reflection parameters - reduced for better stability
    const float restitution = 0.1f;       // Low restitution to prevent bouncing
    const float frictionFactor = 0.5f;    // Higher friction to dissipate energy faster
    const float velocityDamping = 0.95f;  // Additional velocity damping for stability
    
    // Track if a collision occurred and the rigid body data
    bool hadCollision = false;
    float2 rigidVelocity = float2(0.0f, 0.0f);
    float2 collisionNormal = float2(0.0f, 0.0f);

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
                
                // Record collision data
                hadCollision = true;
                collisionNormal = dir;
                rigidVelocity = float2(body.vx, body.vy);
                // Add rotational velocity at the point of contact
                float2 relPos = float2(dx, dy);
                rigidVelocity += float2(-body.omega * relPos.y, body.omega * relPos.x);
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
                
                // Record collision data
                hadCollision = true;
                collisionNormal = dir;
                rigidVelocity = float2(body.vx, body.vy);
                // Add rotational velocity at the point of contact
                float2 relPos = float2(px - body.posX, py - body.posY);
                rigidVelocity += float2(-body.omega * relPos.y, body.omega * relPos.x);
            }
        }
    }

    // Flip sign to push fluid outwards
    p.x -= accumCorr.x;
    p.y -= accumCorr.y;

    // Update fluid velocity if there was a collision
    if (hadCollision) {
        // Safety check for valid collision normal
        float normalLength = length(collisionNormal);
        if (normalLength > 1e-6f) {
            // Ensure normal is normalized
            float2 safeNormal = collisionNormal / normalLength;
            
            float2 fluidVel = float2(p.vx, p.vy);
            float2 relVel = fluidVel - rigidVelocity;
            
            // Decompose into normal and tangential components
            float normalVel = dot(relVel, safeNormal);
            float2 normalComponent = safeNormal * normalVel;
            float2 tangentialComponent = relVel - normalComponent;
            
            // Apply restitution to normal component (bounce)
            // And friction to tangential component (slow down)
            if (normalVel < 0) { // Only reflect if approaching
                // Energy-conservative velocity update
                float2 newVel = rigidVelocity + 
                              (-restitution * normalComponent) +
                              (tangentialComponent * (1.0f - frictionFactor));
                
                // Energy-conservation check
                float oldEnergy = 0.5f * p.mass * dot(fluidVel, fluidVel);
                float newEnergy = 0.5f * p.mass * dot(newVel, newVel);
                
                // If new energy is higher, scale it down to ensure no energy creation
                if (newEnergy > oldEnergy) {
                    float scale = sqrt(oldEnergy / (newEnergy + 1e-6f));
                    newVel = rigidVelocity + (newVel - rigidVelocity) * scale;
                }
                
                // Apply additional velocity damping for stability
                newVel = rigidVelocity + (newVel - rigidVelocity) * velocityDamping;
                
                // Ensure no extreme velocities that could cause instability
                float newVelMag = length(newVel);
                if (newVelMag > 20.0f) { // Lower velocity limit
                    newVel = newVel * (20.0f / newVelMag);
                }
                
                p.vx = newVel.x;
                p.vy = newVel.y;
                
                // Update half-step velocities with proper differential
                // to maintain verlet integration consistency
                p.vxHalf = p.vx - 0.5f * p.ax * (param.dt * 0.5f);
                p.vyHalf = p.vy - 0.5f * p.ay * (param.dt * 0.5f);
            }
        }
    }

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

    // Tunable physics parameters
    const float dt = param.dt;
    
    // Drag coefficients - drag should ALWAYS remove energy
    const float linearDragCoef = 0.01f;     // Increased linear drag for stability
    const float quadraticDragCoef = 0.002f; // Increased quadratic drag
    
    // Buoyancy settings 
    const float buoyancyCoef = 0.005f;     // Reduced buoyancy to prevent excessive bobbing
    const float waterDensity = 1000.0f;    // Reference water density
    
    // Stabilization and safety
    const float maxForce = 0.1f;           // Reduced max force for better stability
    const float maxTorque = 0.02f;         // Reduced max torque
    const float depthScale = 0.05f;        // Increased depth scale for smoother transition
    const float velocitySafetyFactor = 3.0f; // Increased safety at high velocities

    // Get fluid particle data
    GPUFluidParticle fp = fluidParticles[globalID];
    float densityF = max(fp.density, waterDensity); // Use reference density if not computed

    // For each rigid
    for (int r = 0; r < rigidCount; r++) {
        GPURigidBody rb = rigids[r];
        
        // Safety check for extreme velocities
        float rbVelSq = rb.vx*rb.vx + rb.vy*rb.vy + rb.omega*rb.omega;
        if (rbVelSq > 50.0f) { // Reduced threshold for high velocity check
            continue;
        }
        
        // Skip broad-phase check if outside bounding box
        if (fp.x < rb.minX || fp.x > rb.maxX || fp.y < rb.minY || fp.y > rb.maxY) {
            continue;
        }

        // Determine if particle is inside rigid body
        bool inside = false;
        float penetrationDepth = 0.0f;
        float2 relativePos;
        
        if (rb.shapeType == Circle) {
            float rx = fp.x - rb.posX;
            float ry = fp.y - rb.posY;
            float dist2 = rx*rx + ry*ry;
            float radiusSq = rb.radius * rb.radius;
            
            if (dist2 < radiusSq) {
                inside = true;
                float dist = sqrt(dist2);
                if (dist < 1e-6f) dist = 1e-6f;
                penetrationDepth = rb.radius - dist;
                if (penetrationDepth < 0.0f) penetrationDepth = 0.0f;
                relativePos = float2(rx, ry);
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
                if (d < 1e-6f) d = 1e-6f;
                
                penetrationDepth = d;
                if (penetrationDepth < 0.0f) penetrationDepth = 0.0f;
                
                relativePos = float2(fp.x - rb.posX, fp.y - rb.posY);
            }
        }
        
        if (!inside || penetrationDepth < 1e-6f) continue;
        
        // Calculate rigid body velocity at this point
        float2 rotVel = float2(-rb.omega * relativePos.y, rb.omega * relativePos.x);
        float2 rigidVel = float2(rb.vx, rb.vy) + rotVel;
        float2 relVel = float2(fp.vx, fp.vy) - rigidVel;
        float relVelMag = length(relVel);
        
        if (relVelMag < 1e-6f) continue;
        
        // Normalize for direction
        float2 relVelDir = relVel / relVelMag;
        
        // Improved depth transition function with better curve
        float normalizedDepth = penetrationDepth / depthScale;
        float depthFactor = normalizedDepth * normalizedDepth / (1.0f + normalizedDepth * normalizedDepth);
        
        // Velocity safety scaling (stronger reduction at high speeds)
        float velocitySafety = 1.0f / (1.0f + velocitySafetyFactor * relVelMag);
        
        // Calculate drag magnitude - drag MUST OPPOSE relative motion 
        float linearComponent = linearDragCoef * relVelMag;
        float quadraticComponent = quadraticDragCoef * relVelMag * relVelMag;
        float dragMag = (linearComponent + quadraticComponent) * densityF * dt * velocitySafety * depthFactor;
        
        // Buoyancy with depth-dependent transition
        float volume = fp.mass / densityF;
        float buoyancyMag = waterDensity * volume * 9.81f * buoyancyCoef * dt;
        
        // Varying buoyancy based on depth - full buoyancy only when fully submerged
        float buoyancyDepthFactor = min(depthFactor * 1.5f, 1.0f);
        float buoyancy = buoyancyMag * buoyancyDepthFactor;
        
        // CORRECTED SIGN CONVENTION:
        // Drag should ALWAYS oppose motion (negative of relative velocity direction)
        // Buoyancy always points up
        float2 dragForce = relVelDir * min(dragMag, maxForce * 0.8f);
        float2 buoyancyForce = float2(0.0f, -min(buoyancy, maxForce * 0.3f));
        
        // Combined force (drag + buoyancy)
        float2 totalForce = dragForce + buoyancyForce;
        
        // Safety clamp on total force magnitude
        float forceMag = length(totalForce);
        if (forceMag > maxForce) {
            totalForce = totalForce * (maxForce / forceMag);
        }
        
        // Calculate torque with more damping for rotational stability
        float torque = relativePos.x * totalForce.y - relativePos.y * totalForce.x;
        torque = clamp(torque, -maxTorque, maxTorque);
        
        // Add angular damping proportional to angular velocity to prevent excessive spinning
        if (abs(rb.omega) > 0.1f) {
            torque -= 0.01f * sign(rb.omega) * min(abs(rb.omega), 5.0f) * rb.inertia;
        }
        
        // Atomically add to rigid body accumulators
        device atomic_float* aFx = reinterpret_cast<device atomic_float*>(&rigids[r].accumFx);
        device atomic_float* aFy = reinterpret_cast<device atomic_float*>(&rigids[r].accumFy);
        device atomic_float* aTq = reinterpret_cast<device atomic_float*>(&rigids[r].accumTorque);
        
        atomic_fetch_add_explicit(aFx, totalForce.x, memory_order_relaxed);
        atomic_fetch_add_explicit(aFy, totalForce.y, memory_order_relaxed);
        atomic_fetch_add_explicit(aTq, torque, memory_order_relaxed);
    }
}