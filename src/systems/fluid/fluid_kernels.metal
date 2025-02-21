// file: fluid_kernels.metal

#include <metal_stdlib>
using namespace metal;

// Define PI if not already defined.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Maximum number of particles per cell.
constant int MAX_PER_CELL = 64;

//---------------------------------------------------------------------
// Data Structures
//---------------------------------------------------------------------

// Particle structure (must match the CPU side).
struct Particle {
    float x, y;
    float vx, vy;
    float vxHalf, vyHalf;
    float ax, ay;
    float mass, h, c;
    float density, pressure;
};

// GridCell holds an atomic counter and an index list.
// (Note: we never copy a GridCell because its copy constructor is deleted.)
struct GridCell {
    atomic_int count;
    int indices[MAX_PER_CELL];
};

// All fluid-simulation parameters (including particleCount) in one struct.
struct FluidParams {
    float cellSize;
    int   gridDimX;
    int   gridDimY;
    int   gridMinX;
    int   gridMinY;
    float restDensity;
    float stiffness;
    float viscosity;
    float dt;
    float halfDt;
    uint  particleCount;
};

// For the clearGrid kernel.
struct GridParams {
    uint gridCount;
};

//---------------------------------------------------------------------
// Kernel: clearGrid
//---------------------------------------------------------------------
kernel void clearGrid(device GridCell *gridCells        [[ buffer(0) ]],
                      constant GridParams &gridParams   [[ buffer(1) ]],
                      uint tid                        [[ thread_position_in_grid ]])
{
    if (tid < gridParams.gridCount) {
        atomic_store_explicit(&(gridCells[tid].count), 0, memory_order_relaxed);
    }
}

//---------------------------------------------------------------------
// Kernel: assignCells
//---------------------------------------------------------------------
kernel void assignCells(device Particle *particles            [[ buffer(0) ]],
                        device GridCell *gridCells            [[ buffer(1) ]],
                        constant FluidParams &params          [[ buffer(2) ]],
                        uint tid                            [[ thread_position_in_grid ]])
{
    if (tid >= params.particleCount)
        return;

    Particle p = particles[tid];

    // Determine grid cell based on particle position.
    int gx = int(floor(p.x / params.cellSize)) - params.gridMinX;
    int gy = int(floor(p.y / params.cellSize)) - params.gridMinY;

    gx = clamp(gx, 0, params.gridDimX - 1);
    gy = clamp(gy, 0, params.gridDimY - 1);

    int cellIdx = gy * params.gridDimX + gx;

    // Use atomic_fetch_add_explicit on the gridCells array.
    int oldCount = atomic_fetch_add_explicit(&(gridCells[cellIdx].count), 1, memory_order_relaxed);
    if (oldCount < MAX_PER_CELL) {
        gridCells[cellIdx].indices[oldCount] = int(tid);
    }
}

//---------------------------------------------------------------------
// Helper: poly6 coefficient for 2D kernels
//---------------------------------------------------------------------
inline float poly6Coeff2D(float h) {
    float h2 = h * h;
    float h4 = h2 * h2;
    float h8 = h4 * h4;
    return 4.0f / (M_PI * h8);
}

//---------------------------------------------------------------------
// Kernel: computeDensity
// (Compute density for each particle by looping over neighboring cells.)
//---------------------------------------------------------------------
kernel void computeDensity(device Particle *particles         [[ buffer(0) ]],
                           device GridCell *gridCells         [[ buffer(1) ]],
                           constant FluidParams &params       [[ buffer(2) ]],
                           uint tid                         [[ thread_position_in_grid ]])
{
    if (tid >= params.particleCount)
        return;

    Particle pi = particles[tid];
    float hi    = pi.h;
    float h2    = hi * hi;
    float coeff = poly6Coeff2D(hi);

    int gx = int(floor(pi.x / params.cellSize)) - params.gridMinX;
    int gy = int(floor(pi.y / params.cellSize)) - params.gridMinY;
    gx = clamp(gx, 0, params.gridDimX - 1);
    gy = clamp(gy, 0, params.gridDimY - 1);

    float accum = 0.f;

    // Loop over the 3x3 neighboring cells.
    for (int nx = gx - 1; nx <= gx + 1; nx++) {
        if (nx < 0 || nx >= params.gridDimX)
            continue;
        for (int ny = gy - 1; ny <= gy + 1; ny++) {
            if (ny < 0 || ny >= params.gridDimY)
                continue;
            int cellID = ny * params.gridDimX + nx;
            // Directly load the count without copying the GridCell.
            int count = atomic_load_explicit(&(gridCells[cellID].count), memory_order_relaxed);
            for (int c = 0; c < count; c++) {
                int j = gridCells[cellID].indices[c];
                Particle pj = particles[j];
                float dx = pi.x - pj.x;
                float dy = pi.y - pj.y;
                float r2 = dx * dx + dy * dy;
                if (r2 < h2) {
                    float diff = (h2 - r2);
                    float w = coeff * diff * diff * diff;
                    accum += pj.mass * w;
                }
            }
        }
    }

    pi.density = accum;
    particles[tid] = pi;
}

//---------------------------------------------------------------------
// Helpers for force computation: spiky and viscosity coefficients
//---------------------------------------------------------------------
inline float spikyCoeff2D(float h) {
    float h2 = h * h;
    float h4 = h2 * h2;
    float h5 = h4 * h;
    return -30.0f / (M_PI * h5);
}
inline float viscLaplacianCoeff2D(float h) {
    float h2 = h * h;
    float h4 = h2 * h2;
    float h5 = h4 * h;
    return 40.0f / (M_PI * h5);
}

//---------------------------------------------------------------------
// Kernel: computeForces
// (Compute pressure and viscosity forces based on neighbor particle data.)
//---------------------------------------------------------------------
kernel void computeForces(device Particle *particles         [[ buffer(0) ]],
                          device GridCell *gridCells         [[ buffer(1) ]],
                          constant FluidParams &params       [[ buffer(2) ]],
                          uint tid                         [[ thread_position_in_grid ]])
{
    if (tid >= params.particleCount)
        return;

    Particle pi = particles[tid];
    float rhoi = pi.density;
    if (rhoi < 1e-12f) {
        pi.ax = 0.f;
        pi.ay = 0.f;
        particles[tid] = pi;
        return;
    }
    float pressi = params.stiffness * (rhoi - params.restDensity);
    pressi = (pressi > 0.f ? pressi : 0.f);

    int gx = int(floor(pi.x / params.cellSize)) - params.gridMinX;
    int gy = int(floor(pi.y / params.cellSize)) - params.gridMinY;
    gx = clamp(gx, 0, params.gridDimX - 1);
    gy = clamp(gy, 0, params.gridDimY - 1);

    float sumAx = 0.f, sumAy = 0.f;
    for (int nx = gx - 1; nx <= gx + 1; nx++) {
        if (nx < 0 || nx >= params.gridDimX)
            continue;
        for (int ny = gy - 1; ny <= gy + 1; ny++) {
            if (ny < 0 || ny >= params.gridDimY)
                continue;
            int cellID = ny * params.gridDimX + nx;
            int count = atomic_load_explicit(&(gridCells[cellID].count), memory_order_relaxed);
            for (int c = 0; c < count; c++) {
                uint j = gridCells[cellID].indices[c];
                if (j == tid)
                    continue;
                Particle pj = particles[j];
                float rhoj = pj.density;
                if (rhoj < 1e-12f)
                    continue;
                float dx = pi.x - pj.x;
                float dy = pi.y - pj.y;
                float r2 = dx * dx + dy * dy;
                if (r2 < 1e-12f)
                    continue;
                float hj = pj.h;
                float h_ij = 0.5f * (pi.h + hj);
                if (r2 >= (h_ij * h_ij))
                    continue;
                float r = sqrt(r2);
                float pressj = params.stiffness * (pj.density - params.restDensity);
                pressj = (pressj > 0.f ? pressj : 0.f);
                float term = (pressi / (rhoi * rhoi)) + (pressj / (rhoj * rhoj));
                float spikyF = spikyCoeff2D(h_ij);
                float diff = (h_ij - r);
                float w_spiky = spikyF * (diff * diff);
                float rx = dx / r;
                float ry = dy / r;
                float fPress = -pj.mass * term * w_spiky;
                float fx = fPress * rx;
                float fy = fPress * ry;
                float vx_ij = pi.vx - pj.vx;
                float vy_ij = pi.vy - pj.vy;
                float lapF = viscLaplacianCoeff2D(h_ij);
                float w_visc = lapF * diff;
                float fVisc = params.viscosity * pj.mass * (w_visc / rhoj);
                fx -= fVisc * vx_ij;
                fy -= fVisc * vy_ij;
                sumAx += fx;
                sumAy += fy;
            }
        }
    }
    pi.pressure = pressi;
    pi.ax = sumAx;
    pi.ay = sumAy;
    particles[tid] = pi;
}

//---------------------------------------------------------------------
// Kernel: velocityVerletHalf
// (First half update: compute vHalf then update position.)
//---------------------------------------------------------------------
kernel void velocityVerletHalf(device Particle *particles         [[ buffer(0) ]],
                               constant FluidParams &params       [[ buffer(1) ]],
                               uint tid                         [[ thread_position_in_grid ]])
{
    if (tid >= params.particleCount)
        return;

    Particle p = particles[tid];
    
    // Add a constant gravitational acceleration (adjust sign as needed).
    const float GRAVITY = 9.8f; // m/s^2 (assuming positive is downward)
    // Combine SPH acceleration with gravity.
    float totalAy = p.ay + GRAVITY;
    
    // Compute half-step velocity using total acceleration.
    float vxH = p.vx + 0.5f * p.ax * params.dt;
    float vyH = p.vy + 0.5f * totalAy * params.dt;
    
    // Update position using half-step velocity.
    p.x += vxH * params.dt;
    p.y += vyH * params.dt;
    
    p.vxHalf = vxH;
    p.vyHalf = vyH;
    particles[tid] = p;
}

//---------------------------------------------------------------------
// Kernel: velocityVerletFinish
// (Final update: use vHalf to complete the velocity update.)
//---------------------------------------------------------------------
kernel void velocityVerletFinish(device Particle *particles         [[ buffer(0) ]],
                                 constant FluidParams &params       [[ buffer(1) ]],
                                 uint tid                         [[ thread_position_in_grid ]])
{
    if (tid >= params.particleCount)
        return;

    Particle p = particles[tid];
    const float GRAVITY = 9.8f; // m/s^2 (again, adjust sign if needed)
    float totalAy = p.ay + GRAVITY;
    
    // Complete velocity update with the total acceleration.
    p.vx = p.vxHalf + 0.5f * p.ax * params.dt;
    p.vy = p.vyHalf + 0.5f * totalAy * params.dt;
    particles[tid] = p;
}

//---------------------------------------------------------------------
// Kernel: debugSetX
//---------------------------------------------------------------------
kernel void debugSetX(device Particle *particles        [[ buffer(0) ]],
                      constant uint &particleCount      [[ buffer(1) ]],
                      constant float &value             [[ buffer(2) ]],
                      uint tid                          [[ thread_position_in_grid ]])
{
    if(tid >= particleCount)
        return;
    
    Particle p = particles[tid];
    p.x = value;
    particles[tid] = p;
}
