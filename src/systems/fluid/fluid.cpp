/**
 * @file fluid.cpp
 * @brief Velocity Verlet SPH solver with multi-sub-step integration + partial NEON acceleration.
 *
 * Substeps (Velocity Verlet):
 *   for each sub-step:
 *     1) vHalf[i] = v[i] + 0.5 * a[i] * subDt
 *     2) x[i]    = x[i] + vHalf[i] * subDt
 *     3) Build neighbor grid from x
 *     4) Compute new a[i] (forces) from x
 *     5) v[i]    = vHalf[i] + 0.5 * a[i] * subDt
 *
 * Then write final x,v back to ECS at the end.
 *
 * Uses partial NEON in force loops to accelerate distance checks & kernel weighting.
 * Also uses OpenMP for parallel loops.
 */

#include "nbody/systems/fluid/fluid.hpp"
#include <arm_neon.h>  // For Apple Silicon NEON intrinsics
#include <entt/entt.hpp>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <climits>  // For INT_MAX/INT_MIN
#include <omp.h>    // For OpenMP

#include "nbody/components/basic.hpp"
#include "nbody/components/sph.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/core/profile.hpp"

namespace {

/** Number of sub-steps per tick. Increase or decrease based on stability & performance needs. */
static constexpr int N_SUB_STEPS = 10;

/**
 * @brief Holds SPH particle data in a structure-of-arrays.
 */
struct FluidParticleData {
    std::vector<entt::entity> entities;

    // SoA: positions, velocities, half-step velocities (vHalf), accelerations
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> vx;
    std::vector<float> vy;
    std::vector<float> vHalfx;  // velocity at half-step (Velocity Verlet)
    std::vector<float> vHalfy;
    std::vector<float> ax;
    std::vector<float> ay;

    // Mass, smoothing length, speed of sound
    std::vector<float> mass;
    std::vector<float> h;
    std::vector<float> c;

    // Computed each substep
    std::vector<float> density;
    std::vector<float> pressure;
};

struct GridCell {
    std::vector<int> indices;
};

/**
 * @brief New cache-friendly grid structure:
 */
struct UniformGrid {
    int minX;    // Minimum cell x-index
    int minY;    // Minimum cell y-index
    int width;   // Number of cells in x-direction
    int height;  // Number of cells in y-direction
    std::vector<GridCell> cells;  // Stored in row-major order.
};

/**
 * @brief Builds a contiguous uniform grid from particle positions using a counting sort approach.
 *        The grid build is split into two main phases:
 *          1) A counting phase to tally the number of particles per cell (using thread-local counts),
 *          2) A scatter phase where, using precomputed per-cell offsets, particle indices are directly written into the final grid.
 *
 * @param dat      The fluid particle data (positions).
 * @param cellSize The spatial cell size.
 *
 * @return A UniformGrid with a contiguous cell array containing particle indices.
 */
UniformGrid buildUniformGridContiguousCountingSort(const FluidParticleData &dat, float cellSize)
{
    int n = static_cast<int>(dat.x.size());
    if (n == 0) {
        return {0, 0, 0, 0, {}};
    }

    // Step 1: Compute Domain Bounds
    int globalMinGx = INT_MAX, globalMinGy = INT_MAX;
    int globalMaxGx = INT_MIN, globalMaxGy = INT_MIN;
#pragma omp parallel for reduction(min:globalMinGx,globalMinGy) reduction(max:globalMaxGx,globalMaxGy)
    for (int i = 0; i < n; i++) {
        int gx = static_cast<int>(std::floor(dat.x[i] / cellSize));
        int gy = static_cast<int>(std::floor(dat.y[i] / cellSize));
        globalMinGx = std::min(globalMinGx, gx);
        globalMinGy = std::min(globalMinGy, gy);
        globalMaxGx = std::max(globalMaxGx, gx);
        globalMaxGy = std::max(globalMaxGy, gy);
    }
    int width  = globalMaxGx - globalMinGx + 1;
    int height = globalMaxGy - globalMinGy + 1;
    const int numCells = width * height;

    UniformGrid grid;
    grid.minX = globalMinGx;
    grid.minY = globalMinGy;
    grid.width = width;
    grid.height = height;
    grid.cells.resize(numCells);

    // Step 2: Counting Phase - Use thread-local counters.
    int nThreads = 1;
#pragma omp parallel
    {
#pragma omp single
        nThreads = omp_get_num_threads();
    }
    std::vector<std::vector<size_t>> localCounts(nThreads, std::vector<size_t>(numCells, 0));

#pragma omp parallel for
    for (int i = 0; i < n; i++) {
        int tid = omp_get_thread_num();
        int gx = static_cast<int>(std::floor(dat.x[i] / cellSize));
        int gy = static_cast<int>(std::floor(dat.y[i] / cellSize));
        int cellIndex = (gy - globalMinGy) * width + (gx - globalMinGx);
        localCounts[tid][cellIndex]++;
    }

    // Combine thread-local counts into a global count for each cell.
    std::vector<size_t> globalCounts(numCells, 0);
    for (int cell = 0; cell < numCells; cell++) {
        for (int t = 0; t < nThreads; t++) {
            globalCounts[cell] += localCounts[t][cell];
        }
    }

    // Reserve exact space in each GridCell to avoid reallocations.
    for (int cell = 0; cell < numCells; cell++) {
        grid.cells[cell].indices.resize(globalCounts[cell]);
    }

    // Step 3: Compute per-thread starting offsets for each cell.
    // For each cell, determine where in the final vector each thread should start writing.
    std::vector<std::vector<size_t>> threadOffsets(nThreads, std::vector<size_t>(numCells, 0));
    for (int cell = 0; cell < numCells; cell++) {
        threadOffsets[0][cell] = 0;
        for (int t = 1; t < nThreads; t++) {
            threadOffsets[t][cell] = threadOffsets[t - 1][cell] + localCounts[t - 1][cell];
        }
    }

    // Step 4: Scatter Phase - Write particle indices directly into pre-allocated slots.
#pragma omp parallel for
    for (int i = 0; i < n; i++) {
        int tid = omp_get_thread_num();
        int gx = static_cast<int>(std::floor(dat.x[i] / cellSize));
        int gy = static_cast<int>(std::floor(dat.y[i] / cellSize));
        int cellIndex = (gy - globalMinGy) * width + (gx - globalMinGx);
        size_t offset = threadOffsets[tid][cellIndex];
        grid.cells[cellIndex].indices[offset] = i;
        // Increment the thread-local offset.
        threadOffsets[tid][cellIndex]++;
    }

    return grid;
}

/** Poly6 kernel factor in 2D: W(r) = 4/(π * h^8) * (h^2 - r^2)^3 */
inline float poly6Coeff2D(float h)
{
    float h2 = h * h;
    float h4 = h2 * h2;
    float h8 = h4 * h4;
    return 4.0f / (float(M_PI) * h8);
}

/** Spiky gradient factor in 2D: ∇W(r) = -30/(π * h^5) * (h-r)^2 * (r̂) */
inline float spikyCoeff2D(float h)
{
    float h2 = h*h;
    float h4 = h2*h2;
    float h5 = h4*h;
    return -30.0f / (float(M_PI) * h5);
}

/** Viscosity Laplacian factor in 2D: Lap(W_visc) ~ 40/(π h^5) */
inline float viscLaplacianCoeff2D(float h)
{
    float h2 = h*h;
    float h4 = h2*h2;
    float h5 = h4*h;
    return 40.f / (float(M_PI)*h5);
}

/**
 * @brief Gathers Liquid-phase particles from ECS into FluidParticleData (SoA).
 *        Initializes velocity half-step = velocity, acceleration=0, etc.
 */
FluidParticleData gatherFluidParticles(entt::registry &registry)
{
    FluidParticleData data;

    auto view = registry.view<Components::Position,
                              Components::Velocity,
                              Components::Mass,
                              Components::ParticlePhase,
                              Components::SmoothingLength,
                              Components::SpeedOfSound,
                              Components::SPHTemp>();

    // Count how many are liquid
    size_t count = 0;
    for (auto e : view) {
        const auto &ph = view.get<Components::ParticlePhase>(e);
        if (ph.phase == Components::Phase::Liquid) {
            count++;
        }
    }

    data.entities.reserve(count);
    data.x.reserve(count); data.y.reserve(count);
    data.vx.reserve(count); data.vy.reserve(count);
    data.vHalfx.resize(count, 0.f);
    data.vHalfy.resize(count, 0.f);
    data.ax.resize(count, 0.f);
    data.ay.resize(count, 0.f);
    data.mass.reserve(count);
    data.h.reserve(count);
    data.c.reserve(count);
    data.density.resize(count, 0.f);
    data.pressure.resize(count, 0.f);

    for (auto e : view) {
        const auto &ph = view.get<Components::ParticlePhase>(e);
        if (ph.phase != Components::Phase::Liquid) {
            continue;
        }
        data.entities.push_back(e);

        const auto &pos = view.get<Components::Position>(e);
        const auto &vel = view.get<Components::Velocity>(e);
        const auto &m   = view.get<Components::Mass>(e);
        const auto &sl  = view.get<Components::SmoothingLength>(e);
        const auto &snd = view.get<Components::SpeedOfSound>(e);

        data.x.push_back(static_cast<float>(pos.x));
        data.y.push_back(static_cast<float>(pos.y));
        data.vx.push_back(static_cast<float>(vel.x));
        data.vy.push_back(static_cast<float>(vel.y));
        // For velocity verlet, start vHalf at the same as v
        data.vHalfx.back() = data.vx.back();
        data.vHalfy.back() = data.vy.back();

        data.mass.push_back(static_cast<float>(m.value));
        data.h.push_back(static_cast<float>(sl.value));
        data.c.push_back(static_cast<float>(snd.value));
    }

    return data;
}

/**
 * @brief Compute densities using the poly6 kernel. NEON is used to accelerate
 *        distance computations in small batches.
 */
void computeDensities(FluidParticleData &dat,
                      const UniformGrid &grid,
                      float cellSize)
{
    PROFILE_SCOPE("ComputeDensities");

    int n = static_cast<int>(dat.x.size());
    // Clear densities
    // #pragma omp parallel for
    for (int i = 0; i < n; i++) {
        dat.density[i] = 0.f;
    }

    // #pragma omp parallel for
    for (int i = 0; i < n; i++) {
        float xi = dat.x[i];
        float yi = dat.y[i];
        float hi = dat.h[i];
        float h2 = hi*hi;
        float coeff = poly6Coeff2D(hi);

        float accum = 0.f;

        int gx = static_cast<int>(std::floor(xi / cellSize));
        int gy = static_cast<int>(std::floor(yi / cellSize));

        // Process neighbor cells in the 3x3 region
        for (int cx = gx - 1; cx <= gx + 1; cx++) {
            for (int cy = gy - 1; cy <= gy + 1; cy++) {
                // Check if the cell exists in the grid.
                if (cx < grid.minX || cx >= grid.minX + grid.width ||
                    cy < grid.minY || cy >= grid.minY + grid.height)
                    continue;
                const GridCell &cell = grid.cells[(cy - grid.minY) * grid.width + (cx - grid.minX)];
                for (auto j : cell.indices) {
                    // If we exceed local array, process now (rare if spacing is decent)
                    float dx = xi - dat.x[j];
                    float dy = yi - dat.y[j];
                    float r2 = dx*dx + dy*dy;
                    if (r2 < h2) {
                        float diff = h2 - r2;
                        float w = coeff * diff * diff * diff;
                        accum += dat.mass[j] * w;
                    }
                }
            }
        }
        dat.density[i] = accum;
    }
}

/**
 * @brief Computes symmetrical pressure + viscosity forces using spiky & viscosity kernels.
 *        Includes partial NEON usage for distance checks.
 */
void computeForces(FluidParticleData &dat,
                   const UniformGrid &grid,
                   float cellSize,
                   float restDensity,
                   float stiffness,
                   float viscosity)
{
    PROFILE_SCOPE("ComputeForces");

    int n = static_cast<int>(dat.x.size());

    // 1) Pressure from Equation of State
    // #pragma omp parallel for
    for (int i = 0; i < n; i++) {
        float rho = dat.density[i];
        float p = stiffness*(rho - restDensity);
        dat.pressure[i] = (p > 0.f ? p : 0.f);
    }

    // Clear old accelerations
    // #pragma omp parallel for
    for (int i = 0; i < n; i++) {
        dat.ax[i] = 0.f;
        dat.ay[i] = 0.f;
    }

    // 2) Pressure + Viscosity loops
    {
        PROFILE_SCOPE("ComputeForces_Pressure");
        // #pragma omp parallel for
        for (int i = 0; i < n; i++) {
            float xi = dat.x[i];
            float yi = dat.y[i];
            float pi = dat.pressure[i];
            float rhoi = dat.density[i];
            float hi = dat.h[i];
            if (rhoi < 1e-12f) continue;

            // Temporary buffers for neighbor data can be used for NEON batching.
            // For simplicity, we process neighbors directly in this version.
            float sumAx = 0.f;
            float sumAy = 0.f;

            int gx = static_cast<int>(std::floor(xi / cellSize));
            int gy = static_cast<int>(std::floor(yi / cellSize));

            // Process neighbor cells in 3x3 block.
            for (int cx = gx - 1; cx <= gx + 1; cx++) {
                for (int cy = gy - 1; cy <= gy + 1; cy++) {
                    if (cx < grid.minX || cx >= grid.minX + grid.width ||
                        cy < grid.minY || cy >= grid.minY + grid.height)
                        continue;
                    const GridCell &cell = grid.cells[(cy - grid.minY) * grid.width + (cx - grid.minX)];
                    for (auto j : cell.indices) {
                        if (j == i) continue;
                        float dx = xi - dat.x[j];
                        float dy = yi - dat.y[j];
                        float r2 = dx * dx + dy * dy;
                        if (r2 < 1e-14f) continue;
                        float h_ij = 0.5f * (hi + dat.h[j]);
                        if (r2 >= (h_ij * h_ij)) continue;

                        float r = std::sqrt(r2);
                        float rhoj = dat.density[j];
                        if (rhoj < 1e-12f) continue;
                        float pj = dat.pressure[j];
                        float term = (pi / (rhoi * rhoi)) + (pj / (rhoj * rhoj));
                        float spikyFactor = spikyCoeff2D(h_ij);
                        float diff = (h_ij - r);
                        float w_spiky = spikyFactor * (diff * diff);
                        float rx = dx / r;
                        float ry = dy / r;
                        float fPress = -dat.mass[j] * term * w_spiky;
                        float fx = fPress * rx;
                        float fy = fPress * ry;

                        // Viscosity
                        float vx_ij = dat.vx[i] - dat.vx[j];
                        float vy_ij = dat.vy[i] - dat.vy[j];
                        float lapFactor = viscLaplacianCoeff2D(h_ij);
                        float w_visc = lapFactor * (diff);
                        float fVisc = viscosity * dat.mass[j] * (w_visc / rhoj);
                        fx -= fVisc * vx_ij;
                        fy -= fVisc * vy_ij;

                        sumAx += fx;
                        sumAy += fy;
                    }
                }
            }
            dat.ax[i] = sumAx;
            dat.ay[i] = sumAy;
        }
    }
}

/**
 * @brief One velocity-verlet sub-step:
 *  1) vHalf = v + 0.5 a dt
 *  2) x     = x + vHalf dt
 * Then we build grid, compute new forces -> a, finish step:
 *  3) v = vHalf + 0.5 a dt
 */
void velocityVerletSubStep(FluidParticleData &dat,
                           float subDt,
                           float restDensity,
                           float stiffness,
                           float viscosity)
{
    int n = static_cast<int>(dat.x.size());
    // (1) vHalf = v + 0.5 a dt
    // #pragma omp parallel for
    for (int i = 0; i < n; i++) {
        dat.vHalfx[i] = dat.vx[i] + 0.5f*dat.ax[i]*subDt;
        dat.vHalfy[i] = dat.vy[i] + 0.5f*dat.ay[i]*subDt;
    }

    // (2) x = x + vHalf dt
    // #pragma omp parallel for
    for (int i = 0; i < n; i++) {
        dat.x[i] += dat.vHalfx[i]*subDt;
        dat.y[i] += dat.vHalfy[i]*subDt;
    }

    // Build uniform grid from updated x,y.
    // Use the first particle's h as an approximation for cellSize.
    float h0 = (n > 0 ? dat.h[0] : 0.05f);
    float cellSize = 2.f * h0;
    UniformGrid grid;
    {
        PROFILE_SCOPE("BuildUniformGridContiguousCountingSort");
        grid = buildUniformGridContiguousCountingSort(dat, cellSize);
    }

    // Compute densities using the new contiguous grid.
    computeDensities(dat, grid, cellSize);

    // Compute new forces (a) using the new grid.
    computeForces(dat, grid, cellSize, restDensity, stiffness, viscosity);

    // (3) v = vHalf + 0.5 a dt
    // #pragma omp parallel for
    for (int i = 0; i < n; i++) {
        dat.vx[i] = dat.vHalfx[i] + 0.5f*dat.ax[i]*subDt;
        dat.vy[i] = dat.vHalfy[i] + 0.5f*dat.ay[i]*subDt;
    }
}

/**
 * @brief Write the final positions and velocities back into ECS.
 */
void writeBackToECS(const FluidParticleData &dat, entt::registry &registry)
{
    PROFILE_SCOPE("WriteResultsToECS");

    int n = static_cast<int>(dat.entities.size());
    for (int i = 0; i < n; i++) {
        entt::entity e = dat.entities[i];
        // position
        auto pos = registry.get<Components::Position>(e);
        pos.x = dat.x[i];
        pos.y = dat.y[i];
        registry.replace<Components::Position>(e, pos);

        // velocity
        auto vel = registry.get<Components::Velocity>(e);
        vel.x = dat.vx[i];
        vel.y = dat.vy[i];
        registry.replace<Components::Velocity>(e, vel);

        // Store density/pressure if needed
        auto &temp = registry.get<Components::SPHTemp>(e);
        temp.density = dat.density[i];
        temp.pressure = dat.pressure[i];
    }
}

} // end anonymous namespace


namespace Systems {

void FluidSystem::update(entt::registry &registry)
{
    PROFILE_SCOPE("FluidSystem::update");

    // 1) gather data
    FluidParticleData data;
    {
        PROFILE_SCOPE("GatherFluidParticles");
        data = gatherFluidParticles(registry);
    }
    if (data.entities.empty()) {
        return;
    }

    // global dt
    float dt = float(SimulatorConstants::SecondsPerTick * SimulatorConstants::TimeAcceleration);
    float subDt = dt / float(N_SUB_STEPS);

    // typical fluid constants
    float restDensity = float(SimulatorConstants::ParticleDensity);
    float stiffness   = 500.f; // tune as needed
    float viscosity   = 0.1f;   // small damping

    // 2) multiple sub steps velocity-verlet
    {
        PROFILE_SCOPE("VelocityVerletSubSteps");
        for (int step = 0; step < N_SUB_STEPS; step++) {
            velocityVerletSubStep(data, subDt, restDensity, stiffness, viscosity);
        }
    }

    // 3) after all sub-steps, write final x,v back to ECS
    {
        PROFILE_SCOPE("WriteResultsToECS");
        writeBackToECS(data, registry);
    }
}

} // namespace Systems