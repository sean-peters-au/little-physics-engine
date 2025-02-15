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
#include <cassert>  // For assert()

#include "nbody/components/basic.hpp"
#include "nbody/components/sph.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/core/profile.hpp"
#include "nbody/systems/fluid/aligned_allocator.hpp"

namespace {

/** Number of sub-steps per tick. Increase or decrease based on stability & performance needs. */
static constexpr int N_SUB_STEPS = 10;

/**
 * @brief Holds SPH particle data in a structure-of-arrays.
 */
struct FluidParticleData {
    std::vector<entt::entity> entities;

    // SoA: positions, velocities, half-step velocities (vHalf), accelerations
    std::vector<float, AlignedAllocator<float, 32>> x;
    std::vector<float, AlignedAllocator<float, 32>> y;
    std::vector<float, AlignedAllocator<float, 32>> vx;
    std::vector<float, AlignedAllocator<float, 32>> vy;
    std::vector<float, AlignedAllocator<float, 32>> vHalfx;  // velocity at half-step (Velocity Verlet)
    std::vector<float, AlignedAllocator<float, 32>> vHalfy;
    std::vector<float, AlignedAllocator<float, 32>> ax;
    std::vector<float, AlignedAllocator<float, 32>> ay;

    // Mass, smoothing length, speed of sound
    std::vector<float, AlignedAllocator<float, 32>> mass;
    std::vector<float, AlignedAllocator<float, 32>> h;
    std::vector<float, AlignedAllocator<float, 32>> c;

    // Computed each substep
    std::vector<float, AlignedAllocator<float, 32>> density;
    std::vector<float, AlignedAllocator<float, 32>> pressure;

    // For lazy grid updates, store the positions from the last grid build.
    std::vector<float, AlignedAllocator<float, 32>> lastGridX;
    std::vector<float, AlignedAllocator<float, 32>> lastGridY;
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
 * @brief Builds a contiguous uniform grid from particle positions using a thread‑local counts
 *        and offsets approach. This version separates counting and scattering entirely so that
 *        no two threads write concurrently to the same grid cell.
 *
 * Steps:
 *   1. Compute global domain bounds in double precision.
 *   2. Each thread tallies counts into a thread‑local vector (size numCells).
 *   3. Serial aggregation: sum per‑cell counts, and allocate each cell's indices vector.
 *   4. Compute per‑cell thread‑local offsets via prefix sum over threads.
 *   5. In a parallel scatter phase, each thread writes its particle indices into its designated
 *      range in the appropriate grid cell.
 *   6. Finally, sort each cell's indices to enforce deterministic ordering.
 *
 * Note: The particle data in dat must not be modified concurrently!
 *
 * @param dat      The fluid particle data (positions, etc.).
 * @param cellSize The spatial cell size; must be > 0.
 *
 * @return A UniformGrid with a cell array containing particle indices.
 */
namespace {
    constexpr double gridEpsilon = 1e-6;  // Adjust if necessary for stability.
}

UniformGrid buildUniformGridContiguousDomainPartition(const FluidParticleData &dat, float cellSize)
{
    // Ensure valid cellSize.
    assert(cellSize > 0.f && "cellSize must be > 0");

    int n = static_cast<int>(dat.x.size());
    if (n == 0) {
        return {0, 0, 0, 0, {}};
    }

    // ------------------------------------------------------------------
    // Step 1: Compute global domain bounds (grid indices) using double precision.
    // ------------------------------------------------------------------
    int globalMinGx = INT_MAX, globalMinGy = INT_MAX;
    int globalMaxGx = INT_MIN, globalMaxGy = INT_MIN;
#pragma omp parallel for reduction(min: globalMinGx,globalMinGy) reduction(max: globalMaxGx,globalMaxGy)
    for (int i = 0; i < n; i++) {
        double posX = static_cast<double>(dat.x[i]) + gridEpsilon;
        double posY = static_cast<double>(dat.y[i]) + gridEpsilon;
        int gx = static_cast<int>(std::floor(posX / static_cast<double>(cellSize)));
        int gy = static_cast<int>(std::floor(posY / static_cast<double>(cellSize)));
        globalMinGx = std::min(globalMinGx, gx);
        globalMinGy = std::min(globalMinGy, gy);
        globalMaxGx = std::max(globalMaxGx, gx);
        globalMaxGy = std::max(globalMaxGy, gy);
    }
    int width  = globalMaxGx - globalMinGx + 1;
    int height = globalMaxGy - globalMinGy + 1;
    int numCells = width * height;

    UniformGrid grid;
    grid.minX  = globalMinGx;
    grid.minY  = globalMinGy;
    grid.width = width;
    grid.height = height;
    grid.cells.resize(numCells);

    // ------------------------------------------------------------------
    // Step 2: Allocate thread-local counts.
    // Each thread gets a vector (size = numCells) initialized to 0.
    // ------------------------------------------------------------------
    int nThreads = omp_get_max_threads();
    std::vector<std::vector<size_t>> threadCounts(nThreads, std::vector<size_t>(numCells, 0));

#pragma omp parallel
    {
        int tid = omp_get_thread_num();
#pragma omp for schedule(static)
        for (int i = 0; i < n; i++) {
            double posX = static_cast<double>(dat.x[i]) + gridEpsilon;
            double posY = static_cast<double>(dat.y[i]) + gridEpsilon;
            int gx = static_cast<int>(std::floor(posX / static_cast<double>(cellSize)));
            int gy = static_cast<int>(std::floor(posY / static_cast<double>(cellSize)));
            int cellIndex = (gy - globalMinGy) * width + (gx - globalMinGx);
            assert(cellIndex >= 0 && cellIndex < numCells);
            threadCounts[tid][cellIndex]++;
        }
    }

    // ------------------------------------------------------------------
    // Step 3: Aggregate global counts from all thread-local counts.
    // ------------------------------------------------------------------
    std::vector<size_t> globalCounts(numCells, 0);
    for (int cell = 0; cell < numCells; cell++) {
        for (int t = 0; t < nThreads; t++) {
            globalCounts[cell] += threadCounts[t][cell];
        }
    }

    // ------------------------------------------------------------------
    // Step 4: Allocate exact space for each grid cell's indices.
    // ------------------------------------------------------------------
    for (int cell = 0; cell < numCells; cell++) {
        grid.cells[cell].indices.resize(globalCounts[cell]);
    }

    // ------------------------------------------------------------------
    // Step 5: Compute thread-local offsets for each cell via prefix-sum over threads.
    // threadOffsets[t][cell] is the starting index in cell 'cell' for thread t.
    // ------------------------------------------------------------------
    std::vector<std::vector<size_t>> threadOffsets(nThreads, std::vector<size_t>(numCells, 0));
    for (int cell = 0; cell < numCells; cell++) {
        size_t offset = 0;
        for (int t = 0; t < nThreads; t++) {
            threadOffsets[t][cell] = offset;
            offset += threadCounts[t][cell];
        }
    }

    // ------------------------------------------------------------------
    // Step 6: Scatter phase.
    // Each thread writes its particle indices into the preallocated cells using its thread-local offset.
    // ------------------------------------------------------------------
#pragma omp parallel
    {
        int tid = omp_get_thread_num();
#pragma omp for schedule(static)
        for (int i = 0; i < n; i++) {
            double posX = static_cast<double>(dat.x[i]) + gridEpsilon;
            double posY = static_cast<double>(dat.y[i]) + gridEpsilon;
            int gx = static_cast<int>(std::floor(posX / static_cast<double>(cellSize)));
            int gy = static_cast<int>(std::floor(posY / static_cast<double>(cellSize)));
            int cellIndex = (gy - globalMinGy) * width + (gx - globalMinGx);
            assert(cellIndex >= 0 && cellIndex < numCells);
            size_t pos = threadOffsets[tid][cellIndex];
            grid.cells[cellIndex].indices[pos] = i;
            threadOffsets[tid][cellIndex]++;
        }
    }

    // ------------------------------------------------------------------
    // Step 7: Enforce deterministic ordering.
    // Sort the indices in each cell.
    // ------------------------------------------------------------------
    for (int cell = 0; cell < numCells; cell++) {
        std::sort(grid.cells[cell].indices.begin(), grid.cells[cell].indices.end());
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

        int gx = static_cast<int>(std::floor(static_cast<double>(xi) / static_cast<double>(cellSize)));
        int gy = static_cast<int>(std::floor(static_cast<double>(yi) / static_cast<double>(cellSize)));

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

            int gx = static_cast<int>(std::floor(static_cast<double>(xi) / static_cast<double>(cellSize)));
            int gy = static_cast<int>(std::floor(static_cast<double>(yi) / static_cast<double>(cellSize)));

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
        PROFILE_SCOPE("BuildUniformGridContiguousDomainPartition");
        grid = buildUniformGridContiguousDomainPartition(dat, cellSize);
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

/**
 * @brief Lazily updates the uniform grid.
 *
 * This function checks whether particles have moved significantly (larger than lazyThreshold)
 * since the last grid build. If no particle has moved enough, the old grid is reused.
 * Otherwise, the grid is rebuilt (using buildUniformGridContiguousDomainPartition) and the
 * stored positions in dat.lastGridX/lastGridY are updated.
 *
 * @param oldGrid       The grid from the previous update.
 * @param dat           The fluid particle data (including positions).
 * @param cellSize      The spatial cell size.
 * @param lazyThreshold The distance a particle must have moved to trigger a grid update.
 *
 * @return The updated grid.
 */
UniformGrid updateUniformGridLazy(UniformGrid &oldGrid, FluidParticleData &dat, float cellSize, float lazyThreshold)
{
    int n = static_cast<int>(dat.x.size());

    // On first call (or if lastGrid arrays haven't been initialized) copy current positions.
    if (dat.lastGridX.size() != static_cast<size_t>(n) || dat.lastGridY.size() != static_cast<size_t>(n)) {
        dat.lastGridX = dat.x;
        dat.lastGridY = dat.y;
        oldGrid = buildUniformGridContiguousDomainPartition(dat, cellSize);
        return oldGrid;
    }

    // Check if any particle has moved more than lazyThreshold.
    bool significantMovement = false;
    for (int i = 0; i < n; i++) {
        float dx = dat.x[i] - dat.lastGridX[i];
        float dy = dat.y[i] - dat.lastGridY[i];
        if (dx * dx + dy * dy > lazyThreshold * lazyThreshold) {
            significantMovement = true;
            break;
        }
    }

    // If no particle moved enough, reuse the existing grid.
    if (!significantMovement) {
        return oldGrid;
    }

    // Otherwise, rebuild the grid.
    UniformGrid newGrid = buildUniformGridContiguousDomainPartition(dat, cellSize);

    // Update the stored positions.
    for (int i = 0; i < n; i++) {
        dat.lastGridX[i] = dat.x[i];
        dat.lastGridY[i] = dat.y[i];
    }
    return newGrid;
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