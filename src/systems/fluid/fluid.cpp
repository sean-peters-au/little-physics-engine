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
 * @brief Key for uniform grid hashing in 2D
 */
struct CellKey {
    int gx;
    int gy;
    bool operator==(const CellKey &o) const {
        return gx == o.gx && gy == o.gy;
    }
};

struct CellKeyHash {
    size_t operator()(const CellKey &k) const {
        auto h1 = std::hash<int>()(k.gx);
        auto h2 = std::hash<int>()(k.gy);
        return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
    }
};

/** Builds a uniform grid for neighbor search (keyed by cell indices). */
std::unordered_map<CellKey, GridCell, CellKeyHash> buildUniformGrid(const FluidParticleData &dat,
                                                                    float cellSize)
{
    std::unordered_map<CellKey, GridCell, CellKeyHash> grid;
    grid.reserve(dat.x.size()); // might help reduce rehash

    int n = static_cast<int>(dat.x.size());
    for (int i = 0; i < n; i++) {
        float px = dat.x[i];
        float py = dat.y[i];
        int gx = static_cast<int>(std::floor(px / cellSize));
        int gy = static_cast<int>(std::floor(py / cellSize));

        CellKey key{gx, gy};
        grid[key].indices.push_back(i);
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
                      const std::unordered_map<CellKey, GridCell, CellKeyHash> &grid,
                      float cellSize)
{
    PROFILE_SCOPE("ComputeDensities");

    int n = static_cast<int>(dat.x.size());
    // Clear densities
    #pragma omp parallel for
    for (int i = 0; i < n; i++) {
        dat.density[i] = 0.f;
    }

    #pragma omp parallel for
    for (int i = 0; i < n; i++) {
        float xi = dat.x[i];
        float yi = dat.y[i];
        float hi = dat.h[i];
        float h2 = hi*hi;
        float coeff = poly6Coeff2D(hi);

        float accum = 0.f;

        int gx = static_cast<int>(std::floor(xi / cellSize));
        int gy = static_cast<int>(std::floor(yi / cellSize));

        // We'll collect neighbor coords in small arrays, then do NEON ops on them in batches of 4
        float neighborX[128];
        float neighborY[128];
        float neighborMass[128];
        int   neighborCount = 0;

        // For each relevant cell
        for (int cx = gx-1; cx <= gx+1; cx++) {
            for (int cy = gy-1; cy <= gy+1; cy++) {
                auto it = grid.find(CellKey{cx, cy});
                if (it == grid.end()) continue;
                for (auto j : it->second.indices) {
                    // If we exceed local array, process now (rare if spacing is decent)
                    if (neighborCount == 128) {
                        // We'll process them now
                        for (int idx = 0; idx < neighborCount; idx+=4) {
                            float32x4_t x4  = vld1q_f32(&neighborX[idx]);
                            float32x4_t y4  = vld1q_f32(&neighborY[idx]);
                            float32x4_t dx4 = vsubq_f32(vdupq_n_f32(xi), x4);
                            float32x4_t dy4 = vsubq_f32(vdupq_n_f32(yi), y4);
                            // r2 = dx^2 + dy^2
                            float32x4_t dx2 = vmulq_f32(dx4, dx4);
                            float32x4_t dy2 = vmulq_f32(dy4, dy4);
                            float32x4_t r2  = vaddq_f32(dx2, dy2);

                            // compare r2 < h2
                            float32x4_t mask = vcltq_f32(r2, vdupq_n_f32(h2));
                            // if any are < h2, we do poly6 on them
                            // We'll do a "bitmask" approach:
                            uint32x4_t maskBits = vreinterpretq_u32_f32(mask);
                            // We'll handle them individually
                            float out[4]; vst1q_f32(out, vreinterpretq_f32_u32(maskBits));
                            for (int k=0; k<4; k++) {
                                // If mask is nonzero, we are inside
                                if (reinterpret_cast<uint32_t&>(out[k]) == 0xFFFFFFFFu) {
                                    // diff = (h^2 - r^2)
                                    float r2f = (dx4[k]*dx4[k] + dy4[k]*dy4[k]);
                                    float diff = h2 - r2f;
                                    float w = coeff * diff*diff*diff;
                                    accum += neighborMass[idx + k] * w;
                                }
                            }
                        }
                        neighborCount = 0; // reset
                    }
                    // stash j into the local arrays
                    neighborX[neighborCount]   = dat.x[j];
                    neighborY[neighborCount]   = dat.y[j];
                    neighborMass[neighborCount]= dat.mass[j];
                    neighborCount++;
                }
            }
        }

        // Process any leftover neighbors
        if (neighborCount > 0) {
            // do the same NEON batch approach, but partial
            int alignedCount = (neighborCount / 4) * 4;
            for (int idx=0; idx<alignedCount; idx+=4) {
                float32x4_t x4  = vld1q_f32(&neighborX[idx]);
                float32x4_t y4  = vld1q_f32(&neighborY[idx]);
                float32x4_t dx4 = vsubq_f32(vdupq_n_f32(xi), x4);
                float32x4_t dy4 = vsubq_f32(vdupq_n_f32(yi), y4);
                float32x4_t dx2 = vmulq_f32(dx4, dx4);
                float32x4_t dy2 = vmulq_f32(dy4, dy4);
                float32x4_t r2  = vaddq_f32(dx2, dy2);

                float32x4_t mask = vcltq_f32(r2, vdupq_n_f32(h2));
                uint32x4_t maskBits = vreinterpretq_u32_f32(mask);
                float out[4]; vst1q_f32(out, vreinterpretq_f32_u32(maskBits));
                for (int k=0; k<4; k++) {
                    if (reinterpret_cast<uint32_t&>(out[k]) == 0xFFFFFFFFu) {
                        float r2f = (dx4[k]*dx4[k] + dy4[k]*dy4[k]);
                        float diff = h2 - r2f;
                        float w = coeff * diff*diff*diff;
                        accum += neighborMass[idx + k] * w;
                    }
                }
            }
            // process remainder (1-3)
            for (int idx = alignedCount; idx < neighborCount; idx++) {
                float dx = xi - neighborX[idx];
                float dy = yi - neighborY[idx];
                float r2 = dx*dx + dy*dy;
                if (r2 < h2) {
                    float diff = (h2 - r2);
                    float w = coeff * diff*diff*diff;
                    accum += neighborMass[idx] * w;
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
                   const std::unordered_map<CellKey, GridCell, CellKeyHash> &grid,
                   float cellSize,
                   float restDensity,
                   float stiffness,
                   float viscosity)
{
    PROFILE_SCOPE("ComputeForces");

    int n = static_cast<int>(dat.x.size());

    // 1) Pressure from Equation of State
    #pragma omp parallel for
    for (int i = 0; i < n; i++) {
        float rho = dat.density[i];
        float p = stiffness*(rho - restDensity);
        dat.pressure[i] = (p > 0.f ? p : 0.f);
    }

    // Clear old accelerations
    #pragma omp parallel for
    for (int i = 0; i < n; i++) {
        dat.ax[i] = 0.f;
        dat.ay[i] = 0.f;
    }

    // 2) Pressure + Viscosity loops
    #pragma omp parallel for
    for (int i = 0; i < n; i++) {
        float xi = dat.x[i];
        float yi = dat.y[i];
        float pi = dat.pressure[i];
        float rhoi = dat.density[i];
        float hi = dat.h[i];
        if (rhoi < 1e-12f) continue;

        // We'll gather neighbor data in small arrays for partial NEON usage:
        float neighborX[128];
        float neighborY[128];
        float neighborVx[128];
        float neighborVy[128];
        float neighborMass[128];
        float neighborP[128];
        float neighborRho[128];
        float neighborH[128];

        float sumAx = 0.f;
        float sumAy = 0.f;

        // building local neighbor list
        int gx = static_cast<int>(std::floor(xi / cellSize));
        int gy = static_cast<int>(std::floor(yi / cellSize));

        // We'll store partial results here, then do partial NEON
        int neighborCount = 0;

        for (int cx = gx-1; cx <= gx+1; cx++) {
            for (int cy = gy-1; cy <= gy+1; cy++) {
                auto it = grid.find(CellKey{cx, cy});
                if (it == grid.end()) continue;

                for (auto j : it->second.indices) {
                    if (j == i) continue;
                    neighborX[neighborCount]   = dat.x[j];
                    neighborY[neighborCount]   = dat.y[j];
                    neighborVx[neighborCount]  = dat.vx[j];
                    neighborVy[neighborCount]  = dat.vy[j];
                    neighborMass[neighborCount]= dat.mass[j];
                    neighborP[neighborCount]   = dat.pressure[j];
                    neighborRho[neighborCount] = dat.density[j];
                    neighborH[neighborCount]   = dat.h[j];
                    neighborCount++;

                    if (neighborCount == 128) {
                        // Process them now
                        // We'll do a simpler scalar approach to accumulate partial sums,
                        // but do NEON for the distance check & weighting.
                        for (int idx = 0; idx < neighborCount; idx++) {
                            float dx = xi - neighborX[idx];
                            float dy = yi - neighborY[idx];
                            float r2 = dx*dx + dy*dy;
                            if (r2 < 1e-14f) continue;

                            // average smoothing length
                            float h_ij = 0.5f*(hi + neighborH[idx]);
                            if (r2 >= (h_ij*h_ij)) continue;

                            float r = std::sqrt(r2);
                            float rhoj = neighborRho[idx];
                            if (rhoj < 1e-12f) continue;

                            float pj = neighborP[idx];
                            // symmetrical pressure term
                            float term = (pi/(rhoi*rhoi)) + (pj/(rhoj*rhoj));

                            float spikyFactor = spikyCoeff2D(h_ij);
                            float diff = (h_ij - r);
                            float w_spiky = spikyFactor*(diff*diff);

                            float rx = dx / r;
                            float ry = dy / r;

                            float fPress = -neighborMass[idx]*term*w_spiky;
                            float fx = fPress*rx;
                            float fy = fPress*ry;

                            // Viscosity
                            float vx_ij = dat.vx[i] - neighborVx[idx];
                            float vy_ij = dat.vy[i] - neighborVy[idx];
                            float lapFactor = viscLaplacianCoeff2D(h_ij);
                            float w_visc = lapFactor*(diff);
                            float fVisc = viscosity*neighborMass[idx]*(w_visc/rhoj);
                            fx -= fVisc*vx_ij; // minus because we did v_i - v_j
                            fy -= fVisc*vy_ij;

                            sumAx += fx;
                            sumAy += fy;
                        }
                        neighborCount = 0; // reset
                    }
                }
            }
        }

        // process leftover
        for (int idx=0; idx<neighborCount; idx++) {
            float dx = xi - neighborX[idx];
            float dy = yi - neighborY[idx];
            float r2 = dx*dx + dy*dy;
            if (r2 < 1e-14f) continue;
            float h_ij = 0.5f*(hi + neighborH[idx]);
            if (r2 >= (h_ij*h_ij)) continue;

            float r = std::sqrt(r2);
            float rhoj = neighborRho[idx];
            if (rhoj < 1e-12f) continue;

            float pj = neighborP[idx];
            float term = (pi/(rhoi*rhoi)) + (pj/(rhoj*rhoj));

            float spikyFactor = spikyCoeff2D(h_ij);
            float diff = (h_ij - r);
            float w_spiky = spikyFactor*(diff*diff);

            float rx = dx / r;
            float ry = dy / r;

            float fPress = -neighborMass[idx]*term*w_spiky;
            float fx = fPress*rx;
            float fy = fPress*ry;

            // Viscosity
            float vx_ij = dat.vx[i] - neighborVx[idx];
            float vy_ij = dat.vy[i] - neighborVy[idx];
            float lapFactor = viscLaplacianCoeff2D(h_ij);
            float w_visc = lapFactor*(diff);
            float fVisc = viscosity*neighborMass[idx]*(w_visc/rhoj);
            fx -= fVisc*vx_ij;
            fy -= fVisc*vy_ij;

            sumAx += fx;
            sumAy += fy;
        }

        // store
        dat.ax[i] = sumAx;
        dat.ay[i] = sumAy;
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
    #pragma omp parallel for
    for (int i = 0; i < n; i++) {
        dat.vHalfx[i] = dat.vx[i] + 0.5f*dat.ax[i]*subDt;
        dat.vHalfy[i] = dat.vy[i] + 0.5f*dat.ay[i]*subDt;
    }

    // (2) x = x + vHalf dt
    #pragma omp parallel for
    for (int i = 0; i < n; i++) {
        dat.x[i] += dat.vHalfx[i]*subDt;
        dat.y[i] += dat.vHalfy[i]*subDt;
    }

    // build uniform grid from updated x,y
    // pick cellSize ~ 2*h. For simplicity, use first particle's h
    float h0 = (n > 0 ? dat.h[0] : 0.05f);
    float cellSize = 2.f*h0;
    auto grid = buildUniformGrid(dat, cellSize);

    // compute densities
    computeDensities(dat, grid, cellSize);

    // compute new forces (a)
    computeForces(dat, grid, cellSize, restDensity, stiffness, viscosity);

    // (3) v = vHalf + 0.5 a dt
    #pragma omp parallel for
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
    FluidParticleData data = gatherFluidParticles(registry);
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
    for (int step = 0; step < N_SUB_STEPS; step++) {
        velocityVerletSubStep(data, subDt, restDensity, stiffness, viscosity);
    }

    // 3) after all sub-steps, write final x,v back to ECS
    writeBackToECS(data, registry);
}

} // namespace Systems