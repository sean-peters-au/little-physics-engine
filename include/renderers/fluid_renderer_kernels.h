/**
 * @file fluid_renderer_kernels.h
 * @brief Shared data structures and constants for GPU fluid *rendering*.
 *
 * Defines types used by both the C++ FluidRenderer and the Metal
 * compute/render kernels for density grid generation, blurring, and final shading.
 * Separate from fluid_kernels.h which is for physics simulation.
 */
#pragma once

// Type compatibility layer for C++ <-> Metal
#ifdef __METAL_VERSION__
    #include <metal_stdlib>
    using namespace metal;
    #define KERNEL_CONSTANT constant
    // Define float2/uint2 if not automatically available (usually are)
    // typedef metal::float2 float2;
    // typedef metal::uint2 uint2;
#else
    // Define C++ types that map reasonably to Metal types
    #include <simd/simd.h> // Use simd types for Metal compatibility
    typedef simd_float2 float2;
    typedef simd_uint2 uint2;
    typedef simd_float4 float4;
    #define KERNEL_CONSTANT constexpr
#endif

/**
 * @struct GPURenderFluidParticle
 * @brief Simplified particle data needed for rendering kernels.
 */
struct GPURenderFluidParticle {
    float2 position;        // Particle position in simulation meters
    float smoothingRadius; // Particle's influence radius (h) in simulation meters
    // Add mass or other props if needed by density kernel weighting
};

/**
 * @struct GPURenderParams
 * @brief Parameters controlling the rendering kernels.
 */
struct GPURenderParams {
    // Grid Configuration
    uint2 gridSize;         // Dimensions of the density grid (e.g., 200x200)
    float cellSize;         // Size of a grid cell in simulation meters
    float2 gridOrigin;      // Bottom-left corner of the grid in simulation meters

    // Particle Info
    unsigned int particleCount;     // Use unsigned int for C++

    // Kernel Parameters
    float defaultSmoothingRadius; // Fallback h if particle h is invalid
    float smoothingRadius;      // Relative smoothing radius (like CPU's parameter)
    float maxDensity;           // Max density found (for normalization kernel)
    // Add other params like density scaling factors if needed
};

/**
 * @struct FluidFragmentParams
 * @brief Parameters for the fluid fragment shader.
 */
struct FluidFragmentParams {
    float4 baseColor;    // Base color of the fluid
    float threshold;     // Density threshold for fluid surface
    float smoothness;    // Smoothing factor for fluid surface transition
    float2 _padding;    // Maintain 16-byte alignment
};

// Define constants if needed, e.g.:
// KERNEL_CONSTANT float MAX_DENSITY = 100.0f; 