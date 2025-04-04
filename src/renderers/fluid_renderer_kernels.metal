/**
 * @file fluid_renderer_kernels.metal
 * @brief Metal kernels for GPU-accelerated fluid rendering.
 *
 * Includes kernels for:
 * - Calculating particle density onto a grid.
 * - Applying a blur filter to the density grid.
 */

#include <metal_stdlib>
#include "renderers/fluid_renderer_kernels.h" // Shared definitions

using namespace metal;

// --- Constants and Helpers ---
#define M_PI 3.14159265358979323846f

/**
 * @brief SPH Poly6 Kernel (simplified for rendering density contribution).
 */
inline float kernelPoly6(float rSq, float hSq) {
    if (rSq >= hSq || hSq < 1e-12f) return 0.0f;
    float diff = hSq - rSq;
    // Normalization factor often omitted for visual density, scaled later
    // float h8 = hSq*hSq*hSq*hSq;
    // float norm = 4.0f / (M_PI * h8);
    // return norm * diff * diff * diff;
    return diff * diff * diff;
}

// --- Kernel: Calculate Density Grid ---
/**
 * @brief Calculates density contribution of particles to grid cells.
 * Each thread calculates the density for one grid cell.
 */
kernel void calculateDensityGrid(
    texture2d<float, access::write> densityGrid [[texture(0)]], // Output density grid texture
    const device GPURenderFluidParticle* particles [[buffer(0)]], // Input particle data
    constant GPURenderParams& params [[buffer(1)]], // Input parameters
    uint2 gid [[thread_position_in_grid]]) // Grid ID = grid cell coordinates (x, y)
{
    // Check grid bounds
    if (gid.x >= params.gridSize.x || gid.y >= params.gridSize.y) {
        return;
    }

    // Calculate the center position of this grid cell in simulation space
    float2 cellCenterPos = params.gridOrigin + (float2(gid) + 0.5f) * params.cellSize;

    float density = 0.0f;
    float h_default = params.defaultSmoothingRadius;
    float h_default_sq = h_default * h_default;

    // Iterate through all particles
    for (uint i = 0; i < params.particleCount; ++i) {
        float2 particlePos = particles[i].position;
        // --- DEBUG: Force using default H --- 
        // float h = particles[i].smoothingRadius > 0.0f ? particles[i].smoothingRadius : h_default;
        float hSq = h_default_sq; // Always use default h^2 for now
        // ------------------------------------

        float2 diff = cellCenterPos - particlePos;
        float distSq = dot(diff, diff);

        // Accumulate density contribution using SPH kernel
        density += kernelPoly6(distSq, hSq); // Pass the currently used hSq
    }

    // Write the calculated density to the output texture
    // Usually store normalized density [0,1] or scale later in blur/shader
    // For now, store raw density sum
    densityGrid.write(density, gid);
}

// --- Kernel: Box Blur (Simple) ---
/**
 * @brief Applies a simple 3x3 box blur to the input texture.
 * Each thread calculates the blurred value for one pixel.
 */
kernel void boxBlur(
    texture2d<float, access::read> inputTexture [[texture(0)]],
    texture2d<float, access::write> outputTexture [[texture(1)]],
    uint2 gid [[thread_position_in_grid]])
{
    uint width = inputTexture.get_width();
    uint height = inputTexture.get_height();

    if (gid.x >= width || gid.y >= height) return;

    float sum = 0.0f;
    int count = 0;
    int currentWidth = (int)width;   // Cast to int for comparison
    int currentHeight = (int)height; // Cast to int for comparison

    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            int2 sampleCoord = int2(gid) + int2(dx, dy);
            // Use casted width/height for comparison
            if (sampleCoord.x >= 0 && sampleCoord.x < currentWidth && sampleCoord.y >= 0 && sampleCoord.y < currentHeight) {
                sum += inputTexture.read(uint2(sampleCoord)).x;
                count++;
            }
        }
    }
    float blurredValue = (count > 0) ? (sum / count) : 0.0f;
    outputTexture.write(blurredValue, gid);
}

// --- Vertex Shader for Fullscreen Quad ---
/**
 * @brief Input vertex structure for the fullscreen quad.
 */
struct ScreenVertex {
    float4 position [[position]]; // Clip space position
    float2 texCoord;           // Texture coordinate
};

/**
 * @brief Simple pass-through vertex shader.
 */
vertex ScreenVertex screenQuadVertexShader(
    uint vertexID [[vertex_id]]
)
{
    ScreenVertex out;
    // Generate fullscreen triangle vertices based on vertex ID
    // (Covers clip space -1 to +1 in X and Y)
    float2 pos;
    float2 uv;
    if (vertexID == 0) { pos = float2(-1.0, -1.0); uv = float2(0.0, 1.0); }
    else if (vertexID == 1) { pos = float2( 3.0, -1.0); uv = float2(2.0, 1.0); }
    else               { pos = float2(-1.0,  3.0); uv = float2(0.0,-1.0); }

    out.position = float4(pos, 0.0, 1.0);
    out.texCoord = uv;
    return out;
}

// --- Fragment Shader for Screen-Space Fluid ---
/**
 * @brief Renders the fluid surface based on blurred density.
 */
fragment float4 fluidScreenSpaceFragmentShader(
    ScreenVertex in [[stage_in]],
    texture2d<float, access::sample> blurredTexture [[texture(0)]], // Use blurred texture
    constant FluidFragmentParams& params [[buffer(0)]] 
)
{
    constexpr sampler sam(filter::linear, address::clamp_to_edge); // Use linear sampler again
    // Read the blurred density value
    float blurredDensity = blurredTexture.sample(sam, in.texCoord).r;
    
    // Scale the density before thresholding, because raw values are very small
    float scaledDensity = blurredDensity * 100000000.0f; // Keep the large multiplier for now

    // Apply thresholding and smoothing using scaled density
    float alpha = smoothstep(params.threshold - params.smoothness,
                           params.threshold + params.smoothness,
                           scaledDensity);

    // Return final color (premultiplied alpha)
    float4 finalColor = params.baseColor;
    finalColor.a *= alpha;
    finalColor.rgb *= finalColor.a; 

    return finalColor;
}

// --- TODO: Add Screen-space Fragment Shader ---
/*
fragment float4 fluidFragmentShader(
    constant GPURenderParams& params [[buffer(0)]],
    texture2d<float> blurredDensityTexture [[texture(0)]],
    float2 texCoord [[stage_in]]) // Texture coordinates (0-1)
{
    // Sample blurred density
    float density = blurredDensityTexture.sample(sampler, texCoord).r;

    // Apply thresholding and smoothing (similar to fluid_screenspace.frag)
    float alpha = smoothstep(params.threshold - params.smoothness,
                           params.threshold + params.smoothness,
                           density);

    // Return final color (baseColor * alpha)
    return params.baseColor * alpha;
}
*/ 