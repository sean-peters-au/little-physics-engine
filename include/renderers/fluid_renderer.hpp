/**
 * @file fluid_renderer.hpp
 * @brief Handles rendering of fluid particles, potentially wrapping FluidSurfaceRenderer.
 */
#pragma once

#include <entt/entt.hpp>
#include <SFML/Graphics.hpp>
#include <memory>
#include <vector>

#include "core/coordinates.hpp"
#include "fluid_renderer_kernels.h"

// Forward declarations for Metal objects
namespace MTL { class Device; class CommandQueue; class Library; class ComputePipelineState; class Buffer; class Texture; class RenderPipelineState; class DepthStencilState; class BlitCommandEncoder; }

class FluidRenderer {
public:
    /**
     * @brief Constructor.
     * @param coordinates Reference to the coordinate conversion utility.
     * @param screenDims Dimensions of the simulation screen area (used for texture size).
     */
    FluidRenderer(Simulation::Coordinates& coordinates, sf::Vector2u screenDims);
    ~FluidRenderer();

    // Delete copy/move
    FluidRenderer(const FluidRenderer&) = delete;
    FluidRenderer& operator=(const FluidRenderer&) = delete;
    FluidRenderer(FluidRenderer&&) = delete;
    FluidRenderer& operator=(FluidRenderer&&) = delete;

    /**
     * @brief Renders fluid particles to an internal offscreen texture.
     * @param registry The ECS registry.
     */
    void render(const entt::registry& registry);

    /**
     * @brief Copies the rendered fluid texture data to a CPU buffer.
     * @param outBuffer Vector to store the BGRA pixel data.
     * @param outSize Returns the dimensions of the texture.
     * @return True if texture data was successfully read, false otherwise.
     */
    bool readFluidTexture(std::vector<uint8_t>& outBuffer, sf::Vector2u& outSize);

    /** @brief Toggles the internal fluid debug visualization modes (Currently No-Op). */
    void toggleDebugVisualization();

    /** @brief Checks if any fluid debug mode is active (Currently returns false). */
    bool isDebugVisualization() const;

private:
    /** @brief Initializes Metal device, queue, library, and pipelines. */
    bool initializeMetal();
    /** @brief Creates or resizes Metal buffers and textures if needed. */
    void ensureResources(int particleCount);

    Simulation::Coordinates& coordinates; // Reference, not owned
    sf::Vector2u screenDimensions; // Store initial dimensions for texture sizing

    // --- Metal Resources ---
    MTL::Device* device_ = nullptr;
    MTL::CommandQueue* commandQueue_ = nullptr;
    MTL::Library* metalLibrary_ = nullptr;

    // Compute Pipelines
    MTL::ComputePipelineState* densityKernelPSO_ = nullptr;
    MTL::ComputePipelineState* blurKernelPSO_ = nullptr;

    // Render Pipeline & State
    MTL::RenderPipelineState* screenShaderPSO_ = nullptr;
    MTL::DepthStencilState* depthState_ = nullptr; // Depth state for render pass

    // Buffers
    MTL::Buffer* particleBuffer_ = nullptr; // Holds GPURenderFluidParticle
    MTL::Buffer* paramsBuffer_ = nullptr;   // Holds GPURenderParams
    MTL::Buffer* renderUniformsBuffer_ = nullptr; // Buffer for render pass uniforms
    size_t maxParticles_ = 0;

    // Textures
    MTL::Texture* densityTexture_ = nullptr; // Raw density output
    MTL::Texture* blurredTexture_ = nullptr; // Blurred density output
    MTL::Texture* finalFluidTexture_ = nullptr; // Output texture for CPU readback (Managed/Shared)
    uint2 currentGridSize_ = {0, 0};

    // Fallback shader (if needed, moved from PresentationManager?)
    // std::unique_ptr<sf::Shader> metaballShader_;
}; 