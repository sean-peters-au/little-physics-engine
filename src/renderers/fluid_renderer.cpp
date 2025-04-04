/**
 * @file fluid_renderer.cpp
 * @brief GPU implementation of FluidRenderer using Metal.
 */

#include "renderers/fluid_renderer.hpp"
#include "entities/entity_components.hpp"
#include "renderers/fluid_renderer_kernels.h"
#include "core/profile.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <simd/simd.h>

// Metal Headers (ensure these are available in include path)
#include <Metal/Metal.hpp>
#include <Foundation/Foundation.hpp>
#include <QuartzCore/QuartzCore.hpp> // For CAMetalLayer if drawing directly

// Helper to create Metal compute pipelines (can be moved to a utility file)
static MTL::ComputePipelineState* createComputePipelineHelper(
    const char* fnName,
    MTL::Library* lib,
    MTL::Device* device)
{
    if (!lib || !device) return nullptr;
    NS::String* functionName = NS::String::string(fnName, NS::UTF8StringEncoding);
    MTL::Function* fn = lib->newFunction(functionName);
    if (!fn) {
        std::cerr << "Missing Metal function: " << fnName << std::endl;
        return nullptr;
    }
    NS::Error* error = nullptr;
    MTL::ComputePipelineState* pso = device->newComputePipelineState(fn, &error);
    fn->release();
    if (!pso) {
        std::cerr << "Failed to create compute pipeline " << fnName << ": "
                  << (error ? error->localizedDescription()->utf8String() : "Unknown")
                  << std::endl;
    }
    return pso;
}

// --- Helper Struct for Render Uniforms ---
// Needs to match layout expected by the fragment shader
struct GPURenderUniforms {
    // float threshold; // Example if sending individually
    // float smoothness;
    // float4 baseColor;
    // For simplicity, bundle them or pass individually via setFragmentBytes
};

// --- Constructor / Destructor ---
FluidRenderer::FluidRenderer(Simulation::Coordinates& coords, sf::Vector2u screenDims)
    : coordinates(coords),
      screenDimensions(screenDims)
{
    if (!initializeMetal()) {
        std::cerr << "Failed to initialize Metal for FluidRenderer!" << std::endl;
        // Set a flag or handle error - rendering will not work
    }
}

FluidRenderer::~FluidRenderer() {
    // Release Metal objects
    if (densityKernelPSO_) densityKernelPSO_->release();
    if (blurKernelPSO_) blurKernelPSO_->release();
    if (screenShaderPSO_) screenShaderPSO_->release(); // Release render PSO
    if (depthState_) depthState_->release(); // Release depth state
    if (particleBuffer_) particleBuffer_->release();
    if (paramsBuffer_) paramsBuffer_->release();
    if (densityTexture_) densityTexture_->release();
    if (blurredTexture_) blurredTexture_->release();
    if (finalFluidTexture_) finalFluidTexture_->release(); // Release new texture
    if (metalLibrary_) metalLibrary_->release();
    if (commandQueue_) commandQueue_->release();
    if (renderUniformsBuffer_) renderUniformsBuffer_->release(); // Release uniforms buffer
    if (normalizeDensityPSO_) normalizeDensityPSO_->release(); // Release new PSO
    // device_ is typically not released by us if obtained via CreateSystemDefaultDevice
}

// --- Metal Initialization ---
bool FluidRenderer::initializeMetal() {
    device_ = MTL::CreateSystemDefaultDevice();
    if (!device_) {
        std::cerr << "Metal is not supported on this device." << std::endl;
        return false;
    }

    commandQueue_ = device_->newCommandQueue();
    if (!commandQueue_) {
        std::cerr << "Failed to create Metal command queue." << std::endl;
        return false;
    }

    // Load library (adjust path as needed)
    NS::Error* error = nullptr;
    NS::String* libPath = NS::String::string("build/fluid_renderer_kernels.metallib", NS::UTF8StringEncoding);
    metalLibrary_ = device_->newLibrary(libPath, &error);
    if (!metalLibrary_) {
        std::cerr << "Failed to load fluid renderer Metal library: "
                  << (error ? error->localizedDescription()->utf8String() : "Unknown")
                  << std::endl;
        return false;
    }

    // Create Compute Pipeline States
    densityKernelPSO_ = createComputePipelineHelper("calculateDensityGrid", metalLibrary_, device_);
    blurKernelPSO_ = createComputePipelineHelper("boxBlur", metalLibrary_, device_);
    normalizeDensityPSO_ = createComputePipelineHelper("normalizeDensity", metalLibrary_, device_);
    // Add check if new PSOs failed
    if (!densityKernelPSO_ || !blurKernelPSO_ || !normalizeDensityPSO_) { /* Handle error */ return false; }

    // --- Create Render Pipeline State --- 
    MTL::Function* vertexFn   = metalLibrary_->newFunction(NS::String::string("screenQuadVertexShader", NS::UTF8StringEncoding));
    MTL::Function* fragmentFn = metalLibrary_->newFunction(NS::String::string("fluidScreenSpaceFragmentShader", NS::UTF8StringEncoding));

    if (!vertexFn || !fragmentFn) {
        std::cerr << "Failed to get render shader functions." << std::endl;
        if(vertexFn) vertexFn->release();
        if(fragmentFn) fragmentFn->release();
        return false;
    }

    MTL::RenderPipelineDescriptor* psoDesc = MTL::RenderPipelineDescriptor::alloc()->init();
    psoDesc->setVertexFunction(vertexFn);
    psoDesc->setFragmentFunction(fragmentFn);
    psoDesc->colorAttachments()->object(0)->setPixelFormat(MTL::PixelFormat::PixelFormatBGRA8Unorm);
    // --- Restore Original Blending --- //
    // psoDesc->colorAttachments()->object(0)->setBlendingEnabled(false); // Remove debug setting
    psoDesc->colorAttachments()->object(0)->setBlendingEnabled(true);
    psoDesc->colorAttachments()->object(0)->setSourceRGBBlendFactor(MTL::BlendFactor::BlendFactorSourceAlpha);
    psoDesc->colorAttachments()->object(0)->setDestinationRGBBlendFactor(MTL::BlendFactor::BlendFactorOneMinusSourceAlpha);
    psoDesc->colorAttachments()->object(0)->setSourceAlphaBlendFactor(MTL::BlendFactor::BlendFactorOne);
    psoDesc->colorAttachments()->object(0)->setDestinationAlphaBlendFactor(MTL::BlendFactor::BlendFactorOneMinusSourceAlpha);
    // Remove comment markers

    // --- Create Depth/Stencil State (Disable depth) ---
    MTL::DepthStencilDescriptor* depthDesc = MTL::DepthStencilDescriptor::alloc()->init();
    depthDesc->setDepthCompareFunction(MTL::CompareFunctionAlways); // Don't depth test
    depthDesc->setDepthWriteEnabled(false); // Don't write to depth buffer
    depthState_ = device_->newDepthStencilState(depthDesc);
    depthDesc->release();
    if (!depthState_) {
        std::cerr << "Failed to create depth/stencil state." << std::endl;
        // Clean up previous objects if needed
        vertexFn->release();
        fragmentFn->release();
        psoDesc->release();
        return false;
    }
    // --------------------------------------------------

    error = nullptr;
    screenShaderPSO_ = device_->newRenderPipelineState(psoDesc, &error);

    vertexFn->release();
    fragmentFn->release();
    psoDesc->release();

    if (!screenShaderPSO_) {
        std::cerr << "Failed to create screen shader render pipeline state: "
                  << (error ? error->localizedDescription()->utf8String() : "Unknown")
                  << std::endl;
        return false;
    }
    // ----------------------------------

    if (!densityKernelPSO_ || !blurKernelPSO_ || !screenShaderPSO_) {
        std::cerr << "Failed to create one or more Metal pipeline states." << std::endl;
        return false;
    }

    // Create params buffers
    paramsBuffer_ = device_->newBuffer(sizeof(GPURenderParams), MTL::ResourceStorageModeShared);
    renderUniformsBuffer_ = device_->newBuffer(sizeof(GPURenderUniforms), MTL::ResourceStorageModeManaged); // Use Managed or Shared
    if (!paramsBuffer_ || !renderUniformsBuffer_) {
        std::cerr << "Failed to create params/uniforms buffer." << std::endl;
        return false;
    }

    // --- Create Final Output Texture --- 
    MTL::TextureDescriptor* texDesc = MTL::TextureDescriptor::alloc()->init();
    texDesc->setWidth(screenDimensions.x); // Use screen dimensions
    texDesc->setHeight(screenDimensions.y);
    texDesc->setPixelFormat(MTL::PixelFormatBGRA8Unorm); // Match SFML expectation
    texDesc->setTextureType(MTL::TextureType2D);
    // Use Shared or Managed for CPU access (Managed requires synchronization)
    texDesc->setStorageMode(MTL::StorageModeManaged);
    // Usage must include ShaderRead (for potential future effects) and RenderTarget
    texDesc->setUsage(MTL::TextureUsageRenderTarget | MTL::TextureUsageShaderRead);

    finalFluidTexture_ = device_->newTexture(texDesc);
    texDesc->release();
    if (!finalFluidTexture_) {
        std::cerr << "Error: Failed to create final fluid texture!" << std::endl;
        // Perform cleanup of other initialized resources
        return false;
    }
    // Set initial grid size to ensure compute textures are created
    currentGridSize_ = {0, 0}; 

    // --- Create Normalized Density Texture ---
    MTL::TextureDescriptor* normTexDesc = MTL::TextureDescriptor::alloc()->init();
    normTexDesc->setWidth(screenDimensions.x); // Match output size
    normTexDesc->setHeight(screenDimensions.y);
    // Use R8Unorm for single channel normalized output
    normTexDesc->setPixelFormat(MTL::PixelFormatR8Unorm);
    normTexDesc->setTextureType(MTL::TextureType2D);
    normTexDesc->setStorageMode(MTL::StorageModePrivate); // Can be private if only used by shader
    // Needs Read (by final shader) and Write (by normalize kernel)
    normTexDesc->setUsage(MTL::TextureUsageShaderRead | MTL::TextureUsageShaderWrite); 
    normalizedDensityTexture_ = device_->newTexture(normTexDesc);
    normTexDesc->release();
    if (!normalizedDensityTexture_) {
        std::cerr << "Error: Failed to create normalized density texture!" << std::endl;
        // Perform cleanup
        return false;
    }

    // --- Create Compute Textures (density & blur) ---
    MTL::TextureDescriptor* computeTexDesc = MTL::TextureDescriptor::alloc()->init();
    computeTexDesc->setPixelFormat(MTL::PixelFormatR32Float); 
    // Size will be set in ensureResources based on grid
    computeTexDesc->setTextureType(MTL::TextureType2D);
    // Blurred texture needs CPU read access -> Managed/Shared
    // Density texture only needs GPU access -> Private
    computeTexDesc->setStorageMode(MTL::StorageModePrivate); // Default for density
    computeTexDesc->setUsage(MTL::TextureUsageShaderRead | MTL::TextureUsageShaderWrite); 
    
    // Change storage mode for blurred texture
    computeTexDesc->setStorageMode(MTL::StorageModeManaged); 
    computeTexDesc->release();
    // Set initial grid size to ensure compute textures are created
    currentGridSize_ = {0, 0};
    // Check other resources
    if (!finalFluidTexture_ || !normalizedDensityTexture_) { return false; }

    std::cout << "Metal initialized successfully for FluidRenderer." << std::endl;
    return true;
}

// --- Resource Management ---
void FluidRenderer::ensureResources(int particleCount) {
    if (!device_) return;

    // Particle Buffer
    if (!particleBuffer_ || maxParticles_ < static_cast<size_t>(particleCount)) {
        if (particleBuffer_) particleBuffer_->release();
        maxParticles_ = particleCount > 0 ? static_cast<size_t>(particleCount) : 1;
        particleBuffer_ = device_->newBuffer(sizeof(GPURenderFluidParticle) * maxParticles_, MTL::ResourceStorageModeShared);
         if (!particleBuffer_) std::cerr << "Error: Failed to create particle buffer!" << std::endl;
         else std::cout << "Resized particle buffer to " << maxParticles_ << " particles." << std::endl;
    }

    // Compute Textures (based on compute grid size)
    uint2 computeGridSize = {200, 200}; // Example fixed size
    if (!densityTexture_ || !blurredTexture_ || currentGridSize_.x != computeGridSize.x || currentGridSize_.y != computeGridSize.y) {
        if (densityTexture_) densityTexture_->release();
        if (blurredTexture_) blurredTexture_->release();

        MTL::TextureDescriptor* texDesc = MTL::TextureDescriptor::alloc()->init();
        texDesc->setPixelFormat(MTL::PixelFormatR32Float);
        texDesc->setWidth(computeGridSize.x);
        texDesc->setHeight(computeGridSize.y);
        texDesc->setTextureType(MTL::TextureType2D);
        texDesc->setUsage(MTL::TextureUsageShaderRead | MTL::TextureUsageShaderWrite);

        // Create density texture (Private)
        texDesc->setStorageMode(MTL::StorageModePrivate);
        densityTexture_ = device_->newTexture(texDesc);

        // Create blurred texture (Managed)
        texDesc->setStorageMode(MTL::StorageModeManaged);
        blurredTexture_ = device_->newTexture(texDesc);
        
        texDesc->release();

        if (!densityTexture_ || !blurredTexture_) { // Check both
             std::cerr << "Error: Failed to create density/blur textures!" << std::endl;
        } else {
             std::cout << "Recreated compute textures with size " << computeGridSize.x << "x" << computeGridSize.y << std::endl;
        }
        currentGridSize_ = computeGridSize;
    }

    // Ensure normalizedDensityTexture_ matches screen dimensions
    if (!normalizedDensityTexture_ || 
        normalizedDensityTexture_->width() != screenDimensions.x || 
        normalizedDensityTexture_->height() != screenDimensions.y)
    {
        if (normalizedDensityTexture_) normalizedDensityTexture_->release();
        MTL::TextureDescriptor* normTexDesc = MTL::TextureDescriptor::alloc()->init();
        normTexDesc->setWidth(screenDimensions.x);
        normTexDesc->setHeight(screenDimensions.y);
        normTexDesc->setPixelFormat(MTL::PixelFormatR8Unorm);
        normTexDesc->setTextureType(MTL::TextureType2D);
        normTexDesc->setStorageMode(MTL::StorageModePrivate);
        normTexDesc->setUsage(MTL::TextureUsageShaderRead | MTL::TextureUsageShaderWrite);
        normalizedDensityTexture_ = device_->newTexture(normTexDesc);
        normTexDesc->release();
        if (!normalizedDensityTexture_) {
             std::cerr << "Error: Failed to recreate normalized density texture!" << std::endl;
        }
    }
    
    // Ensure finalFluidTexture_ exists (as before)
    if (!finalFluidTexture_) { // Should have been created in init, but check
         std::cerr << "Error: finalFluidTexture_ is null in ensureResources!" << std::endl;
         // Attempt to recreate? For now, just error.
         MTL::TextureDescriptor* texDesc = MTL::TextureDescriptor::alloc()->init();
         texDesc->setWidth(screenDimensions.x);
         texDesc->setHeight(screenDimensions.y);
         texDesc->setPixelFormat(MTL::PixelFormatBGRA8Unorm);
         texDesc->setStorageMode(MTL::StorageModeManaged);
         texDesc->setUsage(MTL::TextureUsageRenderTarget | MTL::TextureUsageShaderRead);
         finalFluidTexture_ = device_->newTexture(texDesc);
         texDesc->release();
         if (!finalFluidTexture_) return; // Give up if recreate fails
    }
}

// --- Main Render Method ---
void FluidRenderer::render(const entt::registry& registry)
{
    if (!device_ || !commandQueue_ || !metalLibrary_ || !densityKernelPSO_ || !blurKernelPSO_ || !screenShaderPSO_ || !depthState_) {
        return; // Ensure Metal is initialized
    }

    PROFILE_SCOPE("FluidRenderer::render (GPU)");

    // 1. Gather particle data
    std::vector<GPURenderFluidParticle> cpuParticles;
    cpuParticles.reserve(registry.view<Components::Position, Components::Shape, Components::ParticlePhase>().size_hint());
    auto view = registry.view<Components::Position, Components::Shape, Components::ParticlePhase>();
    for(auto entity : view) {
        const auto& phase = view.get<Components::ParticlePhase>(entity);
        if (phase.phase != Components::Phase::Liquid) continue;

        const auto& pos = view.get<Components::Position>(entity);
        const auto& shape = view.get<Components::Shape>(entity);

        GPURenderFluidParticle p;
        p.position = { (float)pos.x, (float)pos.y };
        p.smoothingRadius = shape.type == Components::ShapeType::Circle ? (float)shape.size : 0.05f; // Default if not circle
        cpuParticles.push_back(p);
    }

    int particleCount = static_cast<int>(cpuParticles.size());
    if (particleCount == 0) { 
        // TODO: Optionally clear the finalFluidTexture_ here if desired
        return; 
    }

    // 2. Determine Compute Grid Configuration
    uint2 computeGridSize = {200, 200}; // Keep grid resolution fixed for now
    // --- Calculate grid parameters based on simulation space ---
    // Assuming the main simulation area corresponds to screenDimensions width
    float simWidthMeters = static_cast<float>(coordinates.pixelsToMeters(screenDimensions.x));
    // Assuming square simulation area for simplicity 
    // TODO: Adjust if simulation height differs significantly
    float simHeightMeters = static_cast<float>(coordinates.pixelsToMeters(screenDimensions.y)); 
    // Calculate cell size based on simulation width and grid resolution
    float cellSize = simWidthMeters / static_cast<float>(computeGridSize.x);
    // Assume grid covers the area from (0,0) up to (simWidthMeters, simHeightMeters)
    // TODO: Adjust gridOrigin if the simulation area doesn't start at (0,0)
    float2 gridOrigin = {0.0f, 0.0f}; 
    // ---------------------------------------------------------

    // 3. Ensure Metal resources
    ensureResources(particleCount);
    if (!particleBuffer_ || !paramsBuffer_ || !densityTexture_ || !blurredTexture_ || 
        !finalFluidTexture_ || !normalizedDensityTexture_) 
    {
         std::cerr << "Error: Missing required Metal resources for render." << std::endl;
        return; 
    }

    // 4. Update Buffers
    memcpy(particleBuffer_->contents(), cpuParticles.data(), sizeof(GPURenderFluidParticle) * particleCount);
    GPURenderParams params;
    params.gridSize = computeGridSize;
    params.cellSize = cellSize; 
    params.gridOrigin = gridOrigin;
    params.particleCount = particleCount;
    params.defaultSmoothingRadius = 0.05f; 
    params.smoothingRadius = 5.0f;
    memcpy(paramsBuffer_->contents(), &params, sizeof(GPURenderParams));

    // --- DEBUG: Log Parameters and Particle Data ---
    std::cout << "Debug [Compute Params]: particleCount=" << params.particleCount 
              << ", gridSize=(" << params.gridSize.x << "," << params.gridSize.y 
              << "), cellSize=" << params.cellSize 
              << ", gridOrigin=(" << params.gridOrigin.x << "," << params.gridOrigin.y 
              << "), defaultH=" << params.defaultSmoothingRadius 
              << ", smoothingRadius=" << params.smoothingRadius // Log new param
              << std::endl;
    if (!cpuParticles.empty()) {
        std::cout << "Debug [Particle 0]: pos=(" << cpuParticles[0].position.x << "," << cpuParticles[0].position.y 
                  << "), h=" << cpuParticles[0].smoothingRadius << std::endl;
    }
    if (cpuParticles.size() > 100) { // Log another particle if available
         std::cout << "Debug [Particle 100]: pos=(" << cpuParticles[100].position.x << "," << cpuParticles[100].position.y 
                   << "), h=" << cpuParticles[100].smoothingRadius << std::endl;
    }
    // -----------------------------------------------

    // --- Execute Compute & Render --- 
    MTL::CommandBuffer* cmdBuffer = commandQueue_->commandBuffer();
    
    // --- Compute Pass: Density & Blur ---
    MTL::ComputeCommandEncoder* compEnc = cmdBuffer->computeCommandEncoder();
    // 1. Density Kernel (as before)
    compEnc->setComputePipelineState(densityKernelPSO_);
    compEnc->setTexture(densityTexture_, 0);
    compEnc->setBuffer(particleBuffer_, 0, 0);
    compEnc->setBuffer(paramsBuffer_, 0, 1);
    MTL::Size densityGridDim = MTL::Size(currentGridSize_.x, currentGridSize_.y, 1);
    MTL::Size densityTgSize = MTL::Size(densityKernelPSO_->threadExecutionWidth(), densityKernelPSO_->maxTotalThreadsPerThreadgroup() / densityKernelPSO_->threadExecutionWidth(), 1);
    compEnc->dispatchThreads(densityGridDim, densityTgSize);
    std::cout << "Debug: Density kernel dispatched." << std::endl;

    // 2. Blur Kernel (as before)
    compEnc->setComputePipelineState(blurKernelPSO_);
    compEnc->setTexture(densityTexture_, 0);   // Read from raw density
    compEnc->setTexture(blurredTexture_, 1);  // Write to blurred texture
    MTL::Size blurGridDim = MTL::Size(currentGridSize_.x, currentGridSize_.y, 1);
    MTL::Size blurTgSize = MTL::Size(blurKernelPSO_->threadExecutionWidth(), blurKernelPSO_->maxTotalThreadsPerThreadgroup() / blurKernelPSO_->threadExecutionWidth(), 1);
    compEnc->dispatchThreads(blurGridDim, blurTgSize);
    std::cout << "Debug: Blur kernel dispatched." << std::endl;

    // End encoding for density/blur ONLY
    compEnc->endEncoding(); 
    std::cout << "Debug: Density & Blur encoding finished." << std::endl;

    // --- Find Max Density on CPU ---
    // Commit and wait for density/blur to finish
    cmdBuffer->commit(); 
    cmdBuffer->waitUntilCompleted();
    std::cout << "Debug: Density & Blur command buffer completed." << std::endl;

    // Read back blurredTexture_ (Requires it to be Managed storage)
    float maxDensityFound = 0.0f;
    if (blurredTexture_ && blurredTexture_->storageMode() == MTL::StorageModeManaged) {
        const size_t width = blurredTexture_->width();
        const size_t height = blurredTexture_->height();
        const size_t bytesPerRow = width * sizeof(float); // R32Float
        const size_t bufferSize = bytesPerRow * height;
        std::vector<float> cpuTextureData(width * height); // Buffer for floats

        // Synchronize Managed texture
        MTL::CommandBuffer* syncCmdBuffer = commandQueue_->commandBuffer();
        MTL::BlitCommandEncoder* blitEncoder = syncCmdBuffer->blitCommandEncoder();
        blitEncoder->synchronizeResource(blurredTexture_);
        blitEncoder->endEncoding();
        syncCmdBuffer->commit();
        syncCmdBuffer->waitUntilCompleted();
        std::cout << "Debug: Synchronized blurredTexture." << std::endl;

        // Get Bytes
        MTL::Region region = MTL::Region::Make2D(0, 0, width, height);
        blurredTexture_->getBytes(cpuTextureData.data(), bytesPerRow, region, 0);
        std::cout << "Debug: Read blurredTexture to CPU." << std::endl;

        // Find max value in CPU buffer
        for(float val : cpuTextureData) {
            maxDensityFound = std::max(maxDensityFound, val);
        }
        std::cout << "Debug: Found max density on CPU: " << maxDensityFound << std::endl;
    } else {
        std::cerr << "Warning: Cannot find max density, blurredTexture_ is null or not Managed." << std::endl;
    }
    // ----------------------------

    // --- Update Params Buffer with Max Density ---
    params.maxDensity = maxDensityFound;
    // Copy the updated params (including maxDensity) back to the buffer
    memcpy(paramsBuffer_->contents(), &params, sizeof(GPURenderParams));
    // If paramsBuffer_ is Managed, mark range modified
    if (paramsBuffer_->storageMode() == MTL::StorageModeManaged) {
        paramsBuffer_->didModifyRange(NS::Range(0, sizeof(GPURenderParams)));
    }
    std::cout << "Debug: Updated params buffer with maxDensity=" << params.maxDensity << std::endl;

    // --- Compute Pass: Normalization & Render Pass ---
    // Need a NEW command buffer as the previous one was committed
    cmdBuffer = commandQueue_->commandBuffer(); 
    compEnc = cmdBuffer->computeCommandEncoder(); // New encoder

    // Dispatch Normalize Density Kernel
    compEnc->setComputePipelineState(normalizeDensityPSO_);
    compEnc->setTexture(blurredTexture_, 0); // Input: blurred
    compEnc->setTexture(normalizedDensityTexture_, 1); // Output: normalized
    // Pass ALL params via buffer now
    compEnc->setBuffer(paramsBuffer_, 0, 0); // Input: params (contains maxDensity)
    MTL::Size normGridDim = MTL::Size(currentGridSize_.x, currentGridSize_.y, 1);
    MTL::Size normTgSize = MTL::Size(normalizeDensityPSO_->threadExecutionWidth(), normalizeDensityPSO_->maxTotalThreadsPerThreadgroup() / normalizeDensityPSO_->threadExecutionWidth(), 1);
    compEnc->dispatchThreads(normGridDim, normTgSize);
    std::cout << "Debug: Normalize density kernel dispatched." << std::endl;

    // End compute encoding for normalization
    compEnc->endEncoding(); 
    std::cout << "Debug: Normalization encoding finished." << std::endl;

    // --- Render Pass to Offscreen Texture (uses normalized texture) ---
    // Create Render Pass Descriptor targeting finalFluidTexture_
    MTL::RenderPassDescriptor* rpd = MTL::RenderPassDescriptor::renderPassDescriptor();
    if (!rpd) {
        std::cerr << "Error: Failed to create RenderPassDescriptor." << std::endl;
        return;
    }
    rpd->colorAttachments()->object(0)->setTexture(finalFluidTexture_); // Target our texture
    rpd->colorAttachments()->object(0)->setLoadAction(MTL::LoadActionClear);
    rpd->colorAttachments()->object(0)->setStoreAction(MTL::StoreActionStore);
    rpd->colorAttachments()->object(0)->setClearColor(MTL::ClearColor(0.0, 0.0, 0.5, 1.0)); // Opaque Blue clear
    std::cout << "Debug: Configured RenderPassDescriptor for offscreen texture." << std::endl;

    // Create Render Command Encoder
    MTL::RenderCommandEncoder* renderEncoder = cmdBuffer->renderCommandEncoder(rpd);
    if (!renderEncoder) {
        std::cerr << "Error: Failed to create RenderCommandEncoder." << std::endl;
        return;
    }
    std::cout << "Debug: Created RenderCommandEncoder." << std::endl;

    // Set Viewport based on finalFluidTexture_
    MTL::Viewport viewport = {0.0, 0.0, (double)finalFluidTexture_->width(), (double)finalFluidTexture_->height(), 0.0, 1.0 };
    renderEncoder->setViewport(viewport);
    std::cout << "Debug: Set Viewport." << std::endl;

    // Checks for PSO and Depth State (as before)
    // ... checks ...

    // Set PSO and Depth State
    renderEncoder->setRenderPipelineState(screenShaderPSO_);
    renderEncoder->setDepthStencilState(depthState_);
    std::cout << "Debug: Set RenderPipelineState." << std::endl;
    std::cout << "Debug: Set DepthStencilState." << std::endl;

    // Set fragment inputs - Use NORMALIZED texture now!
    renderEncoder->setFragmentTexture(normalizedDensityTexture_, 0); 
    // Create and bind FluidFragmentParams buffer:
    sf::Color sfFluidColor(40, 130, 240, 255); // Restore base color
    FluidFragmentParams fragmentParams = {
        .baseColor = { 
            (float)sfFluidColor.r / 255.0f, 
            (float)sfFluidColor.g / 255.0f, 
            (float)sfFluidColor.b / 255.0f, 
            (float)sfFluidColor.a / 255.0f 
        },
        .threshold = 0.1f, 
        .smoothness = 0.02f 
    }; 
    renderEncoder->setFragmentBytes(&fragmentParams, sizeof(FluidFragmentParams), 0); 
    std::cout << "Debug: Set Fragment resources (Using normalizedDensityTexture_)." << std::endl;

    // Draw
    renderEncoder->drawPrimitives(MTL::PrimitiveTypeTriangle, NS::UInteger(0), NS::UInteger(3));
    std::cout << "Debug: Draw call encoded." << std::endl;

    // End encoding
    renderEncoder->endEncoding();
    std::cout << "Debug: Render encoding finished." << std::endl;

    // --- Commit --- (commits normalization compute + render pass)
    cmdBuffer->commit(); 
    std::cout << "Debug: Final command buffer committed." << std::endl;
}

// --- Read Texture Data --- // Updated for finalFluidTexture_
bool FluidRenderer::readFluidTexture(std::vector<uint8_t>& outBuffer, sf::Vector2u& outSize)
{
    if (!finalFluidTexture_) {
        std::cerr << "Error [readFluidTexture]: finalFluidTexture_ is null." << std::endl;
        return false;
    }

    PROFILE_SCOPE("ReadFluidTexture");

    const size_t width = finalFluidTexture_->width();
    const size_t height = finalFluidTexture_->height();
    const size_t bytesPerRow = width * 4; // BGRA is 4 bytes per pixel
    const size_t bytesPerImage = bytesPerRow * height;

    outBuffer.resize(bytesPerImage);
    outSize = sf::Vector2u(width, height);

    MTL::Region region = MTL::Region::Make2D(0, 0, width, height);

    // If texture storage mode is Managed, synchronize it before CPU access
    if (finalFluidTexture_->storageMode() == MTL::StorageModeManaged) {
        MTL::CommandBuffer* syncCmdBuffer = commandQueue_->commandBuffer();
        MTL::BlitCommandEncoder* blitEncoder = syncCmdBuffer->blitCommandEncoder();
        blitEncoder->synchronizeResource(finalFluidTexture_);
        blitEncoder->endEncoding();
        syncCmdBuffer->commit();
        syncCmdBuffer->waitUntilCompleted(); // Essential to wait for GPU sync
        std::cout << "Debug: Synchronized managed texture." << std::endl;
    }

    // Copy texture contents to buffer
    finalFluidTexture_->getBytes(outBuffer.data(), bytesPerRow, region, 0);
    std::cout << "Debug: Copied texture bytes to CPU buffer." << std::endl;

    // --- Swizzle R and B components (BGRA -> RGBA) ---
    for (size_t i = 0; i < bytesPerImage; i += 4) {
        // Bytes are B, G, R, A
        // Swap B (at i) and R (at i + 2)
        std::swap(outBuffer[i], outBuffer[i + 2]);
    }
    std::cout << "Debug: Swizzled texture bytes for SFML (BGRA -> RGBA)." << std::endl;

    return true;
}

// Stub implementations for debug methods
void FluidRenderer::toggleDebugVisualization() {
    // TODO: Implement Metal-based debug views if needed
    std::cout << "FluidRenderer::toggleDebugVisualization() called (No-Op for Metal)" << std::endl;
}

bool FluidRenderer::isDebugVisualization() const {
    // TODO: Implement Metal-based debug views if needed
    return false; // Currently no Metal debug views active
}
