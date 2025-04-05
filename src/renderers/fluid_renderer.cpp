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
    if (tempBlurTexture_) tempBlurTexture_->release(); // Release temp blur texture
    if (finalFluidTexture_) finalFluidTexture_->release();
    if (normalizedDensityTexture_) normalizedDensityTexture_->release();
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

    // --- Create Compute Textures Descriptors (Sizes set in ensureResources) ---
    MTL::TextureDescriptor* computeTexDesc = MTL::TextureDescriptor::alloc()->init();
    computeTexDesc->setPixelFormat(MTL::PixelFormatR32Float); 
    computeTexDesc->setTextureType(MTL::TextureType2D);
    computeTexDesc->setUsage(MTL::TextureUsageShaderRead | MTL::TextureUsageShaderWrite); 
    // Storage mode set specifically in ensureResources
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

    // Compute Textures - Size should match screen/output resolution now
    if (!densityTexture_ || !blurredTexture_ || !tempBlurTexture_ || // Check temp texture too
        densityTexture_->width() != screenDimensions.x || 
        densityTexture_->height() != screenDimensions.y) 
    { 
        if (densityTexture_) densityTexture_->release();
        if (blurredTexture_) blurredTexture_->release();
        if (tempBlurTexture_) tempBlurTexture_->release(); // Release temp texture

        MTL::TextureDescriptor* texDesc = MTL::TextureDescriptor::alloc()->init();
        texDesc->setPixelFormat(MTL::PixelFormatR32Float);
        texDesc->setWidth(screenDimensions.x);
        texDesc->setHeight(screenDimensions.y);
        texDesc->setTextureType(MTL::TextureType2D);
        texDesc->setUsage(MTL::TextureUsageShaderRead | MTL::TextureUsageShaderWrite);

        // Create density texture (Private)
        texDesc->setStorageMode(MTL::StorageModePrivate);
        densityTexture_ = device_->newTexture(texDesc);

        // Create blurred texture (Private - only used as intermediate)
        texDesc->setStorageMode(MTL::StorageModePrivate); 
        blurredTexture_ = device_->newTexture(texDesc);
        
        // Create temp blur texture (Managed - read by CPU for max density)
        texDesc->setStorageMode(MTL::StorageModeManaged);
        tempBlurTexture_ = device_->newTexture(texDesc);

        texDesc->release();

        // Check all three textures
        if (!densityTexture_ || !blurredTexture_ || !tempBlurTexture_) { 
             std::cerr << "Error: Failed to create density/blur textures!" << std::endl;
        } else {
             currentGridSize_ = {screenDimensions.x, screenDimensions.y}; 
             std::cout << "Recreated compute textures with size " << currentGridSize_.x << "x" << currentGridSize_.y << std::endl;
        }
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
    
    // Ensure finalFluidTexture_ matches screen dimensions
    if (!finalFluidTexture_ || 
        finalFluidTexture_->width() != screenDimensions.x || 
        finalFluidTexture_->height() != screenDimensions.y)
    {
        if (finalFluidTexture_) finalFluidTexture_->release();
        MTL::TextureDescriptor* texDesc = MTL::TextureDescriptor::alloc()->init();
        texDesc->setWidth(screenDimensions.x);
        texDesc->setHeight(screenDimensions.y);
        texDesc->setPixelFormat(MTL::PixelFormatBGRA8Unorm);
        texDesc->setStorageMode(MTL::StorageModeManaged);
        texDesc->setUsage(MTL::TextureUsageRenderTarget | MTL::TextureUsageShaderRead);
        finalFluidTexture_ = device_->newTexture(texDesc);
        texDesc->release();
        if (!finalFluidTexture_) {
            std::cerr << "Error: Failed to recreate final fluid texture!" << std::endl;
        }
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

    // 2. Determine Compute Grid Configuration - No longer needed here
    // uint2 computeGridSize = {200, 200};
    // Calculate grid parameters based on simulation space
    float simWidthMeters = static_cast<float>(coordinates.pixelsToMeters(screenDimensions.x));
    float cellSize = simWidthMeters / static_cast<float>(screenDimensions.x); // Use screenDimensions directly
    float2 gridOrigin = {0.0f, 0.0f}; 

    // 3. Ensure Metal resources
    ensureResources(particleCount);
    if (!particleBuffer_ || !paramsBuffer_ || !densityTexture_ || !blurredTexture_ || !tempBlurTexture_ ||
        !finalFluidTexture_ || !normalizedDensityTexture_) 
    {
         std::cerr << "Error: Missing required Metal resources for render." << std::endl;
        return; 
    }

    // 4. Update Buffers
    memcpy(particleBuffer_->contents(), cpuParticles.data(), sizeof(GPURenderFluidParticle) * particleCount);
    GPURenderParams params;
    params.gridSize = currentGridSize_; 
    params.cellSize = cellSize; 
    params.gridOrigin = gridOrigin;
    params.particleCount = particleCount;
    params.smoothingRadius = 10.0f;
    // maxDensity set later
    memcpy(paramsBuffer_->contents(), &params, sizeof(GPURenderParams));

    // --- Execute Compute & Render --- 
    MTL::CommandBuffer* cmdBuffer = commandQueue_->commandBuffer();
    
    // --- Compute Pass: Density & Blur ---
    MTL::ComputeCommandEncoder* compEnc = cmdBuffer->computeCommandEncoder();
    
    // 1. Density Kernel
    compEnc->setComputePipelineState(densityKernelPSO_);
    compEnc->setTexture(densityTexture_, 0);
    compEnc->setBuffer(particleBuffer_, 0, 0);
    compEnc->setBuffer(paramsBuffer_, 0, 1);
    MTL::Size densityGridDim = MTL::Size(currentGridSize_.x, currentGridSize_.y, 1);
    MTL::Size densityTgSize = MTL::Size(densityKernelPSO_->threadExecutionWidth(), densityKernelPSO_->maxTotalThreadsPerThreadgroup() / densityKernelPSO_->threadExecutionWidth(), 1);
    compEnc->dispatchThreads(densityGridDim, densityTgSize);

    // --- 2-Pass Blur ---
    MTL::Size blurGridDim = MTL::Size(currentGridSize_.x, currentGridSize_.y, 1);
    MTL::Size blurTgSize = MTL::Size(blurKernelPSO_->threadExecutionWidth(), blurKernelPSO_->maxTotalThreadsPerThreadgroup() / blurKernelPSO_->threadExecutionWidth(), 1);
    compEnc->setComputePipelineState(blurKernelPSO_);
    // Pass 1
    compEnc->setTexture(densityTexture_, 0);   
    compEnc->setTexture(blurredTexture_, 1);  
    compEnc->dispatchThreads(blurGridDim, blurTgSize);
    compEnc->memoryBarrier(MTL::BarrierScopeTextures);
    // Pass 2
    compEnc->setTexture(blurredTexture_, 0);  
    compEnc->setTexture(tempBlurTexture_, 1); 
    compEnc->dispatchThreads(blurGridDim, blurTgSize);
    
    compEnc->endEncoding(); 

    // --- Find Max Density on CPU ---
    cmdBuffer->commit(); 
    cmdBuffer->waitUntilCompleted();
    float maxDensityFound = 0.0f;
    if (tempBlurTexture_ && tempBlurTexture_->storageMode() == MTL::StorageModeManaged) {
        const size_t width = tempBlurTexture_->width();
        const size_t height = tempBlurTexture_->height();
        std::vector<float> cpuTextureData(width * height); 
        MTL::CommandBuffer* syncCmdBuffer = commandQueue_->commandBuffer();
        MTL::BlitCommandEncoder* blitEncoder = syncCmdBuffer->blitCommandEncoder();
        blitEncoder->synchronizeResource(tempBlurTexture_);
        blitEncoder->endEncoding();
        syncCmdBuffer->commit();
        syncCmdBuffer->waitUntilCompleted();
        MTL::Region region = MTL::Region::Make2D(0, 0, width, height);
        tempBlurTexture_->getBytes(cpuTextureData.data(), width * sizeof(float), region, 0);
        for(float val : cpuTextureData) {
            maxDensityFound = std::max(maxDensityFound, val);
        }
    } else {
        std::cerr << "Warning: Cannot find max density, tempBlurTexture_ is null or not Managed." << std::endl;
    }
    
    // --- Update Params Buffer with Max Density ---
    params.maxDensity = maxDensityFound;
    memcpy(paramsBuffer_->contents(), &params, sizeof(GPURenderParams));
    if (paramsBuffer_->storageMode() == MTL::StorageModeManaged) {
        paramsBuffer_->didModifyRange(NS::Range(0, sizeof(GPURenderParams)));
    }

    // --- Compute Pass: Normalization & Render Pass ---
    cmdBuffer = commandQueue_->commandBuffer(); 
    compEnc = cmdBuffer->computeCommandEncoder(); 

    // Dispatch Normalize Density Kernel
    compEnc->setComputePipelineState(normalizeDensityPSO_);
    compEnc->setTexture(tempBlurTexture_, 0); 
    compEnc->setTexture(normalizedDensityTexture_, 1); 
    compEnc->setBuffer(paramsBuffer_, 0, 0); 
    MTL::Size normGridDim = MTL::Size(currentGridSize_.x, currentGridSize_.y, 1); 
    MTL::Size normTgSize = MTL::Size(normalizeDensityPSO_->threadExecutionWidth(), normalizeDensityPSO_->maxTotalThreadsPerThreadgroup() / normalizeDensityPSO_->threadExecutionWidth(), 1);
    compEnc->dispatchThreads(normGridDim, normTgSize);

    compEnc->endEncoding(); 

    // --- Render Pass to Offscreen Texture ---
    MTL::RenderPassDescriptor* rpd = MTL::RenderPassDescriptor::renderPassDescriptor();
    if (!rpd) { /* Keep error */ return; }
    rpd->colorAttachments()->object(0)->setTexture(finalFluidTexture_);
    rpd->colorAttachments()->object(0)->setLoadAction(MTL::LoadActionClear);
    rpd->colorAttachments()->object(0)->setStoreAction(MTL::StoreActionStore);
    rpd->colorAttachments()->object(0)->setClearColor(MTL::ClearColor(0.0, 0.0, 0.0, 0.0)); // Transparent Black

    MTL::RenderCommandEncoder* renderEncoder = cmdBuffer->renderCommandEncoder(rpd);
    if (!renderEncoder) { /* Keep error */ return; }

    MTL::Viewport viewport = {0.0, 0.0, (double)finalFluidTexture_->width(), (double)finalFluidTexture_->height(), 0.0, 1.0 };
    renderEncoder->setViewport(viewport);

    renderEncoder->setRenderPipelineState(screenShaderPSO_);
    renderEncoder->setDepthStencilState(depthState_);

    renderEncoder->setFragmentTexture(normalizedDensityTexture_, 0); 
    sf::Color sfFluidColor(40, 130, 240, 255); 
    FluidFragmentParams fragmentParams = {
        .baseColor = { 
            (float)sfFluidColor.r / 255.0f, 
            (float)sfFluidColor.g / 255.0f, 
            (float)sfFluidColor.b / 255.0f, 
            (float)sfFluidColor.a / 255.0f 
        },
        .threshold = 0.19f,
        .smoothness = 0.02f,
    }; 
    renderEncoder->setFragmentBytes(&fragmentParams, sizeof(FluidFragmentParams), 0); 

    renderEncoder->drawPrimitives(MTL::PrimitiveTypeTriangle, NS::UInteger(0), NS::UInteger(3));

    renderEncoder->endEncoding();

    // --- Commit ---
    cmdBuffer->commit(); 
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
    }

    // Copy texture contents to buffer
    finalFluidTexture_->getBytes(outBuffer.data(), bytesPerRow, region, 0);

    // --- Swizzle R and B components (BGRA -> RGBA) ---
    for (size_t i = 0; i < bytesPerImage; i += 4) {
        // Bytes are B, G, R, A
        // Swap B (at i) and R (at i + 2)
        std::swap(outBuffer[i], outBuffer[i + 2]);
    }

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
