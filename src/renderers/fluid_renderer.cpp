/**
 * @file fluid_renderer.cpp
 * @brief Implementation of FluidRenderer.
 */

#include "renderers/fluid_renderer.hpp"
#include "entities/entity_components.hpp" // For component access
#include <iostream> // For warnings

FluidRenderer::FluidRenderer(Simulation::Coordinates& coords, sf::Vector2u screenDims)
    : coordinates(coords) // Store reference
{
    // Initialize the underlying FluidSurfaceRenderer
    surfaceRenderer = std::make_unique<FluidSurfaceRenderer>(screenDims, coordinates);
    // Initialize fallback metaball shader if needed (could be done here or lazy)
    // metaballShader = std::make_unique<sf::Shader>();
    // if (!metaballShader->loadFromFile(...)) { /* handle error */ }
}

FluidRenderer::~FluidRenderer() = default;

void FluidRenderer::render(sf::RenderTarget& target, const entt::registry& registry)
{
    // Remove verbose logging
    // std::cout << "FluidRenderer::render called." << std::endl;

    // Check if there are any liquid particles before doing potentially expensive work
    bool hasLiquidParticles = false;
    auto phaseView = registry.view<Components::ParticlePhase>();
    for (auto entity : phaseView) {
        if (phaseView.get<Components::ParticlePhase>(entity).phase == Components::Phase::Liquid) {
            hasLiquidParticles = true;
            break;
        }
    }
    if (!hasLiquidParticles) {
        // std::cout << "FluidRenderer: No liquid particles found." << std::endl;
        return;
    }
    // std::cout << "FluidRenderer: Liquid particles found." << std::endl;

    // Attempt to use the FluidSurfaceRenderer first
    bool surfaceRendered = false;
    if (surfaceRenderer) {
        // std::cout << "FluidRenderer: surfaceRenderer pointer is valid." << std::endl;
        // Directly try to update and render. FluidSurfaceRenderer::render
        // should internally check if it has valid resources (texture, shader).
        try {
            // Default parameters, can be made configurable
            int gridSize = 200;
            float smoothingRadius = 5.0f;
            // std::cout << "FluidRenderer: Calling updateGridAndTexture..." << std::endl;
            surfaceRenderer->updateGridAndTexture(registry, gridSize, smoothingRadius);

            sf::Color surfaceColor(40, 130, 240, 255);
            float densityThreshold = 0.10f;
            float edgeSmoothness = 0.02f;
            // std::cout << "FluidRenderer: Calling surfaceRenderer->render..." << std::endl;
            surfaceRenderer->render(target, surfaceColor, densityThreshold, edgeSmoothness);

            // Handle debug visualization drawing if needed (assuming FSR handles it)
            // If FSR has a separate renderDebug call, call it here based on its state
            /*
            if (surfaceRenderer->getCurrentDebugMode() != FluidSurfaceRenderer::DebugMode::NONE) {
                 // Assuming FSR draws its own debug view when render() is called in debug mode
                 // OR if it has a separate method:
                 // surfaceRenderer->renderDebug(target);
            }
            */
            surfaceRendered = true;
            // std::cout << "FluidRenderer: surfaceRenderer->render finished successfully." << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Error during FluidSurfaceRenderer execution: " << e.what() << std::endl;
            // Fallback might be attempted below
        }
    } else {
         std::cerr << "Error: FluidRenderer: surfaceRenderer pointer is NULL!" << std::endl;
    }

    // Fallback rendering (e.g., metaballs) if surface renderer failed or isn't available
    if (!surfaceRendered) {
        std::cerr << "Warning: FluidSurfaceRenderer failed or unavailable. Fluid fallback not implemented." << std::endl;
        // --- Implement fallback metaball rendering here if desired --- 
        // This would involve:
        // 1. Creating an sf::RenderTexture
        // 2. Drawing semi-transparent circles (metaballs) onto it with sf::BlendAdd
        // 3. Applying a threshold shader (like the old fluid.frag) to the texture
        // 4. Drawing the result sprite to the main target
    }
}

void FluidRenderer::toggleDebugVisualization() {
    if (surfaceRenderer) {
        int currentMode = static_cast<int>(surfaceRenderer->getCurrentDebugMode());
        int numModes = 3; // Assuming NONE, PRE_BLUR, POST_BLUR
        int nextMode = (currentMode + 1) % numModes;
        surfaceRenderer->setDebugMode(static_cast<FluidSurfaceRenderer::DebugMode>(nextMode));
        // std::cout << "FluidRenderer set debug mode: " << nextMode << std::endl;
    } else {
         // std::cout << "FluidRenderer: Cannot toggle debug, surface renderer not available." << std::endl;
    }
}

bool FluidRenderer::isDebugVisualization() const {
    return surfaceRenderer && surfaceRenderer->getCurrentDebugMode() != FluidSurfaceRenderer::DebugMode::NONE;
} 