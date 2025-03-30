/**
 * @file renderer_fluid_dsf.cpp
 * @brief Implementation of screen-space fluid renderer
 * 
 * Uses a blurred density field and a screen-space shader.
 */

#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>

#include "arch/native/renderer_fluid_dsf.hpp"
#include "entities/entity_components.hpp"
#include "math/constants.hpp"

// Constructor
FluidSurfaceRenderer::FluidSurfaceRenderer(sf::Vector2u resolution, const Simulation::Coordinates& coordinates)
    : resolution(resolution)
    , coordinates(coordinates)
    , lastGridSize(0)
    , lastRadius(0.0f)
{
    // Create density texture 
    if (!densityTexture.create(resolution.x, resolution.y)) {
         std::cerr << "Error: Could not create density texture!" << std::endl;
         // Handle error appropriately - maybe set an initialization flag to false
    }
    densityTexture.setSmooth(true); // Enable bilinear filtering
    
    // Optional: Keep depth texture if future effects need it
    // depthTexture.create(resolution.x, resolution.y);
    
    initializeShader();
}

// Destructor
FluidSurfaceRenderer::~FluidSurfaceRenderer() = default;

// Shader Initializer
void FluidSurfaceRenderer::initializeShader() {
    if (sf::Shader::isAvailable()) {
        fluidShader = std::make_unique<sf::Shader>();
        // Load the dedicated screen-space shader
        if (!fluidShader->loadFromFile("shaders/fluid_screenspace.frag", sf::Shader::Fragment)) {
            std::cerr << "Warning: Failed to load screen-space fluid shader (fluid_screenspace.frag). Fluid won't render correctly.\n";
            fluidShader.reset();
        }
    } else {
        std::cerr << "Warning: Shaders not supported. Fluid won't render correctly.\n";
    }
}

// Update Grid and Texture (formerly updateMesh)
void FluidSurfaceRenderer::updateGridAndTexture(const entt::registry& registry, int gridSize, float smoothingRadius) {
    // --- Optimization: Check if parameters changed --- 
    // Note: Could add a check here to see if particles actually moved significantly
    //       but for simplicity, we update every frame for now.
    bool paramsChanged = (lastGridSize != gridSize || 
                         std::abs(lastRadius - smoothingRadius) > 0.0001f);
                         
    if (!paramsChanged && grid.size() == static_cast<size_t>(gridSize)) {
       // Optimization: If params are same and grid size matches, reuse grid potentially?
       // For now, let's recalculate fully each time for simplicity & correctness
    }
                         
    lastGridSize = gridSize;
    lastRadius = smoothingRadius;
    
    // 1. Generate density onto the grid
    generateDensityGrid(registry, gridSize, smoothingRadius);
    
    // 2. Blur the grid data
    blurDensityGrid(); // Apply blur to smooth it out
    
    // 3. Transfer blurred grid data to the density texture for the shader
    updateDensityTexture(); 
}

// Generate Density Grid - Simplified to only store density
void FluidSurfaceRenderer::generateDensityGrid(const entt::registry& registry, int gridSize, float smoothingRadius) {
    // Debug info
    static int debugCounter = 0;
    bool verbose = (debugCounter++ % 100 == 0);
    int particleCount = 0;
    
    // Resize grid if necessary
    if (grid.size() != static_cast<size_t>(gridSize) || 
        (grid.size() > 0 && grid[0].size() != static_cast<size_t>(gridSize))) {
        grid.resize(gridSize);
        for (auto& row : grid) {
            row.resize(gridSize, GridCell{}); // Ensure new cells are initialized
        }
        // Also resize debug grid if necessary
        if (currentDebugMode != DebugMode::NONE) {
             if (debugDensityGrid.size() != static_cast<size_t>(gridSize) || 
                 (debugDensityGrid.size() > 0 && debugDensityGrid[0].size() != static_cast<size_t>(gridSize))) {
                 debugDensityGrid.resize(gridSize, std::vector<float>(gridSize, 0.0f));
             }
        }
    }
    
    // Clear grid densities (and debug grid if active)
    for (int y = 0; y < gridSize; ++y) {
        for (int x = 0; x < gridSize; ++x) {
            grid[y][x].density = 0.0f;
            if (currentDebugMode == DebugMode::DENSITY_PRE_BLUR) {
                 debugDensityGrid[y][x] = 0.0f;
            }
        }
    }
    
    auto view = registry.view<Components::Position, Components::Shape, Components::ParticlePhase>();
    float cellSize = static_cast<float>(resolution.x) / gridSize;
    float influenceRadiusSq = (cellSize * smoothingRadius) * (cellSize * smoothingRadius);

    for (auto entity : view) {
        const auto& phase = view.get<Components::ParticlePhase>(entity);
        if (phase.phase != Components::Phase::Liquid) continue;
        particleCount++;
        
        const auto& pos = view.get<Components::Position>(entity);
        const auto& shape = view.get<Components::Shape>(entity);
        if (shape.type != Components::ShapeType::Circle || shape.size <= 0.0) continue;
        
        float px = static_cast<float>(coordinates.metersToPixels(pos.x));
        float py = static_cast<float>(coordinates.metersToPixels(pos.y));
        // Use pixel-space radius for influence calculation
        float h = static_cast<float>(coordinates.metersToPixels(shape.size)) * smoothingRadius; 
        if (h < 1.0f) h = 1.0f; // Ensure minimum influence
        float hSq = h * h;
        
        int minX = std::max(0, static_cast<int>((px - h) / cellSize));
        int maxX = std::min(gridSize - 1, static_cast<int>((px + h) / cellSize));
        int minY = std::max(0, static_cast<int>((py - h) / cellSize));
        int maxY = std::min(gridSize - 1, static_cast<int>((py + h) / cellSize));
        
        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                float cx = (x + 0.5f) * cellSize;
                float cy = (y + 0.5f) * cellSize;
                float dx = cx - px;
                float dy = cy - py;
                float distSq = dx*dx + dy*dy;
                
                if (distSq < hSq) { 
                    float dist = std::sqrt(distSq);
                    float weight = kernelPoly6(dist, h);
                    grid[y][x].density += weight;
                    // Store pre-blur density for debugging
                    if (currentDebugMode == DebugMode::DENSITY_PRE_BLUR) {
                        debugDensityGrid[y][x] += weight; 
                    }
                }
            }
        }
    }
    
    if (verbose) {
        std::cerr << "Found " << particleCount << " liquid particles for density grid generation" << std::endl;
    }
}

// Blur Density Grid (Implementation remains the same)
void FluidSurfaceRenderer::blurDensityGrid() {
    if (grid.empty() || grid[0].empty()) return;
    int gridSize = static_cast<int>(grid.size());
    std::vector<std::vector<float>> blurredDensities(gridSize, std::vector<float>(gridSize, 0.0f));

    // Box Blur (could replace with Gaussian for better quality)
    for (int y = 0; y < gridSize; ++y) {
        for (int x = 0; x < gridSize; ++x) {
            float sum = 0.0f;
            int count = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int ny = y + dy;
                    int nx = x + dx;
                    if (nx >= 0 && nx < gridSize && ny >= 0 && ny < gridSize) {
                        sum += grid[ny][nx].density;
                        count++;
                    }
                }
            }
            blurredDensities[y][x] = (count > 0) ? (sum / count) : 0.0f;
        }
    }

    // Store post-blur densities if requested & apply back to grid
    if (currentDebugMode == DebugMode::DENSITY_POST_BLUR) {
         if (debugDensityGrid.size() != static_cast<size_t>(gridSize) || 
             (debugDensityGrid.size() > 0 && debugDensityGrid[0].size() != static_cast<size_t>(gridSize))) {
             debugDensityGrid.resize(gridSize, std::vector<float>(gridSize, 0.0f));
         }
    }
    for (int y = 0; y < gridSize; ++y) {
        for (int x = 0; x < gridSize; ++x) {
            grid[y][x].density = blurredDensities[y][x];
            if (currentDebugMode == DebugMode::DENSITY_POST_BLUR) {
                debugDensityGrid[y][x] = blurredDensities[y][x];
            }
        }
    }
}

// Update Density Texture from Grid Data by drawing onto it
void FluidSurfaceRenderer::updateDensityTexture() {
    if (grid.empty() || grid[0].empty() || !densityTexture.getSize().x) return;

    int gridSize = static_cast<int>(grid.size());
    // Ensure texture size matches grid size
    if (densityTexture.getSize().x != static_cast<unsigned int>(gridSize) || 
        densityTexture.getSize().y != static_cast<unsigned int>(gridSize)) {
        if (!densityTexture.create(gridSize, gridSize)) {
            std::cerr << "Error: Failed to resize density texture!" << std::endl;
            return;
        }
         densityTexture.setSmooth(true);
    }
    
    // Clear the texture (important!)
    densityTexture.clear(sf::Color::Transparent); 

    // --- Normalize density (same logic as before) --- 
    float maxDensity = 0.0f;
     for (const auto& row : grid) {
         for (const auto& cell : row) {
             maxDensity = std::max(maxDensity, cell.density);
         }
     }
     float densityScale = (maxDensity > 0.1f) ? (1.0f / maxDensity) : 1.0f; 
     // --- End Normalization ---

    // Prepare a rectangle shape for drawing cells
    sf::RectangleShape cellShape(sf::Vector2f(1.0f, 1.0f)); // Draw 1x1 pixels

    // Loop through grid and draw each cell's density as a pixel color
    for (int y = 0; y < gridSize; ++y) {
        for (int x = 0; x < gridSize; ++x) {
            float density = grid[y][x].density;
            float scaledDensity = std::clamp(density * densityScale, 0.0f, 1.0f);
            sf::Uint8 intensity = static_cast<sf::Uint8>(scaledDensity * 255.0f);
            
            // Set color (RGBA) - Density in Alpha, grayscale in RGB for potential debug view
            cellShape.setFillColor(sf::Color(intensity, intensity, intensity, intensity));
            cellShape.setPosition(static_cast<float>(x), static_cast<float>(y));
            
            // Draw onto the render texture
            densityTexture.draw(cellShape); 
        }
    }
    
    // Finalize the texture
    densityTexture.display();
}

// Kernel Function (remains the same)
float FluidSurfaceRenderer::kernelPoly6(float r, float h) {
    if (r < 0.0f || r >= h) return 0.0f;
    float hSq = h * h;
    float rSq = r * r;
    float term = hSq - rSq;
    // Using the standard Poly6 normalization factor for 2D (4 / (pi * h^8)) is complex
    // For rendering, often just the shape term^3 is enough, scale via densityScale later
    // return (4.0f / (MathConstants::PI * std::pow(h, 8.0f))) * term * term * term; 
    return term * term * term; // Simpler form, relies on external scaling
}

// Render Function (Screen-Space)
void FluidSurfaceRenderer::render(sf::RenderTarget& target, const sf::Color& surfaceColor, 
                                float densityThreshold, float edgeSmoothness) {
    
    // --- Debug Visualization Logic ---
    if (currentDebugMode == DebugMode::DENSITY_PRE_BLUR || currentDebugMode == DebugMode::DENSITY_POST_BLUR) {
        if (debugDensityGrid.empty() || debugDensityGrid[0].empty()) return;
        int gridSize = static_cast<int>(debugDensityGrid.size());
        float cellSize = static_cast<float>(resolution.x) / gridSize; 
        sf::RectangleShape cellShape(sf::Vector2f(cellSize, cellSize));
        float maxDensity = 0.0f;
        for (const auto& row : debugDensityGrid) for (float d : row) maxDensity = std::max(maxDensity, d);
        if (maxDensity < 0.01f) maxDensity = 1.0f; 
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                float normDensity = std::clamp(debugDensityGrid[y][x] / maxDensity, 0.0f, 1.0f);
                sf::Uint8 intensity = static_cast<sf::Uint8>(normDensity * 255.0f);
                cellShape.setFillColor(sf::Color(intensity, intensity, intensity, 180)); // Semi-transparent
                cellShape.setPosition(x * cellSize, y * cellSize);
                target.draw(cellShape);
            }
        }
        return; 
    }
    // --- End Debug Visualization Logic ---

    // --- Normal Screen-Space Rendering ---
    if (!fluidShader || !fluidShader->isAvailable() || !densityTexture.getSize().x) {
        // Maybe log this warning less frequently
        // std::cerr << "Warning: Fluid shader or density texture unavailable." << std::endl;
        return; 
    }
    
    // Draw a fullscreen sprite textured with the density map
    // Note: Assumes target origin is (0,0) and covers simulation area directly
    sf::Sprite fluidSprite(densityTexture.getTexture());
    // Ensure sprite covers the simulation area correctly (no scaling needed if texture=sim area)
    fluidSprite.setTextureRect(sf::IntRect(0, 0, 
        static_cast<int>(densityTexture.getSize().x),
        static_cast<int>(densityTexture.getSize().y))); 
    fluidSprite.setScale(
        static_cast<float>(resolution.x) / densityTexture.getSize().x,
        static_cast<float>(resolution.y) / densityTexture.getSize().y
    );

    // Set shader uniforms
    fluidShader->setUniform("densityTexture", densityTexture.getTexture());
    fluidShader->setUniform("resolution", sf::Vector2f(static_cast<float>(resolution.x), static_cast<float>(resolution.y)));
    fluidShader->setUniform("baseColor", sf::Glsl::Vec4(surfaceColor));
    fluidShader->setUniform("threshold", densityThreshold);
    fluidShader->setUniform("smoothness", edgeSmoothness);
    fluidShader->setUniform("time", static_cast<float>(clock() % CLOCKS_PER_SEC) / CLOCKS_PER_SEC); // Optional
    
    sf::RenderStates states;
    states.shader = fluidShader.get();
    states.blendMode = sf::BlendAlpha; // Use alpha blending
    
    target.draw(fluidSprite, states);
} 