/**
 * @file hourglass_comparison.cpp
 * @brief A scenario with two hourglasses side by side - one with fluid, 
 *        one with hexagons to compare their flow properties.
 */

#include <random>
#include <cmath>
#include <ctime>
#include <iostream>
#include <vector>

#include "scenarios/hourglasses.hpp"
#include "core/constants.hpp"
#include "entities/entity_components.hpp"
#include "math/polygon.hpp"

// Helper to create a static boundary wall or hourglass component
static void makeBoundaryPoly(
    entt::registry &registry,
    double cx, double cy,
    const std::vector<std::pair<double, double>> &localVertices,
    double staticFriction, double dynamicFriction,
    double wallMass)
{
    auto wallEnt = registry.create();
    registry.emplace<Components::Position>(wallEnt, cx, cy);
    registry.emplace<Components::Velocity>(wallEnt, 0.0, 0.0);
    registry.emplace<Components::Mass>(wallEnt, wallMass);

    // Mark as boundary + solid
    registry.emplace<Components::Boundary>(wallEnt);
    registry.emplace<Components::ParticlePhase>(wallEnt, Components::Phase::Solid);

    // Force asleep so it doesn't move
    auto &sleepC = registry.emplace<Components::Sleep>(wallEnt);
    sleepC.asleep = true;
    sleepC.sleepCounter = 9999999;

    // Material friction
    registry.emplace<Components::Material>(wallEnt, staticFriction, dynamicFriction);

    // Build a polygon shape
    PolygonShape poly;
    poly.type = Components::ShapeType::Polygon;
    for (const auto &point : localVertices) {
        poly.vertices.emplace_back(point.first, point.second);
    }

    // Find maximum radius for bounding circle
    double maxR = 0.0;
    for (const auto &point : localVertices) {
        double dist = std::sqrt(point.first*point.first + point.second*point.second);
        if (dist > maxR) {
            maxR = dist;
        }
    }

    registry.emplace<Components::Shape>(wallEnt, Components::ShapeType::Polygon, maxR);
    registry.emplace<PolygonShape>(wallEnt, poly);
    registry.emplace<Components::AngularPosition>(wallEnt, 0.0);
    
    // Add color
    registry.emplace<Components::Color>(wallEnt, 128, 128, 128);
}

// Helper to create a regular hexagon with counter-clockwise winding
static std::vector<std::pair<double, double>> createHexagonVertices(double size) {
    std::vector<std::pair<double, double>> vertices;
    const int n = 6; // Hexagon
    
    // For CCW winding with a Y-axis that increases downward (standard screen coordinates),
    // we need to go counterclockwise in math coordinates
    for (int i = 0; i < n; i++) {
        // Start at angle 0 and move counterclockwise
        double angle = 2.0 * M_PI * (n - i - 1) / n;
        double x = size * std::cos(angle);
        double y = size * std::sin(angle);
        vertices.emplace_back(x, y);
    }
    
    return vertices;
}

ScenarioSystemConfig HourglassesScenario::getSystemsConfig() const
{
    ScenarioSystemConfig config;
    config.sharedConfig.MetersPerPixel = 1e-2;
    config.sharedConfig.UniverseSizeMeters = SimulatorConstants::ScreenLength * config.sharedConfig.MetersPerPixel;
    config.sharedConfig.SecondsPerTick = 1.0 / SimulatorConstants::StepsPerSecond;
    config.sharedConfig.TimeAcceleration = 1.0;
    config.sharedConfig.GridSize = 50;
    config.sharedConfig.CellSizePixels = static_cast<double>(SimulatorConstants::ScreenLength) / config.sharedConfig.GridSize;

    config.sharedConfig.GravitationalSoftener = 0.0;
    config.sharedConfig.CollisionCoeffRestitution = 0.2; // slight bounce for rigid shapes
    config.sharedConfig.DragCoeff = 0.0;
    config.sharedConfig.ParticleDensity = 100.0;

    return config;
}

void HourglassesScenario::createHourglass(
    entt::registry &registry, 
    double centerX, 
    double centerY) const 
{
    // Calculate dimensions
    double height = scenarioEntityConfig.hourglassHeight;
    double topWidth = scenarioEntityConfig.hourglassTopWidth;
    double neckWidth = scenarioEntityConfig.hourglassNeckWidth;
    double wallThickness = scenarioEntityConfig.hourglassWallThickness;
    
    // Create a small overlap factor to ensure polygons connect seamlessly
    double overlap = 0.03;
    
    // Left wall - with slight overlap at connections
    std::vector<std::pair<double, double>> leftWallPoints = {
        // Start at top and go CCW (in screen coords with Y increasing downward)
        {-topWidth/2, -height/2 - overlap},                 // Top left inner (extend up)
        {-(topWidth/2 + wallThickness), -height/2 - overlap}, // Top left outer (extend up)
        {-(neckWidth/2 + wallThickness), 0},                // Middle left outer
        {-(topWidth/2 + wallThickness), height/2 + overlap}, // Bottom left outer (extend down)
        {-topWidth/2, height/2 + overlap},                  // Bottom left inner (extend down)
        {-neckWidth/2, 0},                                  // Middle left inner
        {-topWidth/2, -height/2 - overlap}                  // Close the polygon
    };
    
    // Right wall - with slight overlap at connections
    std::vector<std::pair<double, double>> rightWallPoints = {
        // Start at top and go CCW
        {topWidth/2, -height/2 - overlap},                  // Top right inner (extend up)
        {neckWidth/2, 0},                                   // Middle right inner
        {topWidth/2, height/2 + overlap},                   // Bottom right inner (extend down)
        {(topWidth/2 + wallThickness), height/2 + overlap}, // Bottom right outer (extend down)
        {(neckWidth/2 + wallThickness), 0},                 // Middle right outer
        {(topWidth/2 + wallThickness), -height/2 - overlap}, // Top right outer (extend up)
        {topWidth/2, -height/2 - overlap}                   // Close the polygon
    };
    
    // Create left and right walls
    makeBoundaryPoly(registry, centerX, centerY, leftWallPoints, 
                    scenarioEntityConfig.wallStaticFriction, 
                    scenarioEntityConfig.wallDynamicFriction,
                    scenarioEntityConfig.wallMass);
                    
    makeBoundaryPoly(registry, centerX, centerY, rightWallPoints, 
                    scenarioEntityConfig.wallStaticFriction, 
                    scenarioEntityConfig.wallDynamicFriction,
                    scenarioEntityConfig.wallMass);
    
    // Top cap in CCW order with slight overlap
    std::vector<std::pair<double, double>> topCapPoints = {
        {-topWidth/2 - wallThickness, -height/2 - wallThickness}, // Top left
        {-topWidth/2 - wallThickness, -height/2 + overlap},       // Bottom left (extended down)
        {topWidth/2 + wallThickness, -height/2 + overlap},        // Bottom right (extended down)
        {topWidth/2 + wallThickness, -height/2 - wallThickness},  // Top right
    };
    
    // Bottom cap in CCW order with slight overlap
    std::vector<std::pair<double, double>> bottomCapPoints = {
        {-topWidth/2 - wallThickness, height/2 - overlap},        // Top left (extended up)
        {-topWidth/2 - wallThickness, height/2 + wallThickness},  // Bottom left
        {topWidth/2 + wallThickness, height/2 + wallThickness},   // Bottom right
        {topWidth/2 + wallThickness, height/2 - overlap},         // Top right (extended up)
    };
    
    makeBoundaryPoly(registry, centerX, centerY, topCapPoints, 
                    scenarioEntityConfig.wallStaticFriction, 
                    scenarioEntityConfig.wallDynamicFriction,
                    scenarioEntityConfig.wallMass);
                    
    makeBoundaryPoly(registry, centerX, centerY, bottomCapPoints, 
                    scenarioEntityConfig.wallStaticFriction, 
                    scenarioEntityConfig.wallDynamicFriction,
                    scenarioEntityConfig.wallMass);
}

void HourglassesScenario::createEntities(entt::registry &registry) const
{
    ScenarioSystemConfig scenarioSystemConfig = getSystemsConfig();
    SharedSystemConfig sharedConfig = scenarioSystemConfig.sharedConfig;
    double sizeM = sharedConfig.UniverseSizeMeters;
    
    // Create two hourglasses - one on the left, one on the right
    double leftHourglassX = sizeM * 0.3;  // Left side
    double rightHourglassX = sizeM * 0.7; // Right side
    double hourglassY = sizeM * 0.5;      // Center vertically
    
    createHourglass(registry, leftHourglassX, hourglassY);  // Fluid hourglass
    createHourglass(registry, rightHourglassX, hourglassY); // Hexagon hourglass
    
    // Random number generation
    std::default_random_engine generator(static_cast<unsigned>(std::time(nullptr)));
    std::uniform_real_distribution<double> jitterDist(-0.05, 0.05);
    
    // 1. Create fluid particles in the left hourglass (top half)
    {
        int numFluid = scenarioEntityConfig.fluidParticleCount;
        double height = scenarioEntityConfig.hourglassHeight;
        double topWidth = scenarioEntityConfig.hourglassTopWidth;
        double neckWidth = scenarioEntityConfig.hourglassNeckWidth;
        
        // Define the top chamber region (with margin to avoid walls)
        double margin = topWidth * 0.15; // Proportional margin based on hourglass size
        double x_min = leftHourglassX - (topWidth/2) + margin;
        double x_max = leftHourglassX + (topWidth/2) - margin;
        double y_min = hourglassY - (height/2) + margin;
        double y_max = hourglassY - 0.1; // Only fill top half, leave space from neck
        
        double regionWidth = x_max - x_min;
        double regionHeight = y_max - y_min;
        
        // Calculate better distribution based on aspect ratio
        double aspectRatio = regionWidth / regionHeight;
        int nRows = static_cast<int>(std::sqrt(numFluid / aspectRatio));
        if (nRows < 1) nRows = 1;
        int nCols = (numFluid + nRows - 1) / nRows; // Ceiling division
        
        double dx = regionWidth / (nCols + 1);
        double dy = regionHeight / (nRows + 1);

        // Calculate the slope of the trapezoidal sides (m in y = mx + b)
        double halfTopWidth = (topWidth / 2) - margin;  // Account for margin
        double halfNeckWidth = neckWidth / 2;
        double chamberHeight = (height / 2) - margin;   // Only top half
        
        // For the left and right edges of the trapezoid:
        // Left edge: x = leftHourglassX - halfTopWidth + (y - y_min) * slopeLeft
        // Right edge: x = leftHourglassX + halfTopWidth - (y - y_min) * slopeRight
        double slope = (halfTopWidth - halfNeckWidth) / chamberHeight;
        
        int count = 0;
        for (int row = 0; row < nRows && count < numFluid; row++) {
            double y = y_min + (row + 1) * dy;
            
            // Calculate the width at this row's y position (trapezoid gets narrower as y increases)
            double progress = (y - y_min) / chamberHeight;  // 0 at top, approaches 1 near neck
            double halfWidthAtRow = halfTopWidth - progress * (halfTopWidth - halfNeckWidth);
            
            // Calculate x bounds for this row
            double rowXmin = leftHourglassX - halfWidthAtRow + margin;
            double rowXmax = leftHourglassX + halfWidthAtRow - margin;
            double rowWidth = rowXmax - rowXmin;
            
            // Skip row if it's too close to neck or outside the chamber
            if (rowWidth < 2 * margin) continue;
            
            // Calculate spacing within this row
            int colsInRow = static_cast<int>((rowWidth / regionWidth) * nCols);
            if (colsInRow < 1) colsInRow = 1;
            double rowDx = rowWidth / (colsInRow + 1);
            
            for (int col = 0; col < colsInRow && count < numFluid; col++) {
                double jitterX = jitterDist(generator) * rowDx * 0.3; // Reduced jitter
                double jitterY = jitterDist(generator) * dy * 0.3; // Reduced jitter
                double x = rowXmin + (col + 1) * rowDx + jitterX;
                double adjustedY = y + jitterY;
                
                auto e = registry.create();
                registry.emplace<Components::Position>(e, x, adjustedY);
                registry.emplace<Components::Velocity>(e, 0.0, 0.0);
                registry.emplace<Components::Mass>(e, scenarioEntityConfig.fluidParticleMass);
                registry.emplace<Components::ParticlePhase>(e, Components::Phase::Liquid);
                
                // No friction for fluid particles
                registry.emplace<Components::Material>(e, 0.0, 0.0);
                
                // Use a circle shape - size must allow passing through neck
                double r = scenarioEntityConfig.fluidParticleSize / 2.0; // Use size from config (convert diameter to radius)
                registry.emplace<Components::Shape>(e, Components::ShapeType::Circle, r);
                registry.emplace<CircleShape>(e, r);
                
                // SPH-related properties
                registry.emplace<Components::SmoothingLength>(e, 0.06);
                registry.emplace<Components::SpeedOfSound>(e, 1000.0);
                registry.emplace<Components::SPHTemp>(e);
                
                // Blue color
                registry.emplace<Components::Color>(e, 20, 100, 220);
                
                count++;
            }
        }
        
        // If we didn't create enough particles with the trapezoidal approach,
        // fill the remaining count with a simpler grid approach near the top
        if (count < numFluid) {
            int remaining = numFluid - count;
            
            // Use the top 1/3 of the chamber for the remaining particles
            double fillY_min = y_min;
            double fillY_max = y_min + regionHeight * 0.33;
            double fillWidth = (leftHourglassX + halfTopWidth - margin) - (leftHourglassX - halfTopWidth + margin);
            
            // Simple grid distribution for remaining particles
            int fillCols = static_cast<int>(sqrt(remaining));
            int fillRows = (remaining + fillCols - 1) / fillCols;
            
            double fillDx = fillWidth / (fillCols + 1);
            double fillDy = (fillY_max - fillY_min) / (fillRows + 1);
            
            for (int row = 0; row < fillRows && count < numFluid; row++) {
                for (int col = 0; col < fillCols && count < numFluid; col++) {
                    double x = leftHourglassX - halfTopWidth + margin + (col + 1) * fillDx;
                    double y = fillY_min + (row + 1) * fillDy;
                    
                    // Add small jitter to prevent perfect grid alignment
                    double jitterX = jitterDist(generator) * fillDx * 0.2;
                    double jitterY = jitterDist(generator) * fillDy * 0.2;
                    
                    auto e = registry.create();
                    registry.emplace<Components::Position>(e, x + jitterX, y + jitterY);
                    registry.emplace<Components::Velocity>(e, 0.0, 0.0);
                    registry.emplace<Components::Mass>(e, scenarioEntityConfig.fluidParticleMass);
                    registry.emplace<Components::ParticlePhase>(e, Components::Phase::Liquid);
                    
                    // No friction for fluid particles
                    registry.emplace<Components::Material>(e, 0.0, 0.0);
                    
                    // Use a circle shape
                    double r = scenarioEntityConfig.fluidParticleSize / 2.0;
                    registry.emplace<Components::Shape>(e, Components::ShapeType::Circle, r);
                    registry.emplace<CircleShape>(e, r);
                    
                    // SPH-related properties
                    registry.emplace<Components::SmoothingLength>(e, 0.06);
                    registry.emplace<Components::SpeedOfSound>(e, 1000.0);
                    registry.emplace<Components::SPHTemp>(e);
                    
                    // Blue color
                    registry.emplace<Components::Color>(e, 20, 100, 220);
                    
                    count++;
                }
            }
        }
        
        std::cerr << "Created " << count << " fluid particles in left hourglass.\n";
    }
    
    // 2. Create hexagon particles in the right hourglass (top half)
    {
        int numHexagons = scenarioEntityConfig.hexagonCount;
        double height = scenarioEntityConfig.hourglassHeight;
        double topWidth = scenarioEntityConfig.hourglassTopWidth;
        double neckWidth = scenarioEntityConfig.hourglassNeckWidth;
        
        // Ensure hexagons fit properly
        double hexSize = scenarioEntityConfig.hexagonSize;
        double margin = topWidth * 0.15; // Proportional margin for better distribution
        double x_min = rightHourglassX - (topWidth/2) + margin;
        double x_max = rightHourglassX + (topWidth/2) - margin;
        double y_min = hourglassY - (height/2) + margin;
        double y_max = hourglassY - hexSize; // Leave space from the neck
        
        double regionWidth = x_max - x_min;
        double regionHeight = y_max - y_min;
        
        // Calculate better distribution based on aspect ratio
        double aspectRatio = regionWidth / regionHeight;
        int nRows = static_cast<int>(std::sqrt(numHexagons / aspectRatio));
        if (nRows < 1) nRows = 1;
        int nCols = (numHexagons + nRows - 1) / nRows; // Ceiling division
        
        double dx = regionWidth / (nCols + 1);
        double dy = regionHeight / (nRows + 1);

        // Calculate the slope of the trapezoidal sides (similar to fluid calculation)
        double halfTopWidth = (topWidth / 2) - margin;
        double halfNeckWidth = neckWidth / 2;
        double chamberHeight = (height / 2) - margin;
        double slope = (halfTopWidth - halfNeckWidth) / chamberHeight;
        
        // Create vertices for a regular hexagon
        auto hexVerts = createHexagonVertices(hexSize);
        
        // Color distribution
        std::uniform_int_distribution<int> colorDist(100, 200);
        
        int count = 0;
        for (int row = 0; row < nRows && count < numHexagons; row++) {
            double y = y_min + (row + 1) * dy;
            
            // Calculate the width at this row's y position (trapezoid gets narrower as y increases)
            double progress = (y - y_min) / chamberHeight;
            double halfWidthAtRow = halfTopWidth - progress * (halfTopWidth - halfNeckWidth);
            
            // Calculate x bounds for this row
            double rowXmin = rightHourglassX - halfWidthAtRow + hexSize;  // Add hexSize for better margin
            double rowXmax = rightHourglassX + halfWidthAtRow - hexSize;
            double rowWidth = rowXmax - rowXmin;
            
            // Skip row if it's too narrow to fit hexagons
            if (rowWidth < 2 * hexSize) continue;
            
            // Calculate spacing within this row
            int colsInRow = static_cast<int>((rowWidth / regionWidth) * nCols);
            if (colsInRow < 1) colsInRow = 1;
            double rowDx = rowWidth / (colsInRow + 1);
            
            for (int col = 0; col < colsInRow && count < numHexagons; col++) {
                double jitterX = jitterDist(generator) * rowDx * 0.2; // Minimal jitter for hexagons
                double jitterY = jitterDist(generator) * dy * 0.2;
                double x = rowXmin + (col + 1) * rowDx + jitterX;
                double adjustedY = y + jitterY;
                
                auto e = registry.create();
                registry.emplace<Components::Position>(e, x, adjustedY);
                registry.emplace<Components::Velocity>(e, 0.0, 0.0);
                registry.emplace<Components::Mass>(e, scenarioEntityConfig.hexagonMass);
                registry.emplace<Components::ParticlePhase>(e, Components::Phase::Solid);
                registry.emplace<Components::Material>(e, 
                                                     scenarioEntityConfig.polyStaticFriction, 
                                                     scenarioEntityConfig.polyDynamicFriction);
                
                // Sleep initially to avoid instability when packed densely
                auto &sleepC = registry.emplace<Components::Sleep>(e);
                sleepC.asleep = false; // Allow falling when simulation starts
                
                // Create polygon shape
                PolygonShape poly;
                poly.type = Components::ShapeType::Polygon;
                for (const auto &point : hexVerts) {
                    poly.vertices.emplace_back(point.first, point.second);
                }
                
                registry.emplace<Components::Shape>(e, Components::ShapeType::Polygon, hexSize);
                registry.emplace<PolygonShape>(e, poly);
                
                // Calculate moment of inertia
                double I = calculatePolygonInertia(poly.vertices, scenarioEntityConfig.hexagonMass);
                registry.emplace<Components::Inertia>(e, I);
                
                // Angular components
                registry.emplace<Components::AngularPosition>(e, 0.0);
                registry.emplace<Components::AngularVelocity>(e, 0.0);
                
                // Random orange/yellow color
                int r = colorDist(generator) + 55;  // 155-255 (orange/yellow range)
                int g = colorDist(generator) - 50;  // 50-150 (less green)
                int b = 30;                         // Low blue for orange/yellow
                registry.emplace<Components::Color>(e, r, g, b);
                
                count++;
            }
        }
        std::cerr << "Created " << count << " hexagons in right hourglass.\n";
    }
    
    std::cerr << "Hourglass comparison scenario ready.\n";
} 