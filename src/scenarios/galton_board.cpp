/**
 * @file galton_board.cpp
 * @brief Implementation of a carefully-designed Galton board (bean machine) scenario.
 * 
 * This scenario features a proper triangular grid of pegs with proportional
 * measurements to create a correct binomial probability distribution. Particles
 * are inserted gradually to maintain stability, and the entire geometry is
 * constructed to ensure proper alignment between pegs, bins, and particles.
 */

#include <cmath>
#include <ctime>
#include <iostream>
#include <random>
#include <vector>
#include <utility> // For std::pair

#include "scenarios/galton_board.hpp"
#include "core/constants.hpp"
#include "entities/entity_components.hpp"
#include "math/polygon.hpp"

// Helper to create a static peg (circle) entity
void GaltonBoardScenario::makePeg(entt::registry &registry,
                                  double cx, double cy, double radius,
                                  double friction, double restitution,
                                  int r, int g, int b) const
{
    auto peg = registry.create();
    registry.emplace<Components::Position>(peg, cx, cy);
    registry.emplace<Components::Velocity>(peg, 0.0, 0.0); // Static
    registry.emplace<Components::Mass>(peg, 1e30); // Infinite mass
    registry.emplace<Components::ParticlePhase>(peg, Components::Phase::Solid);
    registry.emplace<Components::Boundary>(peg); // Mark as boundary

    // Shape and material
    registry.emplace<Components::Shape>(peg, Components::ShapeType::Circle, radius);
    registry.emplace<CircleShape>(peg, radius);
    registry.emplace<Components::Material>(peg, friction, friction);

    // Color
    registry.emplace<Components::Color>(peg, r, g, b);

    // Force asleep
    auto &sleepC = registry.emplace<Components::Sleep>(peg);
    sleepC.asleep = true;
    sleepC.sleepCounter = 9999999;
}

// Helper to create a static polygon boundary
void GaltonBoardScenario::makeStaticPoly(entt::registry &registry,
                                         double cx, double cy,
                                         const std::vector<std::pair<double, double>> &localVertices,
                                         double friction, double restitution,
                                         int r, int g, int b) const
{
    auto wallEnt = registry.create();
    registry.emplace<Components::Position>(wallEnt, cx, cy);
    registry.emplace<Components::Velocity>(wallEnt, 0.0, 0.0);
    registry.emplace<Components::Mass>(wallEnt, 1e30); // Infinite mass

    registry.emplace<Components::Boundary>(wallEnt);
    registry.emplace<Components::ParticlePhase>(wallEnt, Components::Phase::Solid);

    auto &sleepC = registry.emplace<Components::Sleep>(wallEnt);
    sleepC.asleep = true;
    sleepC.sleepCounter = 9999999;

    registry.emplace<Components::Material>(wallEnt, friction, friction);

    PolygonShape poly;
    poly.type = Components::ShapeType::Polygon;
    double maxR = 0.0;
    for (const auto &point : localVertices) {
        poly.vertices.emplace_back(point.first, point.second);
        double distSq = point.first * point.first + point.second * point.second;
        if (distSq > maxR * maxR) {
            maxR = std::sqrt(distSq);
        }
    }

    registry.emplace<Components::Shape>(wallEnt, Components::ShapeType::Polygon, maxR);
    registry.emplace<PolygonShape>(wallEnt, poly);
    registry.emplace<Components::AngularPosition>(wallEnt, 0.0);
    registry.emplace<Components::Color>(wallEnt, r, g, b);
}

ScenarioSystemConfig GaltonBoardScenario::getSystemsConfig() const
{
    ScenarioSystemConfig config;

    // Optimize scaling for this scenario
    config.sharedConfig.MetersPerPixel = 5e-3; // 5mm per pixel, ~5m visible vertically
    config.sharedConfig.UniverseSizeMeters =
        SimulatorConstants::ScreenLength * config.sharedConfig.MetersPerPixel;

    // Basic physics settings
    config.sharedConfig.SecondsPerTick = 1.0 / SimulatorConstants::StepsPerSecond;
    config.sharedConfig.TimeAcceleration = 1.0; // Real-time
    config.sharedConfig.GridSize = 150; // Increased from 120 for more precise collision detection
    config.sharedConfig.CellSizePixels =
        static_cast<double>(SimulatorConstants::ScreenLength) / config.sharedConfig.GridSize;

    // Optimized collision settings for Galton board behavior
    config.sharedConfig.GravitationalSoftener = 0.0; // No n-body gravity needed
    config.sharedConfig.DragCoeff = 0.15; // Increased to 0.15 for better stability
    config.sharedConfig.ParticleDensity = 600.0; // Increased from 500 for more momentum

    // Disable sleep for particles (prevents particles from stopping mid-fall)
    config.sleepConfig.linearSleepThreshold = -1.0; // Never sleep during motion
    config.sleepConfig.angularSleepThreshold = -1.0; // Never sleep during rotation

    // SPH settings (not used, but set defaults)
    config.fluidConfig.gravity = 0.0f; // No local gravity; use global system
    config.fluidConfig.restDensity = 1000.0f;
    config.fluidConfig.stiffness = 3000.0f;
    config.fluidConfig.viscosity = 0.1f;

    return config;
}

void GaltonBoardScenario::createEntities(entt::registry &registry) const
{
    // Get system configuration
    ScenarioSystemConfig scenarioSystemConfig = getSystemsConfig();
    SharedSystemConfig sharedConfig = scenarioSystemConfig.sharedConfig;
    double sizeM = sharedConfig.UniverseSizeMeters;
    
    // Extract config parameters for convenience
    const double ball_d = scenarioEntityConfig.ballDiameter;  // Ball diameter
    const double ball_r = ball_d / 2.0;                       // Ball radius
    const double wallThickness = scenarioEntityConfig.wallThickness;
    
    // Calculate board center
    double boardCenterX = sizeM * 0.5;
    double boardCenterY = sizeM * 0.5;
    
    // Calculate board dimensions from config
    double boardWidth = scenarioEntityConfig.board_width;
    double boardHeight = scenarioEntityConfig.board_height;
    
    // Calculate key vertical positions
    double boardTop = boardCenterY - boardHeight / 2.0;
    double particleStartY = boardTop + scenarioEntityConfig.particle_drop_height / 2.0;
    double funnelTopY = particleStartY + scenarioEntityConfig.particle_drop_height / 2.0;
    double funnelBottomY = funnelTopY + scenarioEntityConfig.funnel_height;
    double firstPegRowY = funnelBottomY + scenarioEntityConfig.peg_row_height / 2.0;
    
    // Calculate dimensions for funnel
    double funnelTopWidth = scenarioEntityConfig.funnel_top_width;
    double funnelExitWidth = scenarioEntityConfig.funnel_exit_width; // Actual gap size
    double funnelHeight = scenarioEntityConfig.funnel_height;
    
    // Peg grid parameters
    int pegRows = scenarioEntityConfig.pegRows;
    double pegSpacing = scenarioEntityConfig.pegSpacing;
    double rowHeight = scenarioEntityConfig.peg_row_height;
    double pegRadius = scenarioEntityConfig.pegRadius;
    
    // Bin dimensions - sized to match peg spacing at the bottom row
    double binWidth = scenarioEntityConfig.binWidth;
    int numBins = pegRows + 1; // One more bin than pegs in last row
    double binsTotalWidth = numBins * binWidth;
    double binBaseY = firstPegRowY + (pegRows - 1) * rowHeight + rowHeight / 2.0;
    double binHeight = boardHeight - (binBaseY - boardTop);
    
    std::cerr << "-- Galton Board Construction --" << std::endl;
    std::cerr << "Board dimensions: " << boardWidth << " x " << boardHeight << " m" << std::endl;
    std::cerr << "Ball diameter: " << ball_d << " m" << std::endl;
    std::cerr << "Peg spacing: " << pegSpacing << " m (should be ~2x ball diameter)" << std::endl;
    std::cerr << "Bin width: " << binWidth << " m (should be ~3x ball diameter)" << std::endl;
    std::cerr << "Funnel exit width: " << funnelExitWidth << " m (should be ~1.5x ball diameter)" << std::endl;
    
    // Create outer walls ------------------------------------------------------
    // Left wall
    makeStaticPoly(registry, boardCenterX - boardWidth / 2.0 - wallThickness / 2.0, 
                  boardCenterY,
                  {{-wallThickness / 2.0, -boardHeight / 2.0},
                   { wallThickness / 2.0, -boardHeight / 2.0},
                   { wallThickness / 2.0,  boardHeight / 2.0},
                   {-wallThickness / 2.0,  boardHeight / 2.0}},
                  scenarioEntityConfig.wallFriction, 
                  scenarioEntityConfig.wallRestitution,
                  scenarioEntityConfig.wallColorR, 
                  scenarioEntityConfig.wallColorG, 
                  scenarioEntityConfig.wallColorB);
                  
    // Right wall
    makeStaticPoly(registry, boardCenterX + boardWidth / 2.0 + wallThickness / 2.0, 
                  boardCenterY,
                  {{-wallThickness / 2.0, -boardHeight / 2.0},
                   { wallThickness / 2.0, -boardHeight / 2.0},
                   { wallThickness / 2.0,  boardHeight / 2.0},
                   {-wallThickness / 2.0,  boardHeight / 2.0}},
                  scenarioEntityConfig.wallFriction, 
                  scenarioEntityConfig.wallRestitution,
                  scenarioEntityConfig.wallColorR, 
                  scenarioEntityConfig.wallColorG, 
                  scenarioEntityConfig.wallColorB);
                  
    // Bottom floor
    makeStaticPoly(registry, boardCenterX, boardCenterY + boardHeight / 2.0 - wallThickness / 2.0,
                  {{-boardWidth / 2.0 - wallThickness, -wallThickness / 2.0},
                   { boardWidth / 2.0 + wallThickness, -wallThickness / 2.0},
                   { boardWidth / 2.0 + wallThickness,  wallThickness / 2.0},
                   {-boardWidth / 2.0 - wallThickness,  wallThickness / 2.0}},
                  scenarioEntityConfig.wallFriction, 
                  scenarioEntityConfig.wallRestitution,
                  scenarioEntityConfig.wallColorR, 
                  scenarioEntityConfig.wallColorG, 
                  scenarioEntityConfig.wallColorB);
    
    // Create funnel -----------------------------------------------------------
    // Left funnel wall
    double funnelHalfTop = funnelTopWidth / 2.0;
    double funnelHalfExit = funnelExitWidth / 2.0;
    
    std::vector<std::pair<double, double>> leftFunnelPoints = {
        {-funnelHalfTop, -funnelHeight / 2.0},                     // Top outer
        {-funnelHalfExit - wallThickness, funnelHeight / 2.0},     // Bottom outer
        {-funnelHalfExit, funnelHeight / 2.0},                     // Bottom inner
        {-funnelHalfTop + wallThickness, -funnelHeight / 2.0}      // Top inner
    };
    
    makeStaticPoly(registry, boardCenterX, funnelTopY + funnelHeight / 2.0, 
                  leftFunnelPoints,
                  0.05, // Lower friction for funnel walls
                  scenarioEntityConfig.wallRestitution,
                  scenarioEntityConfig.wallColorR, 
                  scenarioEntityConfig.wallColorG, 
                  scenarioEntityConfig.wallColorB);
    
    // Right funnel wall
    std::vector<std::pair<double, double>> rightFunnelPoints = {
        {funnelHalfTop, -funnelHeight / 2.0},                     // Top outer
        {funnelHalfExit + wallThickness, funnelHeight / 2.0},     // Bottom outer
        {funnelHalfExit, funnelHeight / 2.0},                     // Bottom inner
        {funnelHalfTop - wallThickness, -funnelHeight / 2.0}      // Top inner
    };
    
    makeStaticPoly(registry, boardCenterX, funnelTopY + funnelHeight / 2.0, 
                  rightFunnelPoints,
                  0.05, // Lower friction for funnel walls 
                  scenarioEntityConfig.wallRestitution,
                  scenarioEntityConfig.wallColorR, 
                  scenarioEntityConfig.wallColorG, 
                  scenarioEntityConfig.wallColorB);
    
    // Create pegs -------------------------------------------------------------
    for (int row = 0; row < pegRows; ++row) {
        // Each row has (row + 1) pegs
        int pegsInRow = row + 1;
        
        // Calculate row width and center it
        double rowWidth = (pegsInRow - 1) * pegSpacing;
        double rowStartX = boardCenterX - rowWidth / 2.0;
        
        // Calculate Y position
        double rowY = firstPegRowY + row * rowHeight;
        
        for (int i = 0; i < pegsInRow; ++i) {
            double pegX = rowStartX + i * pegSpacing;
            makePeg(registry, pegX, rowY, pegRadius,
                    0.05, // Reduced friction from 0.1 to 0.05 
                    scenarioEntityConfig.pegRestitution,
                    scenarioEntityConfig.pegColorR, 
                    scenarioEntityConfig.pegColorG, 
                    scenarioEntityConfig.pegColorB);
        }
    }
    
    // Create bin dividers -----------------------------------------------------
    // One more bin than pegs in last row
    double binDividerHeight = binHeight * 0.9; // Slightly shorter than total bin height
    double binStartX = boardCenterX - binsTotalWidth / 2.0;
    
    // Skip the first and last dividers as they would overlap with outer walls
    for (int i = 0; i <= numBins; ++i) {
        double dividerX = binStartX + i * binWidth;
        
        makeStaticPoly(registry, dividerX, binBaseY + binDividerHeight / 2.0,
                      {{-wallThickness / 2.0, -binDividerHeight / 2.0},
                       { wallThickness / 2.0, -binDividerHeight / 2.0},
                       { wallThickness / 2.0,  binDividerHeight / 2.0},
                       {-wallThickness / 2.0,  binDividerHeight / 2.0}},
                      scenarioEntityConfig.wallFriction, 
                      scenarioEntityConfig.wallRestitution,
                      scenarioEntityConfig.wallColorR, 
                      scenarioEntityConfig.wallColorG, 
                      scenarioEntityConfig.wallColorB);
    }
    
    // Create initial batch of particles ---------------------------------------
    std::default_random_engine generator{static_cast<unsigned int>(time(nullptr))};
    
    // Calculate the width of the funnel at the top where particles will be placed
    // Leave 3 ball diameters of space at the top of the funnel to ensure particles don't overlap walls
    double actualFunnelTopWidth = funnelTopWidth - ball_d * 3.0;
    
    // Calculate how many particles can fit horizontally across the funnel
    // Each particle needs ball_d * 1.1 space (with 10% margin)
    int particlesPerRow = static_cast<int>(actualFunnelTopWidth / (ball_d * 1.1));
    
    // Calculate how many rows of particles we can fit 
    // Each row needs ball_d * 1.1 vertical space (with 10% margin)
    double availableVerticalSpace = boardHeight * 0.2; // Use only top 20% of board for particles
    int maxRows = static_cast<int>(availableVerticalSpace / (ball_d * 1.1));
    
    // Calculate total particles we can safely place
    int totalPossibleParticles = particlesPerRow * maxRows;
    int particlesToCreate = std::min(scenarioEntityConfig.particleCount, totalPossibleParticles);
    
    std::cerr << "Funnel top width: " << funnelTopWidth << " m" << std::endl;
    std::cerr << "Actual usable width: " << actualFunnelTopWidth << " m" << std::endl;
    std::cerr << "Can fit " << particlesPerRow << " particles per row" << std::endl;
    std::cerr << "Can fit " << maxRows << " rows of particles" << std::endl;
    std::cerr << "Creating " << particlesToCreate << " particles" << std::endl;
    
    // Create particles in a proper grid layout
    int particlesCreated = 0;
    int currentRow = 0;
    
    // Place particles in the funnel, row by row
    while (particlesCreated < particlesToCreate && currentRow < maxRows) {
        // For each row, calculate number of particles to place
        int particlesInThisRow = std::min(particlesPerRow, particlesToCreate - particlesCreated);
        
        // Calculate horizontal spacing for this row
        double rowWidth = particlesInThisRow * ball_d * 1.1;
        double startX = boardCenterX - rowWidth/2.0 + ball_d * 0.55; // Start half a particle from the edge
        
        // Create particles for this row
        for (int i = 0; i < particlesInThisRow; i++) {
            auto ball = registry.create();
            
            // Position particles in a grid with slight jitter to avoid perfect alignment
            std::uniform_real_distribution<> tiny_jitter(-ball_d * 0.01, ball_d * 0.01);
            double dropX = startX + i * (ball_d * 1.1) + tiny_jitter(generator);
            
            // Position row at the top of funnel with appropriate vertical spacing
            double dropY = particleStartY - currentRow * (ball_d * 1.1) + tiny_jitter(generator);
            
            registry.emplace<Components::Position>(ball, dropX, dropY);
            
            // Zero initial velocity - let gravity do the work
            registry.emplace<Components::Velocity>(ball, 0.0, 0.0);
            
            registry.emplace<Components::Mass>(ball, scenarioEntityConfig.particleMass);
            registry.emplace<Components::ParticlePhase>(ball, Components::Phase::Solid);
            
            // Shape is perfect circle with radius = ball_r
            registry.emplace<Components::Shape>(ball, Components::ShapeType::Circle, ball_r);
            registry.emplace<CircleShape>(ball, ball_r);
            registry.emplace<Components::Material>(ball, 
                                               scenarioEntityConfig.particleFriction, 
                                               scenarioEntityConfig.particleFriction);
            
            // Angular components for rotation
            registry.emplace<Components::AngularPosition>(ball, 0.0);
            registry.emplace<Components::AngularVelocity>(ball, 0.0);
            
            // Calculate moment of inertia for a solid disk: I = 0.5 * m * r²
            double I = 0.5 * scenarioEntityConfig.particleMass * ball_r * ball_r;
            registry.emplace<Components::Inertia>(ball, I);
            
            // Orange color
            registry.emplace<Components::Color>(ball,
                                           scenarioEntityConfig.particleColorR,
                                           scenarioEntityConfig.particleColorG,
                                           scenarioEntityConfig.particleColorB);
            
            // Add Sleep component (but particles shouldn't sleep due to sleep threshold config)
            registry.emplace<Components::Sleep>(ball);
            
            particlesCreated++;
        }
        
        currentRow++;
    }
    
    std::cerr << "Galton Board created with " << particlesCreated << " particles, "
              << scenarioEntityConfig.pegRows << " peg rows, and "
              << numBins << " collection bins." << std::endl;
}