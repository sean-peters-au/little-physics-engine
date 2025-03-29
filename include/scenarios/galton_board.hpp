/**
 * @file galton_board.hpp
 * @brief Declaration of the GaltonBoardScenario class
 */

#pragma once

#include "scenarios/i_scenario.hpp"
#include "systems/shared_system_config.hpp"
#include "entt/entt.hpp"

#include <vector>
#include <utility> // For std::pair

/**
 * @struct GaltonBoardConfig
 * @brief Configuration parameters specific to the Galton board scenario
 * 
 * All measurements are carefully proportioned for clean simulation:
 * - Bins are exactly 3x particle diameter wide
 * - Pegs are positioned with peg-to-peg spacing based on ball diameter
 * - Funnel is shaped to guide particles to the central peg
 */
struct GaltonBoardConfig {
    // Base scale factor - all other parameters are derived from this
    double ballDiameter = 0.05;        // Fundamental unit for proportions
    
    // Particle (ball) parameters
    int particleCount = 55;            // Total particles to create at initialization (carefully calculated to fit)
    double particleMass = 0.05;        // Keep mass relatively low
    double particleFriction = 0.05;    // Low friction for better flow
    
    // Colors
    int particleColorR = 255;          // Orange balls
    int particleColorG = 165;
    int particleColorB = 0;
    
    // Board structure parameters
    int pegRows = 10;                  // Number of rows of pegs
    double pegRadius = 0.025;          // Increased from 0.0125 (now 1/2 of ball radius)
    double pegSpacing = 0.2;           // 2x ball diameter
    double binWidth = 0.15;            // 3x ball diameter
    double wallThickness = 0.05;       // Increased from 0.02 for better stability
    
    // Material properties
    double pegRestitution = 0.3;
    double pegFriction = 0.05;          // Reduced from 0.1 for better flow
    double wallRestitution = 0.3;
    double wallFriction = 0.05;         // Reduced from 0.1 for better flow
    
    // Colors
    int pegColorR = 120;
    int pegColorG = 120;
    int pegColorB = 120;
    int wallColorR = 80;
    int wallColorG = 80;
    int wallColorB = 80;
    
    // Computed dimensions and layout values (set by constructor)
    double funnel_top_width;          // Width of top funnel opening
    double funnel_exit_width;         // Width of funnel exit (slightly wider than ball diameter)
    double funnel_height;             // Height of funnel section
    double peg_row_height;            // Vertical spacing between peg rows
    double board_height;              // Total height of the board
    double board_width;               // Total width of the board
    double particle_drop_height;      // Initial height for particles
    
    // Set default computed values
    GaltonBoardConfig() {
        // All proportions are derived from ball diameter
        peg_row_height = ballDiameter * 3.0;     // 3x ball diameter
        funnel_exit_width = ballDiameter * 2.0;  // Increased from 1.5x to 2.0x for easier passage
        funnel_height = ballDiameter * 15.0;     // Increased to 15x for more gradual slope
        funnel_top_width = ballDiameter * 16.0;  // 16x ball diameter
        particle_drop_height = ballDiameter * 3.0; // 3x ball diameter
        
        // Board dimensions based on bin/peg layout
        int lastRowPegCount = pegRows;
        board_width = (lastRowPegCount - 1) * pegSpacing + ballDiameter * 4.0; // Padding on sides
        board_height = pegRows * peg_row_height + funnel_height + particle_drop_height + ballDiameter * 10.0;
    }
};

/**
 * @class GaltonBoardScenario
 * @brief Implements a scenario simulating a Galton board (bean machine).
 * 
 * Features:
 * - Progressive triangular grid of pegs for particle distribution
 * - Proportionally-sized collection bins aligned with the peg grid
 * - Funnel system to guide particles to the central top peg
 * - Staged particle creation to avoid instability
 */
class GaltonBoardScenario : public IScenario {
public:
    GaltonBoardScenario() = default;
    ~GaltonBoardScenario() override = default;

    /**
     * @brief Retrieves the system configuration for the scenario.
     * @return ScenarioSystemConfig including gravity and restitution settings.
     */
    ScenarioSystemConfig getSystemsConfig() const override;

    /**
     * @brief Creates the Galton board with walls, funnel, pegs, bins, and particles.
     * @param registry The entt registry to populate.
     */
    void createEntities(entt::registry &registry) const override;

private:
    /**
     * @brief Helper function to create a static polygon boundary.
     * @param registry The entt registry.
     * @param cx Center X coordinate.
     * @param cy Center Y coordinate.
     * @param localVertices Vertices relative to the center.
     * @param friction Friction coefficient.
     * @param restitution Restitution coefficient.
     * @param r Red color component.
     * @param g Green color component.
     * @param b Blue color component.
     */
    void makeStaticPoly(entt::registry &registry,
                        double cx, double cy,
                        const std::vector<std::pair<double, double>> &localVertices,
                        double friction, double restitution,
                        int r, int g, int b) const;

    /**
     * @brief Helper function to create a static peg (circle).
     * @param registry The entt registry.
     * @param cx Center X coordinate.
     * @param cy Center Y coordinate.
     * @param radius Peg radius.
     * @param friction Friction coefficient.
     * @param restitution Restitution coefficient.
     * @param r Red color component.
     * @param g Green color component.
     * @param b Blue color component.
     */
    void makePeg(entt::registry &registry,
                 double cx, double cy, double radius,
                 double friction, double restitution,
                 int r, int g, int b) const;

    GaltonBoardConfig scenarioEntityConfig;
};
