/**
 * @file fluid_kernels.h
 * @brief Shared header between Metal GPU kernels and C++ fluid system
 * 
 * This file contains shared data structures and configuration parameters
 * that need to be consistent between the GPU kernels and the host-side system.
 * It is designed to be compilable by both Metal and C++ compilers.
 */

#pragma once

// Shared constants and configuration structures used by both C++ and Metal
#ifdef __METAL_VERSION__
    // Metal-specific preprocessor defines
    #define FLUID_CONSTANT constant
#else
    // C++-specific preprocessor defines
    #define FLUID_CONSTANT constexpr
#endif

/**
 * @struct FluidPositionSolverConfig
 * @brief Configuration parameters specific to the rigid-fluid position solver
 */
struct FluidPositionSolverConfig
{
    float safetyMargin;       // Separation margin for collision resolution
    float relaxFactor;        // Gradual collision correction factor
    float maxCorrection;      // Maximum position correction per frame
    float maxVelocityUpdate;  // Clamp for max velocity change 
    float minSafeDistance;    // Minimum safe distance for collision calculations
    float velocityDamping;    // Velocity damping factor for position-based dynamics
};

/**
 * @struct FluidImpulseSolverConfig
 * @brief Configuration parameters specific to the rigid-fluid impulse solver
 */
struct FluidImpulseSolverConfig
{
    float maxForce;           // Maximum force per particle
    float maxTorque;          // Maximum torque per particle
    float fluidForceScale;    // Scale factor for fluid forces
    float fluidForceMax;      // Maximum force on fluid
    float buoyancyStrength;   // Buoyancy multiplier
};

/**
 * @struct GPUFluidParams
 * @brief Parameters controlling the SPH and integration steps on the GPU.
 */
struct GPUFluidParams
{
    float cellSize;           // Size of grid cells for neighbor search
    int   gridMinX;           // Grid bounds min X
    int   gridMinY;           // Grid bounds min Y
    int   gridDimX;           // Grid dimensions X
    int   gridDimY;           // Grid dimensions Y
    float restDensity;        // Target rest density for pressure calculation
    float stiffness;          // Pressure stiffness coefficient
    float viscosity;          // Viscosity coefficient for force calculation
    float dt;                 // Current sub-step time
    float halfDt;             // Half of sub-step time (for Verlet)
    unsigned int particleCount; // Number of particles in simulation
    
    // Kernel-specific configuration
    FluidPositionSolverConfig positionSolver;  // Position solver configuration
    FluidImpulseSolverConfig  impulseSolver;   // Impulse solver configuration
}; 