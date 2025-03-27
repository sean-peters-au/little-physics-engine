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
 * @struct FluidGridConfig
 * @brief Configuration parameters related to grid assignment and particle spacing
 */
struct FluidGridConfig 
{
    float gridEpsilon;        // Small offset for grid assignment (default: 1e-6f)
    float smoothingLength; // Default smoothing length if not specified (default: 0.05f)
    float boundaryOffset;     // Offset from boundary when clamping (default: 0.001f)
};

/**
 * @struct FluidNumericalConfig
 * @brief Configuration parameters for numerical stability and thresholds
 */
struct FluidNumericalConfig
{
    float minDistanceThreshold;   // Minimum distance to avoid division by zero (default: 1e-14f)
    float minDensityThreshold;    // Minimum density to consider (default: 1e-12f)
    float minTimestep;            // Minimum timestep to use (default: 1e-10f)
    float fallbackTimestep;       // Fallback timestep if below minimum (default: 1e-4f)
};

/**
 * @struct FluidPositionSolverConfig
 * @brief Configuration parameters specific to the rigid-fluid position solver
 */
struct FluidPositionSolverConfig
{
    // Collision response
    float safetyMargin;       // Separation margin for collision resolution
    float relaxFactor;        // Gradual collision correction factor
    float maxCorrection;      // Maximum position correction per frame
    
    // Velocity update parameters
    float maxVelocityUpdate;  // Clamp for max velocity change 
    float minSafeDistance;    // Minimum safe distance for collision calculations
    float velocityDamping;    // Velocity damping for position-based dynamics
    
    // Additional parameters for stability and scale handling
    float minPositionChange;  // Minimum position change to consider for velocity update
};

/**
 * @struct FluidImpulseSolverConfig
 * @brief Configuration parameters specific to the rigid-fluid impulse solver
 */
struct FluidImpulseSolverConfig
{
    // Force limits
    float maxForce;           // Maximum force per particle
    float maxTorque;          // Maximum torque per particle
    float fluidForceScale;    // Scale factor for fluid forces
    float fluidForceMax;      // Maximum force on fluid
    
    // Physical behavior
    float buoyancyStrength;   // Buoyancy multiplier
    float viscosityScale;     // Scale applied to base viscosity
    
    // Penetration and depth effects
    float depthScale;         // Penetration depth scaling
    float depthTransitionRate; // How quickly depth effects ramp up
    float depthEstimateScale; // For hydrostatic pressure estimation
    
    // Force composition
    float pressureForceRatio; // Pressure force limit ratio
    float viscousForceRatio;  // Viscous force limit ratio
    
    // Rotation damping
    float angularDampingThreshold; // When to apply rotation damping
    float angularDampingFactor;    // Rotation damping strength
    
    // Safety thresholds 
    float maxSafeVelocitySq;  // Max safe velocity squared
    float minPenetration;     // Min penetration to consider
    float minRelVelocity;     // Min relative velocity to consider
};

/**
 * @struct GPUFluidParams
 * @brief Parameters controlling the SPH and integration steps on the GPU.
 */
struct GPUFluidParams
{
    // Grid and neighbor search
    float cellSize;           // Size of grid cells for neighbor search
    int   gridMinX;           // Grid bounds min X
    int   gridMinY;           // Grid bounds min Y
    int   gridDimX;           // Grid dimensions X
    int   gridDimY;           // Grid dimensions Y
    
    // Core physics parameters
    float gravity;            // Gravitational acceleration (m/s²)
    float restDensity;        // Target rest density for pressure calculation
    float stiffness;          // Pressure stiffness coefficient
    float viscosity;          // Viscosity coefficient for force calculation
    
    // Time step information
    float dt;                 // Current sub-step time
    float halfDt;             // Half of sub-step time (for Verlet)
    unsigned int particleCount; // Number of particles in simulation
    
    // Grid and numerical configs
    FluidGridConfig gridConfig;                // Grid-related parameters
    FluidNumericalConfig numericalConfig;      // Numerical stability parameters
    
    // Kernel-specific configuration
    FluidPositionSolverConfig positionSolver;  // Position solver configuration
    FluidImpulseSolverConfig  impulseSolver;   // Impulse solver configuration
}; 