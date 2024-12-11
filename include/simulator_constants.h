#ifndef SIMULATOR_CONSTANTS_H
#define SIMULATOR_CONSTANTS_H

namespace SimulatorConstants {
	// Simulation types for different physical scenarios
	enum class SimulationType {
		CELESTIAL_GAS,    // Gas-to-planet formation (e.g., proto-Jupiter)
		SOLAR_SYSTEM,     // Planetary orbits
		GALAXY,           // Galactic rotation
		MOLECULAR         // Molecular dynamics
	};

	// Universal constants (truly constant)
	extern const double Pi;
	extern const double RealG;  // Real gravitational constant 6.674e-11

	// Universe size in meters
	extern double UniverseSizeMeters;

	// Time acceleration factor (how much faster than real time)
	extern double TimeAcceleration;
	
	// Conversion factors
	extern double MetersPerPixel;     // How many meters one pixel represents
	extern double SecondsPerTick;     // How many real seconds each simulation tick represents

	// Simulation parameters
	extern double GravitationalSoftener;
	extern double CollisionCoeffRestitution;
	extern double DragCoeff;
	extern double ParticleDensity;

	// Grid parameters
	extern int GridSize;              // Number of cells in each dimension
	extern double CellSizePixels;     // Size of each grid cell in pixels

	// Particle generation
	extern int ParticleCount;
	extern double ParticleMassMean;
	extern double ParticleMassStdDev;
	extern double InitialVelocityFactor;  // Multiplier for calculated orbital velocities

	// Display settings (truly constant)
	extern const unsigned int ScreenLength;
	extern const unsigned int StepsPerSecond;
	extern const unsigned int Threads;

	// Initialize constants for a specific simulation type
	void initializeConstants(SimulationType type);

	// Helper functions to convert between simulation and real units
	double pixelsToMeters(double pixels);
	double metersToPixels(double meters);
	double simulationToRealTime(double ticks);
	double realToSimulationTime(double seconds);
}

#endif
