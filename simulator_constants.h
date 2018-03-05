#ifndef SIMULATOR_CONSTANTS_H
#define SIMULATOR_CONSTANTS_H

namespace SimulatorConstants {
	extern const double GravitationalConst;
	extern const double GravitationalSoftener;

  extern const double CollisionDistance;

	extern const double BarnesHutRatio;

	extern const double TimeStep; // seconds
	extern const double PixelStep; // metres
	extern const double MassStep; // kilograms

	extern const double ParticleCount;
	extern const double ParticleMassMean; // kilograms
	extern const double ParticleMassStdDev; 
	extern const double ParticleVelocityMean; // metres per second
	extern const double ParticleVelocityStdDev;

	extern const double UniverseLength; 
	extern const unsigned int ScreenLength;
	extern const unsigned int StepsPerSecond;
	extern const unsigned int Threads;
}

#endif
